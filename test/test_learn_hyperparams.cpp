/*
 * Copyright (c) 2017, The Regents of the University of California (Regents).
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *
 *    1. Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *
 *    2. Redistributions in binary form must reproduce the above
 *       copyright notice, this list of conditions and the following
 *       disclaimer in the documentation and/or other materials provided
 *       with the distribution.
 *
 *    3. Neither the name of the copyright holder nor the names of its
 *       contributors may be used to endorse or promote products derived
 *       from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS AS IS
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * Please contact the author(s) of this library if you have any questions.
 * Authors: David Fridovich-Keil   ( dfk@eecs.berkeley.edu )
 */

#include <kernels/rbf_kernel.hpp>
#include <process/gaussian_process.hpp>
#include <optimization/cost_functors.hpp>
#include <utils/types.hpp>
#include <utils/plot_1d.hpp>

#include "test_functions.hpp"
#include "test_plotting.hpp"

#include <glog/logging.h>
#include <gtest/gtest.h>
#include <gflags/gflags.h>
#include <random>
#include <vector>
#include <math.h>


#ifdef SYSTEM_OSX
#include <GLUT/glut.h>
#endif

#ifdef SYSTEM_LINUX
//#include <GL/glew.h>
#include <GL/glut.h>
#endif

DECLARE_bool(visualize);

namespace gp {
namespace test {

// Check that the gradient computation in TrainingLogLikelihood is correct.
TEST(TrainingLogLikelihood, TestGradient) {
  const size_t kDimension = 10;
  const size_t kNumTrainingPoints = 10;
  const size_t kNumTests = 100;
  const double kEpsilon = 1e-8;
  const double kMaxError = 1e-4;
  const double kNoiseVariance = 0.1;

  // Random number generator.
  std::random_device rd;
  std::default_random_engine rng(rd());
  std::uniform_real_distribution<double> unif(0.0, 1.0);

  // Get training points/targets.
  PointSet points(new std::vector<VectorXd>);
  VectorXd targets(kNumTrainingPoints);

  for (size_t ii = 0; ii < kNumTrainingPoints; ii++) {
    points->push_back(VectorXd::Random(kDimension));
    targets(ii) = unif(rng);
  }

  // Create a kernel.
  VectorXd lengths(kDimension);
  for (size_t ii = 0; ii < kDimension; ii++)
    lengths(ii) = 1.0 + unif(rng);

  const Kernel::Ptr kernel = RbfKernel::Create(lengths);

  // Create a TrainingLogLikelihood.
  TrainingLogLikelihood cost(points, &targets, kernel, kNoiseVariance);

  // Test that the analytic and numerical derivatives match at a bunch of
  // different length vectors.
  double parameters[kDimension];
  double gradient[kDimension];
  for (size_t ii = 0; ii < kNumTests; ii++) {
    // Pick a random set of length vectors.
    for (size_t jj = 0; jj < kDimension; jj++)
      parameters[jj] = 1.0 + unif(rng);

    // Evaluate objective and gradient.
    double objective;
    EXPECT_TRUE(cost.Evaluate(parameters, &objective, gradient));

    // Evaluate numerical gradient with a forward difference.
    for (size_t jj = 0; jj < kDimension; jj++) {
      double forward;
      parameters[jj] += kEpsilon;
      EXPECT_TRUE(cost.Evaluate(parameters, &forward, NULL));
      parameters[jj] -= kEpsilon;

      // Check that analytic and numeric derivatives agree.
      EXPECT_NEAR(gradient[jj], (forward - objective) / kEpsilon, kMaxError);
    }
  }
}

// Sample points from a simple function and fit a GP model. Make sure that
// after learning hyperparameters for an RBF kernel, the GP improves its
// root mean squared error against a random set of points.
TEST(GaussianProcess, TestLearnHyperparams) {
  const size_t kNumTrainingPoints = 100;
  const size_t kNumTestPoints = 100;
  const double kMaxRmsError = 0.01;
  const double kNoiseVariance = 1e-4;

  // Random number generator.
  std::random_device rd;
  std::default_random_engine rng(rd());
  std::uniform_real_distribution<double> unif(0.0, 1.0);

  // Get training points/targets.
  PointSet points(new std::vector<VectorXd>);
  VectorXd targets(kNumTrainingPoints);

  for (size_t ii = 0; ii < kNumTrainingPoints; ii++) {
    VectorXd random_point(1);
    random_point(0) = unif(rng);

    points->push_back(random_point);
    targets(ii) = BumpyParabola(random_point(0));
  }

  // Train a GP.
  const Kernel::Ptr kernel = RbfKernel::Create(VectorXd::Constant(1, 1.0));
  GaussianProcess gp(kernel, kNoiseVariance, points, targets,
                     kNumTrainingPoints);

  EXPECT_TRUE(gp.LearnHyperparams());

  // Test that we have approximated the function well.
  double squared_error = 0.0;
  double mean, variance;
  for (size_t ii = 0; ii < kNumTestPoints; ii++) {
    VectorXd test_point(1);
    test_point(0) = unif(rng);

    gp.Evaluate(test_point, mean, variance);
    squared_error +=
      (mean - BumpyParabola(test_point(0))) *
      (mean - BumpyParabola(test_point(0)));
  }

  EXPECT_LE(std::sqrt(squared_error / static_cast<double>(kNumTestPoints)),
            kMaxRmsError);

  // Maybe visualize.
  if (FLAGS_visualize) {
    // Create a new plot.
    plotting::plot = new Plot1D(&gp, 0.0, 1.0, -0.3, 0.3, 1000,
                                "Learned Hyperparameters");

    // Visualize.
    glutCreateWindow(100, 100, 400, 300);
    glutDisplayFunc(plotting::display);
    glutReshapeFunc(plotting::reshape);
    glutIdleFunc(plotting::idle);
    glutMotionFunc(plotting::motion);
    glutMouseFunc(plotting::mouse);
    glutPassiveMotionFunc(plotting::passive);
    glutKeyboardFunc(plotting::keyboard);
    glutMainLoop();

    delete plotting::plot;
  }
}

} //\namespace test
} //\namespace gp
