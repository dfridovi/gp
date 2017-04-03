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
DECLARE_bool(verbose);

namespace gp {
namespace test {

// Check that we can approximate a simple 1D function.
TEST(GaussianProcess, TestFunctionApprox1D) {
  const size_t kNumTrainingPoints = 100;
  const size_t kNumTestPoints = 100;
  const double kMaxRmsError = 0.01;
  const double kNoiseVariance = 1e-4;
  const double kLength = 0.1;

  const size_t kBatchSize = 16;
  const size_t kGradUpdates = 1000;
  const double kStepSize = 0.5;

  // Random number generator.
  std::random_device rd;
  std::default_random_engine rng(rd());
  std::uniform_real_distribution<double> unif(0.0, 1.0);

  // Get training points/targets.
  PointSet points(new std::vector<VectorXd>);
  VectorXd targets(kNumTrainingPoints);

  for (size_t ii = 0; ii < kNumTrainingPoints; ii++) {
    points->push_back(VectorXd::Constant(1, unif(rng)));
    targets(ii) = unif(rng);;
  }

  // Train a GP.
  const Kernel::Ptr kernel = RbfKernel::Create(VectorXd::Constant(1, kLength));
  GaussianProcess gp(kernel, kNoiseVariance, points, targets,
                     kNumTrainingPoints);

  // Run gradient descent.
  std::vector<VectorXd> batch_points;
  std::vector<double> batch_targets;
  double mse = std::numeric_limits<double>::infinity();
  for (size_t ii = 0; ii < kGradUpdates; ii++) {
    batch_points.clear();
    batch_targets.clear();

    // Get a random batch.
    for (size_t jj = 0; jj < kBatchSize; jj++) {
      const double x = unif(rng);
      batch_points.push_back(VectorXd::Constant(1, x));
      batch_targets.push_back(BumpyParabola(x));
    }

    // Update parameters.
    mse = gp.UpdateTargets(batch_points, batch_targets,
                           kStepSize, ii == kGradUpdates - 1);

    if (FLAGS_verbose)
      std::printf("MSE at step %zu was %5.3f.\n", ii, mse);
  }

  // Test that we have approximated the function well.
  double squared_error = 0.0;
  double mean, variance;
  for (size_t ii = 0; ii < kNumTestPoints; ii++) {
    const double x = unif(rng);

    gp.Evaluate(VectorXd::Constant(1, x), mean, variance);
    squared_error += (mean - BumpyParabola(x)) * (mean - BumpyParabola(x));
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
