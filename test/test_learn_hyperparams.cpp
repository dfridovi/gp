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

#include <rbf_kernel.hpp>
#include <gaussian_process.hpp>
#include <types.hpp>

#include <glog/logging.h>
#include <gtest/gtest.h>
#include <random>
#include <vector>
#include <math.h>

namespace gp {

namespace test {
// A simple function (quadratic with lots of bumps) on the interval [0, 1].
double f(double x) {
  return (x - 0.5) * (x - 0.5) + 0.1 * std::sin(2.0 * M_PI * 10.0 * x);
}

// Sample points from a simple function and fit a GP model. Make sure that
// after learning hyperparameters for an RBF kernel, the GP improves its
// root mean squared error against a random set of points.
TEST(GaussianProcess, TestLearnHyperparams) {
  const size_t kNumTrainingPoints = 10;
  const size_t kNumTestPoints = 100;
  const double kMaxRmsError = 0.01;
  const double kNoiseVariance = 0.0001;

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
    targets(ii) = f(random_point(0));
  }

  // Train a GP.
  const Kernel::Ptr kernel = RbfKernel::Create(VectorXd::Constant(1, 1.0));
  GaussianProcess gp(kernel, kNoiseVariance, points, targets,
                     kNumTrainingPoints);

  EXPECT_TRUE(gp.LearnHyperparams());

  std::cout << "Length: " << kernel->ImmutableParams().transpose() << std::endl;

  // Test that we have approximated the function well.
  double squared_error = 0.0;
  double mean, variance;
  for (size_t ii = 0; ii < kNumTestPoints; ii++) {
    VectorXd test_point(1);
    test_point(0) = unif(rng);

    gp.Evaluate(test_point, mean, variance);
    squared_error += (mean - f(test_point(0))) * (mean - f(test_point(0)));
  }

  EXPECT_LE(std::sqrt(squared_error / static_cast<double>(kNumTestPoints)),
            kMaxRmsError);
}

} //\namespace test

} //\namespace gp
