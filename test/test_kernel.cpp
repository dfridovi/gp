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
#include <types.hpp>

#include <glog/logging.h>
#include <gtest/gtest.h>
#include <random>
#include <vector>
#include <math.h>

namespace gp {

namespace test {

// Make sure that the kernel partial derivatives are correct.
TEST(RbfKernel, TestPartials) {
  const double kMaxError = 1e-8;
  const double kEpsilon = 1e-8;
  const size_t kDimension = 10;
  const size_t kNumTests = 10;

  // Random number generator.
  std::random_device rd;
  std::default_random_engine rng(rd());
  std::uniform_real_distribution<double> unif(0.0, 1.0);

  // Create a kernel.
  VectorXd lengths(kDimension);
  for (size_t ii = 0; ii < kDimension; ii++)
    lengths(ii) = unif(rng);

  const Kernel::Ptr kernel = RbfKernel::Create(lengths);

  // Check that the partial derivative in each dimension is correct.
  for (size_t ii = 0; ii < kDimension; ii++) {
    for (size_t jj = 0; jj < kNumTests; jj++) {
      const VectorXd x = VectorXd::Random(kDimension);
      const VectorXd y = VectorXd::Random(kDimension);

      // Compute analytic derivative.
      const double analytic = kernel->Partial(x, y, ii);

      // Compute numerical derivative.
      kernel->Adjust(kEpsilon, ii);
      const double forward = kernel->Evaluate(x, y);

      kernel->Adjust(-2.0 * kEpsilon, ii);
      const double backward = kernel->Evaluate(x, y);

      kernel->Adjust(kEpsilon, ii);
      EXPECT_NEAR(analytic, (forward - backward) / (2.0 * kEpsilon), kMaxError);
    }
  }
}

} //\namespace test

} //\namespace gp
