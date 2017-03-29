/*
 * Copyright (c) 2016, The Regents of the University of California (Regents).
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
///////////////////////////////////////////////////////////////////////////////
//
// Cost functors for Ceres.
//
///////////////////////////////////////////////////////////////////////////////

#ifndef GP_COST_FUNCTORS_H
#define GP_COST_FUNCTORS_H

#include "gaussian_process.hpp"
#include "kernel.hpp"

#include <ceres/ceres.h>
#include <glog/logging.h>
#include <math.h>

namespace gp {

  // Compute twice the negative log-likelihood of the training data of a GP
  // and the gradient against all the parameters of the kernel. Since this is
  // set up as a least squares problem, we actually compute residuals. Each
  // training point contributes two residuals: (1) is from the normalization
  // (related to its variance) and (2) is essentially the z-score.
  struct TrainingLogLikelihood {
    // Inputs: training points, training targets, and initial kernel.
    // Optimization variables: kernel parameters.
    const std::vector<VectorXd>* points_;
    const VectorXd* targets_;
    const Kernel::Ptr kernel_;

    TrainingLogLikelihood(const std::vector<VectorXd>* points,
                          const VectorXd* targets,
                          const Kernel::Ptr& kernel)
      : points_(points),
        targets_(targets),
        kernel_(kernel) {
      CHECK_NOTNULL(points_);
      CHECK_NOTNULL(targets_);
      CHECK_NOTNULL(kernel_.get());
    }

    template <typename T>
    bool operator()(T const* const* belief, T* expected_error) const {
      // Compute the difference between the expected measurement and the
      // actual measurement.
      *expected_error = -static_cast<T>(measurement_);

      for (const auto& voxel_id : *voxels_) {
        *expected_error += belief[0][voxel_id];
      }

      return true;
    }

    // Factory method.
    static ceres::CostFunction* Create(unsigned int num_rows,
                                       unsigned int num_cols,
                                       const std::vector<unsigned int>* voxels,
                                       const unsigned int measurement) {
      // Only a single residual.
      const int kNumResiduals = 1;

      // Number of parameters is the number of grid cells.
      const int kNumParameters = static_cast<int>(num_rows * num_cols);

      // Stride. Number of derivatives to calculate. See below for details:
      // http://ceres-solver.org/nnls_modeling.html#dynamicautodiffcostfunction
      const int kStride = 4;

      ceres::DynamicAutoDiffCostFunction<BeliefError, kStride>* cost =
        new ceres::DynamicAutoDiffCostFunction<BeliefError, kStride>(
                                                                     new BeliefError(voxels, measurement));
      cost->AddParameterBlock(kNumParameters);
      cost->SetNumResiduals(kNumResiduals);
      return cost;
    }
  }; // struct TrainingLogLikelihood

} // namespace gp

#endif
