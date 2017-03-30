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
    // Inputs: training points, training targets, kernel, and noise.
    // Optimization variables: kernel parameters.
    const PointSet points_;
    const VectorXd* targets_;
    const Kernel::Ptr kernel_;
    const double noise_;

    TrainingLogLikelihood(const PointSet& points,
                          const VectorXd* targets,
                          const Kernel::Ptr& kernel,
                          double noise)
      : points_(points),
        targets_(targets),
        kernel_(kernel),
        noise_(noise) {}

    template <typename T>
    bool operator()(T const* const* params, T* resids) const {
      // First, create a new GP model using this set of parameters.
      // Parameters are already stored in the kernel!
      GaussianProcess gp(kernel_, noise_,
                         points_, *targets_, targets_->size());

      // Compute residuals. As above, each training point contributes
      // two residuals:
      // (1) sqrt(log(2pi * var(points_[ii])))
      // (2) (mu(points_[ii]) - targets_[ii]) / sqrt(var(points_[ii]))
      double mean, variance;
      for (size_t ii = 0; ii < points_->size(); ii++) {
        gp.EvaluateTrainingPoint(ii, mean, variance);
        resids[2 * ii] += static_cast<T>(std::sqrt(std::log(2.0 * M_PI * variance)));
        resids[2 * ii + 1] = static_cast<T>((mean - targets_->operator()(ii)) /
                                            std::sqrt(variance));
      }

      return true;
    }

    // Factory method.
    static ceres::CostFunction* Create(const PointSet& points,
                                       const VectorXd* targets,
                                       const Kernel::Ptr& kernel,
                                       double noise) {
      CHECK_NOTNULL(targets);
      CHECK_NOTNULL(points.get());
      CHECK_NOTNULL(kernel.get());

      CHECK_EQ(points->size(), targets->size());
      CHECK_GE(points->size(), 1);
      CHECK_GT(noise, 0.0);

      // Number of residuals is twice the number of points.
      const int kNumResiduals = 2 * points->size();

      // Number of parameters is given by the kernel.
      const int kNumParameters = static_cast<int>(kernel->Params().size());

      // Stride. Number of derivatives to calculate. See below for details:
      // http://ceres-solver.org/nnls_modeling.html#dynamicautodiffcostfunction
      const int kStride = 4;

      ceres::DynamicAutoDiffCostFunction<TrainingLogLikelihood, kStride>* cost =
        new ceres::DynamicAutoDiffCostFunction<TrainingLogLikelihood, kStride>(
          new TrainingLogLikelihood(points, targets, kernel, noise));
      cost->AddParameterBlock(kNumParameters);
      cost->SetNumResiduals(kNumResiduals);
      return cost;
    }
  }; // struct TrainingLogLikelihood

} // namespace gp

#endif
