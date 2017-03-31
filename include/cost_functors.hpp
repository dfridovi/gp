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
  // and the gradient against all the parameters of the kernel.
  class TrainingLogLikelihood : public ceres::FirstOrderFunction {
  public:
    // Inputs: training points, training targets, kernel, and noise.
    // Optimization variables: kernel parameters.
    TrainingLogLikelihood(const PointSet& points,
                          const VectorXd* targets,
                          const Kernel::Ptr& kernel,
                          double noise)
      : points_(points),
        targets_(targets),
        kernel_(kernel),
        noise_(noise) {
      CHECK_NOTNULL(targets);
      CHECK_NOTNULL(points.get());
      CHECK_NOTNULL(kernel.get());

      CHECK_EQ(points->size(), targets->size());
      CHECK_GE(points->size(), 1);
      CHECK_GT(noise, 0.0);
    }

    // Evaluate objective function and gradient. For details please see
    // R&W, pg. 113/4, eqs. 5.8/9. For simplicity we leave off the constant.
    bool Evaluate(const double* const parameters,
                  double* cost, double* gradient) const {
      // Update the kernel.
      for (size_t ii = 0; ii < NumParameters(); ii++)
        kernel_->Params()(ii) = parameters[ii];

      // Create a new GP model and extract computed variables.
      GaussianProcess gp(kernel_, noise_, points_, *targets_, points_->size());
      const Eigen::LLT<MatrixXd>& llt = gp.ImmutableCholesky();
      const VectorXd& regressed = gp.ImmutableRegressedTargets();
      const MatrixXd& L = llt.matrixL();

      // Compute log det of covariance matrix.
      double logdet = 0.0;
      for (size_t ii = 0; ii < L.rows(); ii++)
        logdet += std::log(L(ii, ii));

      logdet *= 2.0;

      // Evaluate cost.
      *cost = targets_->dot(regressed) + logdet;

      // Maybe compute gradient.
      if (gradient) {
        MatrixXd dK(points_->size(), points_->size());

        for (size_t ii = 0; ii < NumParameters(); ii++) {
          // Compute the derivative of covariance against the ii'th parameter.
          for (size_t jj = 0; jj < points_->size(); jj++) {
            dK(jj, jj) = kernel_->Partial(points_->at(jj), points_->at(jj), ii);

            for (size_t kk = 0; kk < jj; kk++) {
              dK(jj, kk) =
                kernel_->Partial(points_->at(jj), points_->at(kk), ii);
              dK(kk, jj) = dK(jj, kk);
            }
          }

          gradient[ii] = llt.solve(dK).trace() - regressed.dot(dK * regressed);
        }
      }

      return true;
    }

    // Number of parameters in the problem.
    int NumParameters() const {
      return static_cast<int>(kernel_->ImmutableParams().size());
    }

  private:
    // Inputs: training points, training targets, kernel, and noise.
    // Optimization variables: kernel parameters.
    const PointSet points_;
    const VectorXd* targets_;
    const Kernel::Ptr kernel_;
    const double noise_;
  }; // struct TrainingLogLikelihood

} // namespace gp

#endif
