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

///////////////////////////////////////////////////////////////////////////////
//
// Defines the GaussianProcess class.
//
///////////////////////////////////////////////////////////////////////////////

#ifndef GP_PROCESS_GAUSSIAN_PROCESS_H
#define GP_PROCESS_GAUSSIAN_PROCESS_H

#include "../kernels/kernel.hpp"
#include "../utils/types.hpp"

#include <Eigen/Cholesky>
#include <glog/logging.h>
#include <vector>

namespace gp {

  class GaussianProcess {
  public:
    ~GaussianProcess() {}

    // Constructors. By default picks 10% of the maximum number of points
    // randomly within the unit box [-1, 1]^d.
    explicit GaussianProcess(const Kernel::Ptr& kernel, double noise,
                             size_t dimension, size_t max_points = 100);
    explicit GaussianProcess(const Kernel::Ptr& kernel, double noise,
                             const PointSet& points,
                             size_t max_points = 100);
    explicit GaussianProcess(const Kernel::Ptr& kernel, double noise,
                             const PointSet& points,
                             const VectorXd& targets,
                             size_t max_points = 100);

    // Evaluate mean and variance at a point.
    void Evaluate(const VectorXd& x, double& mean, double& variance) const;
    void EvaluateTrainingPoint(size_t ii, double& mean, double& variance) const;

    // Learn kernel hyperparameters by maximizing log-likelihood of the
    // training data.
    bool LearnHyperparams();

    // Immutable accessors.
    const MatrixXd& ImmutableCovariance() const { return covariance_; }
    const VectorXd& ImmutableRegressedTargets() const { return regressed_; }
    const VectorXd& ImmutableTargets() const { return targets_; }
    const ConstPointSet ImmutablePoints() const { return points_; }
    const Eigen::LLT<MatrixXd>& ImmutableCholesky() const { return llt_; }
    size_t Dimension() const { return dimension_; }

  private:
    // Compute the covariance and cross covariance against the training points.
    void Covariance();
    void CrossCovariance(const VectorXd& x, VectorXd& cross) const;

    // Kernel.
    const Kernel::Ptr kernel_;

    // Noise variance.
    const double noise_;

    // Training points, targets, and regressed targets (inv(cov) * targets).
    const PointSet points_;
    size_t dimension_;
    VectorXd targets_;
    VectorXd regressed_;

    // Maximum number of points.
    const size_t max_points_;

    // Covariance matrix, with Cholesky decomposition.
    MatrixXd covariance_;
    Eigen::LLT<MatrixXd> llt_;
  }; //\class GaussianProcess

}  //\namespace gp

#endif
