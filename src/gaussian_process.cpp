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

#include <gaussian_process.hpp>
#include <cost_functors.hpp>

#include <ceres/ceres.h>
#include <random>

namespace gp {

  GaussianProcess::GaussianProcess(const Kernel::Ptr& kernel, double noise,
                                   size_t dimension, size_t max_points)
    : kernel_(kernel),
      noise_(noise),
      max_points_(max_points),
      targets_(max_points),
      regressed_(max_points),
      covariance_(max_points, max_points) {
    CHECK_NOTNULL(kernel_.get());
    CHECK_GE(max_points_, 1);
    CHECK_GE(dimension, 1);
    CHECK_GT(noise_, 0.0);

    // Random number generator.
    std::random_device rd;
    std::default_random_engine rng(rd());
    std::uniform_real_distribution<double> unif(-1.0, 1.0);
    std::normal_distribution<double> normal(0.0, 0.1);

    // Populate 'points_' and 'targets_'.
    for (size_t ii = 0; ii < max_points / 10 + 1; ii++) {
      VectorXd x(dimension);

      for (size_t jj = 0; jj < dimension; jj++)
        x(jj) = unif(rng);

      points_.push_back(x);
      targets_(ii) = normal(rng);
    }

    // Compute covariance matrix.
    Covariance();

    // Compute Cholesky decomposition of covariance matrix.
    llt_.compute(covariance_.topLeftCorner(points_.size(), points_.size()));

    // Compute regressed targets.
    regressed_.head(points_.size()) =
      llt_.solve(targets_.head(points_.size()));
  }

  GaussianProcess::GaussianProcess(const Kernel::Ptr& kernel, double noise,
                                   const std::vector<VectorXd>& points,
                                   size_t max_points)
    : kernel_(kernel),
      noise_(noise),
      max_points_(max_points),
      targets_(max_points),
      regressed_(max_points),
      points_(points),
      covariance_(max_points, max_points) {
    CHECK_NOTNULL(kernel_.get());
    CHECK_GE(max_points_, 1);
    CHECK_LE(points_.size(), max_points_);
    CHECK_GT(noise_, 0.0);

    // Random number generator.
    std::random_device rd;
    std::default_random_engine rng(rd());
    std::normal_distribution<double> normal(0.0, 0.1);

    // Populate 'targets_'.
    for (size_t ii = 0; ii < points_.size(); ii++)
      targets_(ii) = normal(rng);

    // Compute covariance matrix.
    Covariance();

    // Compute Cholesky decomposition of covariance matrix.
    llt_.compute(covariance_.topLeftCorner(points_.size(), points_.size()));

    // Compute regressed targets.
    regressed_.head(points_.size()) =
      llt_.solve(targets_.head(points_.size()));
  }

  GaussianProcess::GaussianProcess(const Kernel::Ptr& kernel, double noise,
                                   const std::vector<VectorXd>& points,
                                   const VectorXd& targets,
                                   size_t max_points)
    : kernel_(kernel),
      noise_(noise),
      max_points_(max_points),
      targets_(max_points),
      regressed_(max_points),
      points_(points),
      covariance_(max_points, max_points) {
    CHECK_NOTNULL(kernel_.get());
    CHECK_GE(max_points_, 1);
    CHECK_LE(points_.size(), max_points_);
    CHECK_EQ(targets.size(), points_.size());
    CHECK_GT(noise_, 0.0);

    // Set 'targets_'.
    targets_.head(points_.size()) = targets;

    // Compute covariance matrix.
    Covariance();

    // Compute Cholesky decomposition of covariance matrix.
    llt_.compute(covariance_.topLeftCorner(points_.size(), points_.size()));

    // Compute regressed targets.
    regressed_.head(points_.size()) =
      llt_.solve(targets_.head(points_.size()));
  }

  // Evaluate mean and variance at a point.
  void GaussianProcess::Evaluate(const VectorXd& x,
                                 double& mean, double& variance) const {
    // Compute cross covariance.
    VectorXd cross(points_.size());
    CrossCovariance(x, cross);

    // Compute mean and variance.
    mean = cross.dot(regressed_);
    variance = 1.0 - cross.dot(llt_.solve(cross));
  }

  // Evaluate at the ii'th training point.
  void GaussianProcess::EvaluateTrainingPoint(
     size_t ii, double& mean, double& variance) const {
    CHECK_LT(ii, points_.size());

    // Extract cross covariance (must subtract off added noise).
    VectorXd cross = covariance_.col(ii);
    cross(ii) -= noise_;

    // Compute mean and variance.
    mean = cross.dot(regressed_);
    variance = 1.0 - cross.dot(llt_.solve(cross));
  }

  // Learn kernel hyperparameters by maximizing the log-likelihood of the
  // training data.
  bool GaussianProcess::LearnHyperparams() {
    // Create a Ceres problem.
    ceres::Problem problem;
    problem.AddResidualBlock(
      TrainingLogLikelihood::Create(&points_, &targets_, kernel_, noise_),
      NULL, // Squared loss (no outlier rejection).
      kernel_->Params().data()); // Direct access to kernel params.

    // Assume all parameters are positive. This can be removed later,
    // and at least for the RBF kernel it really shouldn't matter.
    for (size_t ii = 0; ii < kernel_->Params().size(); ii++)
      problem.SetParameterLowerBound(kernel_->Params().data(), ii, 0.0);

    // Set up solver options.
    ceres::Solver::Summary summary;
    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_QR;
    options.function_tolerance = 1e-16;
    options.gradient_tolerance = 1e-16;
    options.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT;

    // Solve and return. Solution parameters are automatically stored in
    // the kernel!
    ceres::Solve(options, &problem, &summary);

    return summary.IsSolutionUsable();
  }

  // Compute the covariance and cross covariance against the training points.
  void GaussianProcess::Covariance() {
    for (size_t ii = 0; ii < points_.size(); ii++) {
      covariance_(ii, ii) = 1.0 + noise_;

      for (size_t jj = 0; jj < ii; jj++) {
        covariance_(ii, jj) = kernel_->Evaluate(points_[ii], points_[jj]);
        covariance_(jj, ii) = covariance_(ii, jj);
      }
    }
  }

  void GaussianProcess::CrossCovariance(const VectorXd& x, VectorXd& cross) const {
    for (size_t ii = 0; ii < points_.size(); ii++)
      cross(ii) = kernel_->Evaluate(points_[ii], x);
  }
}  //\namespace gp
