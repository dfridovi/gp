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
// Interfaces with the matplotpp library to plot a 1D GP with 2-sigma bounds.
//
///////////////////////////////////////////////////////////////////////////////

#ifndef GP_UTILS_PLOT_1D_H
#define GP_UTILS_PLOT_1D_H

#include <process/gaussian_process.hpp>

#include <matplotpp/matplotpp.h>
#include <glog/logging.h>
#include <vector>
#include <math.h>

namespace gp {

  class Plot1D : public MatPlot {
  public:
    ~Plot1D() {}
    Plot1D(const GaussianProcess* const gp, double xmin, double xmax,
           double ymin, double ymax, size_t num_points = 100,
           const std::string& title = "", const std::string& xlabel = "",
           const std::string& ylabel = "")
      : gp_(gp),
        xmin_(xmin), xmax_(xmax),
        ymin_(ymin), ymax_(ymax),
        num_points_(num_points),
        title_(title),
        xlabel_(xlabel),
        ylabel_(ylabel),
        MatPlot() {
      CHECK_NOTNULL(gp);
      CHECK_EQ(gp_->Dimension(), 1);
    }

    // Implement pure virtual display function.
    void DISPLAY() {
      // Sample the x axis.
      const std::vector<double> x = linspace(xmin_, xmax_, num_points_);

      // Three functions to compute: mean, upper bound, lower bound.
      std::vector<double> mu(num_points_);
      std::vector<double> upper(num_points_);
      std::vector<double> lower(num_points_);

      double mean, variance;
      VectorXd test_point(1);
      for (size_t ii = 0; ii < num_points_; ii++) {
        // Compute the GP mean and variance here.
        test_point(0) = x[ii];
        gp_->Evaluate(test_point, mean, variance);

        // Populate mean, upper, lower.
        mu[ii] = mean;
        upper[ii] = mean + 2.0 * std::sqrt(variance);
        lower[ii] = mean - 2.0 * std::sqrt(variance);
      }

      // Plot.
      plot(x, mu); set(2); set("b");
      plot(x, upper); set(2); set("r"); set(":");
      plot(x, lower); set(2); set("r"); set(":");

      title(title_);
      xlabel(xlabel_);
      ylabel(ylabel_);
    }

  private:
    // Gaussian process pointer.
    const GaussianProcess* const gp_;

    // Plot params.
    const double xmin_, xmax_, ymin_, ymax_;
    const size_t num_points_;

    // Title and labels.
    const std::string title_, xlabel_, ylabel_;
  }; //\ class Plot1D

}  //\namespace gp

#endif
