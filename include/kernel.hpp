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
// Defines the Kernel base class. The kernel is what tells the GP how much
// the underyling function value depends on each training point. Common kernels
// include the squared exponential or RBF and the Matern class.
//
///////////////////////////////////////////////////////////////////////////////

#ifndef GP_KERNEL_H
#define GP_KERNEL_H

#include "types.hpp"

#include <glog/logging.h>
#include <memory>

namespace gp {

  class Kernel {
  public:
    // Typedefs.
    typedef std::shared_ptr<Kernel> Ptr;
    typedef std::shared_ptr<const Kernel> ConstPtr;

    // Destructor.
    virtual ~Kernel() {}

    // Pure virtual methods to be implemented in a derived class.
    virtual double Evaluate(const VectorXd& x, const VectorXd& y) const = 0;
    virtual double Partial(const VectorXd& x, const VectorXd& y,
                           size_t ii) const = 0;
    virtual void Gradient(const VectorXd& x, const VectorXd& y,
                          VectorXd& gradient) const = 0;

    // Access and reset params.
    VectorXd& Params() { return params_; }
    const VectorXd& ImmutableParams() const { return params_; }
    void Reset(const VectorXd& params) {
      CHECK_EQ(params_.size(), params.size());
      params_ = params;
    }

  protected:
    explicit Kernel(const VectorXd& params)
      : params_(params) {}

    // Parameter vector.
    VectorXd params_;
  }; //\class Kernel

}  //\namespace gp

#endif
