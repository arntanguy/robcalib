// Copyright 2017-2018 CNRS-UM LIRMM
// Copyright 2017-2018 Arnaud TANGUY <arnaud.tanguy@lirmm.fr>
//
// This file is part of robcalib.
//
// robcalib is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// robcalib is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// along with robcalib.  If not, see <http://www.gnu.org/licenses/>.


#pragma once

#include <roboptim/core/linear-function.hh>
#include <roboptim/core/differentiable-function.hh>
#include <roboptim/core/sum-of-c1-squares.hh>
#include <roboptim/core/decorator/finite-difference-gradient.hh>
#include <roboptim/core/function.hh>

#include <robcalib/logging.h>
#include <sophus/se3.hpp>

namespace robcalib
{
using Sophus::SE3d;

/**
 * @brief Computes the Link-Eye problem using the analytical jacobian.
 * This cost function is suited a non-linear least square algorithm.
 *
 * To construct the problem, use the following:
 *
 * auto fd = boost::shared_ptr<CostLinkEyeSE3Analytical>(new CostLinkEyeSE3Analytical(eye, link));
 * // Compute sum of C1 squares on function
 *  SumOfC1Squares fSum(fd, "some function");
 *
 *  And solve over fSum.
 */
struct CostLinkEyeSE3: public roboptim::DifferentiableFunction
{
  std::vector<SE3d> eye_;
  std::vector<SE3d> link_;

  void set_link_eye(const std::vector<SE3d>& eye, const std::vector<SE3d>& link)
  {
    assert(eye.size() == link.size());
    this->eye_ = eye;
    this->link_ = link;
  }

  CostLinkEyeSE3(const std::vector<SE3d>& eye, const std::vector<SE3d>& link)
      : roboptim::DifferentiableFunction(6, 16 * eye.size(),
                                         "CostLinkEyeSE3Analytical")
  {
    LOG(info) << "Input: " << this->inputSize() << ", Output: " << this->outputSize() << std::endl;
    set_link_eye(eye, link);
  }

  void impl_compute(result_ref result, const_argument_ref x) const override
  {
    // Exponential map of the current twist
    const SE3d::Tangent t(x);
    const SE3d X = SE3d::exp(t);
    result.setZero();

    const auto k = eye_.size();
    for (size_t i = 0; i < k; ++i)
    {
      // Residual
      auto r = (eye_[i] * X).matrix() - (X * link_[i]).matrix();
      for (int j = 0; j < 16; ++j)
      {
        result[i * 16 + j] = r.matrix()(j);
      }
    }
  }

  /**
   * @brief The gradient for Link Eye Calibration
   *
   * @param grad
   * @param const_argument_ref
   * @param functionId
   * The current output of the function, ie output corresponding to a "line of the jacobian"
   *
   *  XXX this is the jacobian at identity, should't it be jacobian at current
   *  solution?
   */
  void impl_gradient(gradient_ref grad, const_argument_ref, size_type functionId) const override
  {
    grad.setZero();
    // Function id is a line of the jacobian.

    // Jacobian Ji
    const auto i = functionId / 16;
    const auto& a = eye_[i].matrix();
    const auto& b = link_[i].matrix();

    switch (functionId % 16)
    {
      // Each case is a derivation variable
      case 0:
        grad << 0, 0, 0, 0, -a(0, 2) - b(2, 0), a(0, 1) + b(1, 0);
        break;
      case 1:
        grad << 0, 0, 0, b(2, 0), -a(1, 2), a(1, 1) - b(0, 0);
        break;
      case 2:
        grad << 0, 0, 0, -b(1, 0), b(0, 0) - a(2, 2), a(2, 1);
        break;
      case 3:
        grad << 0, 0, 0, 0, 0, 0;
        break;
      case 4:
        grad << 0, 0, 0, a(0, 2), -b(2, 1), b(1, 1) - a(0, 0);
        break;
      case 5:
        grad << 0, 0, 0, a(1, 2) + b(2, 1), 0, -a(1, 0) - b(0, 1);
        break;
      case 6:
        grad << 0, 0, 0, a(2, 2) - b(1, 1), b(0, 1), -a(2, 0);
        break;
      case 7:
        grad << 0, 0, 0, 0, 0, 0;
        break;
      case 8:
        grad << 0, 0, 0, -a(0, 1), a(0, 0) - b(2, 2), b(1, 2);
        break;
      case 9:
        grad << 0, 0, 0, b(2, 2) - a(1, 1), a(1, 0), -b(0, 2);
        break;
      case 10:
        grad << 0, 0, 0, -a(2, 1) - b(1, 2), a(2, 0) + b(0, 2), 0;
        break;
      case 11:
        grad << 0, 0, 0, 0, 0, 0;
        break;
      case 12:
        grad << a(0, 0) - 1, a(0, 1), a(0, 2), 0, -b(2, 3), b(1, 3);
        break;
      case 13:
        grad << a(1, 0), a(1, 1) - 1, a(1, 2), b(2, 3), 0, -b(0, 3);
        break;
      case 14:
        grad << a(2, 0), a(2, 1), a(2, 2) - 1, -b(1, 3), b(0, 3), 0;
        break;
      case 15:
        grad << 0, 0, 0, 0, 0, 0;
        break;

      default:
        abort();
    }
  }
};

} /* robcalib */
