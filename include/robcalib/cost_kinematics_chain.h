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

#include <robcalib/calib_data.h>
#include <robcalib/maths.h>
#include <robcalib/logging.h>
#include <roboptim/core/decorator/finite-difference-gradient.hh>
#include <roboptim/core/differentiable-function.hh>
#include <roboptim/core/function.hh>
#include <roboptim/core/linear-function.hh>
#include <roboptim/core/sum-of-c1-squares.hh>

#include <sophus/se3.hpp>

namespace robcalib
{
using Sophus::SE3d;

/**
 * @brief Cost function without analytical jacobian.
 * Wrap it with finite differences on it to get a numerical jacobian suitable
 * for optimisation
 */
struct CostKinematicsChain : public roboptim::DifferentiableFunction
{
  // Index of joint to joint pose to estimate
  std::vector<size_t> estimate_x_;
  // Poses already known in the kinematic chain
  std::vector<SE3d::Tangent> known_x_;
  std::shared_ptr<CalibData> data_;

  CostKinematicsChain(std::shared_ptr<CalibData> data)
      : roboptim::DifferentiableFunction(6 * (data->joints_size() - 1),
                                         16 * data->rel_poses_size() * (data->joints_size() - 1),
                                         "CostKinematicsChain"),
        data_(data)
  {
    for (size_t i = 0; i < data->joints_size() - 1; ++i)
    {
      estimate_x_.push_back(i);
    }
    LOG(info) << "CostKinematicsChain with " << data->rel_poses_size() << " poses." << std::endl;
    LOG(info) << "Input: " << this->inputSize() << ", Output: " << this->outputSize() << std::endl;
    LOG(info) << "Calibrating " << estimate_x_.size() << " of " << data->joints_size() - 1 << " poses" << std::endl;
  }

  CostKinematicsChain(const std::vector<size_t>& estimate_x, const std::vector<Sophus::SE3d::Tangent>& known_x, std::shared_ptr<CalibData> data)
      : roboptim::DifferentiableFunction(6 * estimate_x.size(), 16 * data->rel_poses_size(), "CostKinematicsChain"),
        estimate_x_(estimate_x),
        known_x_(known_x),
        data_(data)
  {
    LOG(info) << "CostKinematicsChain with " << data->rel_poses_size() << " poses." << std::endl;
    LOG(info) << "Input: " << this->inputSize() << ", Output: " << this->outputSize() << std::endl;
    LOG(info) << "Calibrating " << estimate_x_.size() << " of " << data->joints_size() - 1 << " poses" << std::endl;
  }

  /**
   * @brief Computes the cost vector
   * WARNING: Data is organised from bottom of kinematic chain to the top
   *
   * @param result
   * Result is the residual of the cost function at x.
   *
   * @param x
   * The point at which the cost function is computed
   *
   * XXX loads of unneeded lambda functions. To be cleaned up
   */
  void impl_compute(result_ref result, const_argument_ref x) const
  {
    LOG(debug) << "compute" << std::endl;
    using namespace Eigen;

    const auto k = data_->rel_poses_size();
    const auto Neq = data_->joints_size() - 1;
    // LOG(debug) << "Neq = " << Neq << std::endl;
    // LOG(debug) << "k = " << k << std::endl;
    result.setZero();

    auto Sij = [this](size_t i, size_t j) -> Sophus::SE3d {
      return rpyToSE3(data_->S[j][i]);
    };

    auto combine_X = [&x, this]() -> std::vector<Sophus::SE3d> {
      auto N = x.size()/6 + known_x_.size();
      std::vector<Sophus::SE3d>  combined(N);
      int id_known = 0;
      for (size_t i = 0; i < N; ++i) {
        const auto& it = std::find(estimate_x_.begin(), estimate_x_.end(), i);
        if (it != estimate_x_.end())
        {
          // Get id remapped to reduced range (estimaing only some parameters)
          auto idm = std::distance(estimate_x_.begin(), it);
          LOG(debug) << "Use Xj(i): i=" << i << ", idm=" << idm << std::endl;

          // estimate_x_ contains
          const SE3d::Tangent t(x.block<6, 1>(idm * 6, 0));
          combined[i] = SE3d::exp(t);
        }
        else
        {
          combined[i] = SE3d::exp(known_x_[id_known]);
          id_known++;
        }
      }
      return combined;
    };
    std::vector<Sophus::SE3d> X = combine_X();

    for (size_t i = 0; i < k; i++)
    {
      LOG(debug) << "i: " << i << std::endl;
      const Sophus::SE3d& Ai = data_->rel_poses_[i];
      const auto& relative_pair = data_->rel_to_[i];

      Sophus::SE3d Xeq_l(Eigen::Matrix4d::Identity());
      Sophus::SE3d Xeq_r(Eigen::Matrix4d::Identity());

      for (size_t eq = 0; eq < Neq; eq++)
      {
        LOG(debug) << "eq: " << eq;
        Xeq_l = Xeq_l * Sij(relative_pair.second, eq) * X[eq];
        LOG(debug) << "Xeq_l: \n" << Xeq_l.matrix() << std::endl;
        // Multiply by initial joint rotation to estimate robot at Identity and
        // not at current joint state
        Xeq_r = Xeq_r * Sij(relative_pair.first, eq) * X[eq];
        LOG(debug) << "Xeq_r: \n" << Xeq_r.matrix() << std::endl;
      }
      LOG(debug) << "computing residual for i=" << i << std::endl;
      LOG(debug) << "Camera i: " << Ai.matrix() << std::endl;
      const Eigen::Matrix4d& e = (Ai * Xeq_l.inverse()).matrix() - Xeq_r.inverse().matrix();
      // XXX use Eigen Map
      for (int r = 0; r < e.size(); r++)
      {
        // LOG(debug) << i*16 + r;
        result[i * 16 + r] = e(r);
      }
      LOG(debug) << "Done for i=" << i << std::endl;
    }
  }

  void impl_gradient(gradient_ref, const_argument_ref, size_type) const override
  {
    throw(std::runtime_error{"Gradient not implemented, use finite differences"});
  }
};

} /* robcalib */
