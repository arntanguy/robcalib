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

#include <robcalib/cost_kinematics_chain.h>
#include <robcalib/cost_link_eye.h>
#include <robcalib/maths.h>
#include <robcalib/logging.h>
#include <iostream>
#include <memory>
#include <roboptim/core/io.hh>
#include <roboptim/core/optimization-logger.hh>
#include <roboptim/core/solver-factory.hh>
#include <roboptim/core/solver.hh>
#include <sophus/se3.hpp>

#include <boost/make_shared.hpp>

namespace robcalib
{
// Specify the solver that will be used.
using solver_t = roboptim::Solver<roboptim::EigenMatrixDense>;

/**
 * @brief Uses CRTP to set up an interface for various calibration procedures
 *
 * @tparam T
 */
template <class T>
struct BaseCalib
{
  solver_t::parameters_t parameters;
  // Create problem.
  std::unique_ptr<solver_t::problem_t> pb;

  BaseCalib()
  {
    LOG(info) << "BaseCalib::BaseCalib()" << std::endl;

    // Default solver parameters
    // TODO parse config from file (see roboptim-core-config)
    parameters["eigen.factor"].value = 100.;
    parameters["eigen.maxfev"].value = 30;
    parameters["eigen.ftol"].value = std::sqrt(Eigen::NumTraits<double>::epsilon());
    parameters["eigen.xtol"].value = std::sqrt(Eigen::NumTraits<double>::epsilon());
    parameters["eigen.gtol"].value = 0.001;
    parameters["eigen.epsfcn"].value = 0.;
  }

  void createProblem(const std::shared_ptr<CalibData> data)
  {
    LOG(info) << "BaseCalib::createProblem" << std::endl;

    // Problem is defined by the data and the cost functions
    static_cast<T*>(this)->initSolverImpl(data);
  }

  /**
   * @brief Runs the solver (of the child class T)
   */
  solver_t::result_t run()
  {
    LOG(info) << "BaseCalib::run" << std::endl;
    if (pb != nullptr)
    {
      // Run the actual function in the derived class
      return static_cast<T*>(this)->runImpl();
    }
    else
    {
      throw std::runtime_error("Problem hasn't been defined!");
    }
  }
};

struct HeadEyeCalib : public BaseCalib<HeadEyeCalib>
{
  using cost_t = CostLinkEyeSE3;

  Sophus::SE3d::Tangent current_x;

 public:
  HeadEyeCalib()
  {
    LOG(debug) << "Contructor of HeadEyeCalib" << std::endl;
    current_x.setZero();
  }

  void initSolverImpl(const std::shared_ptr<HeadEyeData> data)
  {
    LOG(info) << "HeadEyeCalib::initSolverImpl" << std::endl;
    // Minimum 2 relative poses
    // TODO add a pose validator
    if (data->eye.size() > 1)
    {
      const auto& eye = data->eye;
      const auto& link = data->link;
      // Cost function
      boost::shared_ptr<cost_t> fd(new cost_t(eye, link));
      // Compute sum of C1 squares on function
      boost::shared_ptr<roboptim::SumOfC1Squares> fSum(new roboptim::SumOfC1Squares(fd, "some function"));

      pb.reset(new solver_t::problem_t(fSum));

      // Set bounds for all optimization parameters.
      // 1. < x_i < 5. (x_i in [1.;5.])
      for (roboptim::Function::size_type i = 0; i < pb->function().inputSize(); ++i)
        pb->argumentBounds()[i] = roboptim::Function::makeInfiniteInterval();

      // Set the starting point.
      roboptim::Function::vector_t start(pb->function().inputSize());
      start = current_x;
      LOG(debug) << "Starting point: " << start.matrix().transpose() << std::endl;
      pb->startingPoint() = start;
    }
    else
    {
      throw std::runtime_error("Invalid data");
    }
  }

  solver_t::result_t runImpl()
  {
    LOG(info) << "HeadEyeCalib::runImpl()" << std::endl;

    roboptim::SolverFactory<solver_t> factory("eigen-levenberg-marquardt", *pb);
    solver_t& solver = factory();

    // Add optimization logger.
    roboptim::OptimizationLogger<solver_t> logger(solver, std::string("/tmp/head-eye-logger"));

    // ==== CONFIGURE SOLVER ====
    auto& param = solver.parameters();
    // Set default parameters
    param = parameters;

    // Compute the minimum and retrieve the result.
    solver_t::result_t res = solver.minimum();

    logger.append("Test");
    logger << solver;

    /**
     * If there is a solution, update the initial guess current_x
     * for the next time the calibration problem is called
     **/
    switch (res.which())
    {
      case solver_t::SOLVER_VALUE:
      {
        roboptim::Result& result = boost::get<roboptim::Result>(res);

        // Display the result.
        LOG(info) << "A solution has been found: " << std::endl << result << std::endl;
        current_x = result.x;
        break;
      }

      case solver_t::SOLVER_VALUE_WARNINGS:
      {
        // Get the result.
        roboptim::ResultWithWarnings& result = boost::get<roboptim::ResultWithWarnings>(res);

        // Display the result.
        LOG(info) << "A solution has been found with warnings: " << std::endl << result << std::endl;
        Eigen::Vector3d xyz, rpy;
        se3toXYZRPY(result.x, xyz, rpy);
        LOG(info) << "xyz: " << xyz.transpose() << ", rpy " << rpy.transpose();
        current_x = result.x;
        break;
      }

      case solver_t::SOLVER_NO_SOLUTION:
      case solver_t::SOLVER_ERROR:
      {
        LOG(error) << "A solution should have been found. Failing..." << std::endl
                   << boost::get<roboptim::SolverError>(res).what() << std::endl;

        break;
      }
    }
    static int count = 0;
    std::cout << count++ << std::endl;
    return res;
  }
};

struct KinematicsEyeCalib : public BaseCalib<KinematicsEyeCalib>
{
  Eigen::VectorXd current_x;
  using cost_t = CostKinematicsChain;

  // XXX this makes finite differences much faster than the default
  // Make it configurable
  // using FDPolicy = roboptim::finiteDifferenceGradientPolicies::FivePointsRule<roboptim::EigenMatrixDense>;
  using FDPolicy = roboptim::finiteDifferenceGradientPolicies::Simple<roboptim::EigenMatrixDense>;
  using FiniteDifferences = roboptim::GenericFiniteDifferenceGradient<roboptim::EigenMatrixDense, FDPolicy>;
  // Default is roboptim::finiteDifferenceEpsilon = 10e-8
  // const double FDepsilon = 10e-3;
  const double FDepsilon = 10e-4;

  boost::shared_ptr<cost_t> f;
  std::shared_ptr<CalibData> data_;
  std::vector<size_t> estimate_x_;
  std::vector<Sophus::SE3d::Tangent> known_x_;

  KinematicsEyeCalib()
  {
    LOG(debug) << "KinematicsEyeCalib::KinematicsEyeCalib" << std::endl;
    current_x.setZero();
  }

  KinematicsEyeCalib(const std::vector<size_t>& estimate_x, const std::vector<Sophus::SE3d::Tangent> known_x) : estimate_x_(estimate_x), known_x_(known_x)
  {
    LOG(debug) << "KinematicsEyeCalib::KinematicsEyeCalib" << std::endl;
    current_x.setZero();
  }

  void initSolverImpl(const std::shared_ptr<CalibData> data)
  {
    LOG(info) << "KinematicsEyeCalib::initSolverImpl" << std::endl;
    data_ = data;
    assert(data->rel_poses_size() > 1);

    // If no information is supplied on what to estimate, estimate everything
    if (estimate_x_.empty())
    {
      for (size_t i = 0; i < data->joints_size() - 1; ++i)
      {
        estimate_x_.push_back(i);
      }
    }

    // Cost function
    f = boost::make_shared<cost_t>(estimate_x_, known_x_, data);
    // Instantiate the FD wrapper over Function
    // XXX why passed by reference?
    auto fd = boost::make_shared<FiniteDifferences>(*f, FDepsilon);
    // Compute sum of C1 squares on function
    auto fSum = boost::make_shared<roboptim::SumOfC1Squares>(fd, "Least Square KinematicsEyeCalib");

    LOG(debug) << "Cost function defined";

    LOG(debug) << "Resetting problem";
    pb.reset(new solver_t::problem_t(fSum));
    LOG(debug) << "Problem reset";

    LOG(debug) << "Setting bounds";
    // Set bounds for all optimization parameters.
    for (roboptim::Function::size_type i = 0; i < pb->function().inputSize(); ++i)
    {
      pb->argumentBounds()[i] = roboptim::Function::makeInfiniteInterval();
      // if (i % 6 < 3)
      // {
      //   // Translation bounds
      //   pb->argumentBounds()[i] = roboptim::Function::makeInterval(-1, 1);
      // }
      // else
      // {
      //   // Rotation bounds
      //   pb->argumentBounds()[i] = roboptim::Function::makeInterval(-M_PI, M_PI);
      // }
    }
    LOG(debug) << "Bounds set";

    LOG(debug) << "Initializing starting point";
    // If not initializerd
    if (current_x.size() == 0)
    {
      current_x = Eigen::VectorXd(pb->function().inputSize());
      current_x.setZero();
    }
    pb->startingPoint() = current_x;
    LOG(debug) << "Starting point initialized";
  }

  solver_t::result_t runImpl()
  {
    LOG(info) << "KinematicsEyeCalib::runImpl()" << std::endl;
    // Initialize solver.
    LOG(debug) << "Initializing solver";
    roboptim::SolverFactory<solver_t> factory("eigen-levenberg-marquardt", *pb);
    solver_t& solver = factory();
    LOG(debug) << "Solver initialized";
    // ==== CONFIGURE SOLVER ====
    LOG(debug) << "Configuring solver";
    // XXX configuration of parameters crashes, investigate
    LOG(debug) << "CONFIGURATION DISABLED FOR NOW (crashes)";
    // auto& param = solver.parameters();
    // // Set default parameters
    // param = parameters;
    LOG(debug) << "solver configured";

    // Add optimization logger.
    roboptim::OptimizationLogger<solver_t> logger(solver, std::string("/tmp/kinematics-logger"));

    // Compute the minimum and retrieve the result.
    LOG(debug) << "Minimizing...";
    solver_t::result_t res = solver.minimum();
    LOG(debug) << "Minimization finished";

    /**
     * If there is a solution, update the initial guess current_x
     * for the next time the calibration problem is called
     **/
    LOG(debug) << "Printing results";
    switch (res.which())
    {
      case solver_t::SOLVER_VALUE:
      {
        roboptim::Result& result = boost::get<roboptim::Result>(res);

        // Display the result.
        LOG(info) << "A solution has been found: " << std::endl << result << std::endl;
        current_x = result.x;
        break;
      }

      case solver_t::SOLVER_VALUE_WARNINGS:
      {
        // Get the result.
        roboptim::ResultWithWarnings& result = boost::get<roboptim::ResultWithWarnings>(res);

        // Display the result.
        LOG(info) << "A solution has been found with warnings: " << std::endl << result << std::endl;
        for (size_t i = 0; i < static_cast<size_t>(result.x.size()); i += 6)
        {
          const auto id = i / 6;
          const Sophus::SE3d::Tangent& x = result.x.block<6, 1>(i, 0);
          LOG(info) << "X" << id << " (" << data_->joints_[estimate_x_[id]].joint
                    << " -> " << data_->joints_[estimate_x_[id]+1].joint << "): " << x.transpose().matrix() << std::endl;
          Eigen::Vector3d xyz, rpy;
          se3toXYZRPY(x, xyz, rpy);
          LOG(info) << "xyz: " << xyz.transpose() << ", rpy " << rpy.transpose();

        }
        current_x = result.x;
        break;
      }

      case solver_t::SOLVER_NO_SOLUTION:
      case solver_t::SOLVER_ERROR:
      {
        LOG(error) << "A solution should have been found. Failing..." << std::endl
                   << boost::get<roboptim::SolverError>(res).what() << std::endl;

        break;
      }
    }
    return res;
  }
};

} /* robcalib */
