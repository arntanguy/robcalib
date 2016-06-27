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

#include "robcalib/maths.h"

namespace robcalib
{
Sophus::SE3d rpyToSE3(const Eigen::Vector3d& rpy)
{
  Eigen::Matrix3d m;
  m = Eigen::AngleAxisd(rpy(2), Eigen::Vector3d::UnitZ()) * Eigen::AngleAxisd(rpy(1), Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxisd(rpy(0), Eigen::Vector3d::UnitX());
  Eigen::Matrix4d s = Eigen::Matrix4d::Identity();
  s.block<3, 3>(0, 0) = m;
  return Sophus::SE3d(s);
}

void se3toXYZRPY(const Sophus::SE3d::Tangent& x, Eigen::Vector3d& xyz, Eigen::Vector3d& rpy)
{
  Sophus::SE3d X = Sophus::SE3d::exp(x);
  xyz = X.translation();
  rpy = X.rotationMatrix().matrix().eulerAngles(0, 1, 2);
}

} /* robcalib */
