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

#include <json/json.h>
#include <robcalib/calib_data.h>

namespace robcalib
{
  class CalibJson
  {
   public:
    CalibJson(const std::string& path);

    std::string robot;
    std::string joint_topic;
    std::vector<std::string> joints;
    std::vector<Eigen::Vector3d> axes;
    std::pair<std::string, std::string> camera_frame;
    size_t n_pairs;
    unsigned int relative_step = 1;
    std::vector<size_t> estimate_x;
    enum class Method {
      HEAD_EYE,
      KINEMATICS
    };
    Method method;

    std::vector<Joint> getJoints();
  };
} /* robcalib */
