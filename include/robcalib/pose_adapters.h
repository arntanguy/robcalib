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

#include <sophus/se3.hpp>
#include <iostream>

namespace robcalib
{
/**
 * @brief Computes the relative pose
 *
 * This is a very simple way of generating pairs of poses.
 * It would be worth exploring more fine-grained options, such as selecting a
 * miminum threshold between consecutive poses, running EKF, and such.
 */
struct RelativePoseAdapter
{
  std::pair<std::vector<std::pair<int, int>>, std::vector<Sophus::SE3d>> operator()(const std::vector<Sophus::SE3d>& poses)
  {
    // XXX reserve vector
    std::vector<Sophus::SE3d> relative_poses;
    std::vector<std::pair<int, int>> relative_to;
    for (size_t j = 0; j < poses.size(); j+=step)
    {
      if(j<poses.size()) {
        relative_poses.push_back(poses[0].inverse() * poses[j]);
        relative_to.push_back(std::make_pair( 0, j ));
      }
      // relative_poses.push_back(poses[0].inverse() * poses[i]);
      // relative_to.push_back(0);
    }
    return {relative_to, relative_poses};
  }

  /* Compute a relative pose every N steps */
  unsigned step = 1;
};

} /* robcalib */
