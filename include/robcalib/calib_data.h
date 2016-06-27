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

#include <robcalib/logging.h>
#include <robcalib/pose_adapters.h>
#include <sophus/se3.hpp>

namespace robcalib
{
struct Joint
{
  std::string joint;
  Eigen::Vector3d axis;

  Joint() {}

  Joint(const std::string& joint, const Eigen::Vector3d& axis)
      : joint(joint), axis(axis)
  {
  }
};

struct CalibData
{
  // XXX todo template with that, or provide it at runtime
  RelativePoseAdapter adapter;
  using poses_t = std::vector<Sophus::SE3d>;


  // Kinematic chain properties
  // Joints are ordered from base to top of Kinematic tree
  std::vector<Joint> joints_;

  // Absolute poses for the camera (in odometry reference frame)
  poses_t poses_;
  // Absolute joint angles for each joint (w.r.t identity joint)
  std::vector<std::vector<Eigen::Vector3d>> S;

  // Index of relative pose, from i to j
  std::vector<std::pair<int, int>> rel_to_;

  // Relative pose
  // NOTE call update() to keep it up to date!
  poses_t rel_poses_;

  CalibData(std::vector<Joint> joints) : joints_(joints) { S.resize(joints.size()); }
  void insert_pose(const Sophus::SE3d& poses) {
    LOG(info) << "inserting pose: " << poses.translation().transpose();
    poses_.push_back(poses); }
  void insert_joint_angles(const std::map<std::string, Eigen::Vector3d>& angles)
  {
    for (size_t i = 0; i < joints_.size()-1; ++i)
    {
      const auto& joint = joints_[i];
      LOG(info) << "inserting joint: " << joint.joint << " = " << angles.at(joint.joint).transpose();
      S[i].push_back(angles.at(joint.joint));
    }
    // xtion_link fixed joint
    S[joints_.size()-1].push_back(Eigen::Vector3d{0.,0.,0.});
  }

  size_t size() const { return poses_.size(); }
  Sophus::SE3d pose(const size_t i) { return rel_poses_[i]; }
  void update()
  {
    rel_poses_.clear();
    rel_to_.clear();
    auto a = adapter(poses_);
    rel_to_ = a.first;
    rel_poses_ = a.second;
  }

  size_t joints_size() const { return joints_.size(); }
  size_t rel_poses_size() const { return rel_poses_.size(); }
  bool is_valid() const { return true; }
  friend std::ostream& operator<<(std::ostream& os, const CalibData& dt);


  void readFromDisk(const std::string& folder);
  void saveToDisk(const std::string& folder);

  /**
   * @brief XXX Implemented with inefficient copies.
   * Use boost::circular_buffer instead
   *
   * @param s window size (number of absolute poses to store)
   */
  void rolling_window(const size_t s)
  {
    if(size() <= s) return;

    auto start = size()-s;
    poses_t cposes;
    // Absolute joint angles for each joint (w.r.t identity joint)
    std::vector<std::vector<Eigen::Vector3d>> cS;

    std::copy(poses_.begin()+start, poses_.end(), std::back_inserter(cposes));
    for(const auto & s: S)
    {
      std::vector<Eigen::Vector3d> ss;
      std::copy(s.begin()+start, s.end(), std::back_inserter(ss));
      cS.push_back(ss);
    }

    poses_ = cposes;
    S = cS;
    update();
  }
};

std::ostream& operator<<(std::ostream& os, const CalibData& dt);


struct HeadEyeData
{
  std::vector<Sophus::SE3d> eye;
  std::vector<Sophus::SE3d> link;
};

} /* robcalib */
