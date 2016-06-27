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
#include "robcalib/input_module.h"

#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_eigen/tf2_eigen.h>
#include <sensor_msgs/JointState.h>

namespace robcalib
{

class ROSInputModule : public InputModule
{
 private:
  std::pair<std::string, std::string> head_link_;

  std::shared_ptr<ros::NodeHandle> nh;
  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tf_l;
  tf2_ros::TransformBroadcaster tf_caster;
  std::thread spin_th;
  void spinner();

  ros::Subscriber sub_joint;
  std::mutex joints_mutex;
  std::map<std::string, Eigen::Vector3d> latest_joints;
  void processJoints(const sensor_msgs::JointState::ConstPtr& _js);

  bool valid_joints = false;

  std::shared_ptr<HeadEyeData> head_eye_data_;
  using head_eye_data_callback_t = std::function<void(const std::shared_ptr<HeadEyeData>)>;
  head_eye_data_callback_t head_eye_data_callback_;

 public:
  ROSInputModule(const std::string& joint_topic);
  ~ROSInputModule();

  void set_head_eye_data_callback(head_eye_data_callback_t callback) { head_eye_data_callback_ = callback; }

  void setCameraFrame(std::pair<std::string, std::string>& head_link)
  {
    head_link_ = head_link;
    LOG(info) << "Using camera frame: " << head_link_.first << " -> " << head_link_.second;
  }

  void setJoints(const std::vector<Joint>& joints)
  {
    data_ = std::make_shared<CalibData>(joints);
    LOG(info) << "Using joints:";
    for(const Joint&j : joints)
    {
      LOG(info) << "Name: " << j.joint << ", axis: " << j.axis.transpose();
    }
  }

};

} /* robcalib */
