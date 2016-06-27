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

#include "robcalib/ros_input_module.h"
#include "robcalib/logging.h"

namespace robcalib
{
ROSInputModule::ROSInputModule(const std::string& joint_topic) : InputModule(),  nh(new ros::NodeHandle), tf_l(tfBuffer), tf_caster()
{
  sub_joint = nh->subscribe(joint_topic, 1, &ROSInputModule::processJoints, this);

  // HRP4
  // links_["control/base_link"] = {"control/xtion_link", "control/NECK_P_LINK", "control/NECK_Y_LINK",
  // "control/CHEST_P_LINK", "control/torso", "control/body"};
  // head_link_ = std::make_pair("control/base_link", "control/xtion_link");
  // auto joints = std::vector<Joint>{// {"control/body", "waist", {0., 0., 0.}},
  //                                  {"control/CHEST_P_LINK", "CHEST_P", {0., 1., 0.}},
  //                                  {"control/torso", "CHEST_Y", {0., 0., 1.}},
  //                                  {"control/NECK_Y_LINK", "NECK_Y", {0., 0., 1.}},
  //                                  {"control/NECK_P_LINK", "NECK_P", {0., 1., 0.}},
  //                                  {"control/xtion_link", "xtion_link_joint", {0., 0., 0.}}};


  // HRP2
  // links_["px2m_map"] = {"camera_rgb_frame"};
  // data_.joints_ = {
  //   // {"control/body", "waist", {0., 0., 0.}},
  //   // {"CHEST_LINK0", "CHEST_JOINT0", {0., 0., 1.}},
  //   // {"CHEST_LINK1", "CHEST_JOINT1", {0., 1., 0.}},
  //   {"HEAD_LINK0", "HEAD_JOINT0", {0., 0., 1.}},
  //   {"HEAD_LINK1", "HEAD_JOINT1", {0., 1., 0.}},
  //   {"camera_rgb_frame", "xtion_link_joint", {0., 0., 0.}}};

  // NAO
  head_link_ = std::make_pair("map", "camera_link");
  // head_link_ = std::make_pair("robot_map", "real/xtion_link");
  auto joints = std::vector<Joint>{{"HeadYaw", {0., 0., 1.}},
                                   {"HeadPitch", {0., 1., 0.}},
                                   {"xtion_joint", {0., 0., 0.}}};

  head_eye_data_ = std::make_shared<HeadEyeData>();

  data_ = std::make_shared<CalibData>(joints);
  spin_th = std::thread(std::bind(&ROSInputModule::spinner, this));
}

ROSInputModule::~ROSInputModule() { spin_th.join(); }
void ROSInputModule::spinner()
{
  LOG(info) << "ROS Input spinner started" << std::endl;
  ros::Rate rt(rate_);
  while (ros::ok())
  {
    ros::spinOnce();
    rt.sleep();

    if (latest_joints.empty())
    {
      LOG(warning) << "Couldn't fetch joint state: " << latest_joints.size() << std::endl;
      continue;
    }
  }
}

void ROSInputModule::processJoints(const sensor_msgs::JointState::ConstPtr& js)
{
  LOG(debug) << "process joints" << std::endl;
  std::map<std::string, double> jv;
  for (unsigned int i = 0; i < js->name.size(); i++)
  {
    const auto& name = js->name[i];
    const auto value = js->position[i];
    jv[name] = value;
    // LOG(info) << "Joint: " << name << " = " << value;
  }

  std::lock_guard<std::mutex> l(joints_mutex);
  latest_joints.clear();
  int i = 0;
  valid_joints = true;
  for (const auto& j : data_->joints_)
  {
    const auto& j_name = j.joint;
    // LOG(info) << "Looking for joint: " << j_name;
    if (jv.count(j_name) > 0)
    {
      // LOG(info) << "Joint found with value: " << jv[j_name] << ", and axis: " << j.axis;
      latest_joints[j_name] = jv[j_name] * j.axis;
    }
    else
    {
      // LOG(error) << "Joint: " << j_name << " not found";
      // latest_joints[j_name] = Eigen::Vector3d::Zero();
    }
    i++;
  }


  // Get camera from SLAM
  bool pose_available = true;
  Sophus::SE3d pose, pose_head;
  geometry_msgs::TransformStamped transform ;
  try
  {
    LOG(info) << "Getting tf for eye pose: " << head_link_.first << ", " << head_link_.second;
    transform = tfBuffer.lookupTransform(head_link_.first, head_link_.second, ros::Time(0));
    LOG(info) << "Got TF";
    Eigen::Affine3d X = tf2::transformToEigen(transform);
    pose = Sophus::SE3d(X.matrix());

    transform =
        tfBuffer.lookupTransform("robot_map", "control/NECK_P_LINK", ros::Time(0));
    Eigen::Affine3d X_head = tf2::transformToEigen(transform);
    pose_head = Sophus::SE3d(X_head.matrix());
  }
  catch (tf2::TransformException &ex) {
         ROS_WARN("%s",ex.what());
         ros::Duration(1.0).sleep();
       }
  catch (...)
  {
    pose_available = false;
    LOG(warning) << "WARNING: Missing transformation from " << head_link_.first << " to " << head_link_.second
                 << std::endl;
    //break;
  }


  {
    // LOG(info) << "Pose: " << pose_available << ", latest joints size: " << latest_joints.size() << ", " << data_->joints_size();
    if (pose_available && latest_joints.size() == data_->joints_size()-1)
    {
      data_->insert_pose(pose);
      data_->insert_joint_angles(latest_joints);
      data_->update();
      if(data_callback_)
      {
        data_callback_(data_);
      }

      head_eye_data_->eye.push_back(pose);
      head_eye_data_->link.push_back(pose_head);
      if(head_eye_data_callback_)
      {
        head_eye_data_callback_(head_eye_data_);
      }
    }
    else
    {
      LOG(warning) << "Invalid insert" << std::endl;
    }
  }
}
} /* robcalib */
