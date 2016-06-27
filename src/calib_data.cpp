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

#include <robcalib/calib_data.h>
#include <algorithm>
#include <iostream>
#include <iterator>

#include <fstream>

using namespace std;

namespace robcalib
{
void removeCharsFromString(string& str, const string& charsToRemove)
{
  for (unsigned int i = 0; i < charsToRemove.length(); ++i)
  {
    str.erase(remove(str.begin(), str.end(), charsToRemove[i]), str.end());
  }
}

ostream& operator<<(ostream& os, const Sophus::SE3d& dt)
{
  for (int i = 0; i < dt.matrix().rows(); i++)
  {
    for (int j = 0; j < dt.matrix().cols(); j++)
    {
      os << dt.matrix()(i, j) << " ";
    }
  }
  return os;
}

ostream& operator<<(ostream& os, const Eigen::Vector3d& dt)
{
  os << dt(0) << " " << dt(1) << " " << dt(2);
  return os;
}

istream& operator>>(istream& iss, Eigen::Vector3d& res)
{
  iss >> res(0) >> res(1) >> res(2);
  return iss;
}

istream& operator>>(istream& iss, Eigen::Matrix4d& res)
{
  for (int i = 0; i < res.rows(); i++)
  {
    for (int j = 0; j < res.cols(); j++)
    {
      iss >> res(i, j);
    }
  }
  return iss;
}

std::ostream& operator<<(std::ostream& os, const CalibData& dt)
{
  os << "Calibration Data" << std::endl;
  os << "Number of joints: " << dt.joints_.size() << std::endl;
  os << "Relative poses: " << dt.rel_poses_.size() << std::endl;
  os << "Joint angles: " << dt.S[0].size() << std::endl;

  os << "A:" << std::endl;
  for (const auto& m : dt.rel_poses_)
  {
    os << m.matrix() << "\n\n";
  }

  os << std::endl << std::endl;

  for (unsigned int i = 0; i < dt.joints_.size(); ++i)
  {
    const auto& joint = dt.joints_[i];

    os << "S for joint " << joint.joint << std::endl;
    for (const auto& d : dt.S[i])
    {
      os << d.transpose().matrix() << std::endl;
    }
  }
  return os;
}

void CalibData::readFromDisk(const std::string& folder)
{
  LOG(debug) << "Reading from " << folder << std::endl;
  poses_.clear();
  rel_poses_.clear();
  S.clear();

  std::string name = joints_.back().joint;
  removeCharsFromString(name, "/");
  std::string pose_path = folder + "/poses_" + name + ".csv";
  LOG(debug) << "Reading head pose from: " << pose_path << std::endl;
  ifstream f_pose_head(pose_path);
  Eigen::Matrix4d p;
  while (f_pose_head >> p)
  {
    Eigen::Matrix3d rot = Sophus::makeRotationMatrix(p.topLeftCorner<3,3>());
    Eigen::Vector3d t = p.topRightCorner<3,1>();
    poses_.emplace_back(Sophus::SE3d{rot, t});
  }

  for (const Sophus::SE3d& pose : poses_)
  {
    LOG(debug) << pose.matrix() << std::endl << std::endl;
  }

  S.resize(this->joints_size());
  for (size_t i = 0; i < this->joints_size(); ++i)
  {
    const auto& joint = this->joints_[i];

    name = joint.joint;
    removeCharsFromString(name, "/");
    std::string fname = folder + "/angles_" + name + ".csv";
    LOG(debug) << "Reading joint_angles for joint " << joint.joint << " from file: " << fname << std::endl;
    std::ifstream fangles(fname);
    Eigen::Vector3d Sj;
    while (fangles >> Sj)
    {
      S[i].emplace_back(Sj);
    }
  }
}

void CalibData::saveToDisk(const std::string& folder)
{
  std::string name = joints_.back().joint;
  removeCharsFromString(name, "/");
  std::string pose_path = folder + "/poses_" + name + ".csv";
  std::cout << "Saving poses for head joint to " << pose_path << std::endl;
  std::ofstream fposes(pose_path);
  for (const Sophus::SE3d& pose : this->poses_)
  {
    fposes << pose << std::endl;
  }

  for (size_t i = 0; i < this->joints_size(); ++i)
  {
    const auto& joint = this->joints_[i];

    name = joint.joint;
    removeCharsFromString(name, "/");
    std::cout << "Saving joint_angles for joint " << joint.joint << std::endl;
    std::ofstream fangles(folder + "/angles_" + name + ".csv");
    for (const Eigen::Vector3d& angle : this->S[i])
    {
      fangles << angle << std::endl;
    }
  }
}
} /* robcalib */
