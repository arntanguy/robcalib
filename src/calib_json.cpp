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

#include "robcalib/calib_json.h"
#include <fstream>
#include "robcalib/logging.h"

std::istream& operator>>(std::istream& in, Eigen::Vector3d& v)
{
  in >> v(0) >> v(1) >> v(2);
  return in;
}

namespace robcalib
{
CalibJson::CalibJson(const std::string& path)
{
  method = Method::KINEMATICS;

  Json::Value root;
  std::ifstream ifs(path);
  try
  {
    ifs >> root;
  }
  catch (const std::exception& exc)
  {
    LOG(error) << "Failed to read configuration file: " << path;
    LOG(error) << exc.what();
    return;
  }

  if(root.isMember("method"))
  {
    const auto& m = root["method"].asString();
    if(m == "head_eye")
    {
      method = Method::HEAD_EYE;
    }
    else
    {
      method = Method::KINEMATICS;
    }
  }
  if (root.isMember("robot"))
  {
    robot = root["robot"].asString();
  }

  if (root.isMember("joints"))
  {
    Json::Value jv = root["joints"];
    if(jv.isMember("topic"))
    {
      joint_topic = jv["topic"].asString();
    }
    if (jv.isMember("joints"))
    {
      Json::Value joints_arr = jv["joints"];
      for (size_t i = 0; i < joints_arr.size(); i++)
      {
        joints.emplace_back(joints_arr[static_cast<int>(i)].asString());
      }
    }
    if (jv.isMember("axes"))
    {
      Json::Value axes_arr = jv["axes"];
      for (size_t i = 0; i < axes_arr.size(); i++)
      {
        Json::Value val = axes_arr[static_cast<int>(i)];
        Eigen::Vector3d axis;
        axis[0] = val[0].asDouble();
        axis[1] = val[1].asDouble();
        axis[2] = val[2].asDouble();
        axes.push_back(axis);
      }
    }
  }

  if (root.isMember("camera_frame"))
  {
    camera_frame = std::make_pair<std::string, std::string>(root["camera_frame"][0].asString(),
                                                            root["camera_frame"][1].asString());
  }

  if(root.isMember("n_pairs"))
  {
    n_pairs = static_cast<size_t>(root["n_pairs"].asInt());
  }
  if(root.isMember("relative_step"))
  {
    relative_step = static_cast<size_t>(root["relative_step"].asInt());
  }

  if(root.isMember("estimate_x"))
  {
    Json::Value jv = root["estimate_x"];
    for(size_t i=0; i<jv.size(); i++)
    {
      estimate_x.emplace_back(static_cast<size_t>(jv[static_cast<int>(i)].asInt()));
    }
  }
}

std::vector<Joint> CalibJson::getJoints()
{
  assert(joints.size() == axes.size());

  std::vector<Joint> joint_v(joints.size());
  for (unsigned int i = 0; i < joints.size(); i++)
  {
    joint_v[i] = {joints[i], axes[i]};
  }
  return joint_v;
}

} /* robcalib */
