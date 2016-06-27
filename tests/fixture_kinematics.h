#pragma once

#include <sophus/se3.hpp>
#pragma once

#include <vector>
#include <robcalib/calib_data.h>
#include <robcalib/logging.h>

namespace robcalib
{

struct KinematicsVREPHRP4Fixture
{
  std::shared_ptr<CalibData> data;
  std::vector<size_t> estimate_x = {1,3};
  std::vector<Sophus::SE3d::Tangent> known_x;
  Eigen::VectorXd expected_x;
  double precision = 10e-3;

  KinematicsVREPHRP4Fixture()
  {
    auto joints = std::vector<Joint>{
                                     {"CHEST_P", {0., 1., 0.}},
                                     {"CHEST_Y", {0., 0., 1.}},
                                     {"NECK_Y", {0., 0., 1.}},
                                     {"NECK_P", {0., 1., 0.}},
                                     {"xtion_link_joint", {0., 0., 0.}}};

    known_x.resize(2);
    known_x[0].setZero();
    known_x[1].setZero();

    data = std::make_shared<CalibData>(joints);
    data->adapter.step = 10;
    data->readFromDisk("./data/hrp4_vrep");
    data->update();
    expected_x.resize(12);
    expected_x << -0.04,0.0,0.434, 0, 0, 0, 0.09, 0.02, 0.004, 0., 0., 0.;
  }
};

} /* robcalib */
