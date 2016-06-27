#pragma once

#include <sophus/se3.hpp>
#include <vector>

namespace robcalib
{
struct ParkExampleFixture
{
  std::vector<Sophus::SE3d> eye;
  std::vector<Sophus::SE3d> link;
  Sophus::SE3d expected_SE3;
  Eigen::Matrix<double, 6, 1> expected_se3;
  Eigen::Matrix<double, 6, 1> expected_se3_inv;

  ParkExampleFixture()
  {
    Eigen::Matrix4d A, B;
    Eigen::Matrix3d Ra, Rb;
    Eigen::Vector3d ta, tb;
    Ra << -0.989992, -0.141120,  0.,
           0.141120, -0.989992,  0.,
                 0.,        0., 1.;
    ta << 0., 0., 0.;
    Rb << -0.989992, -0.138307,  0.028036,
           0.138307,  -0.91149,  0.387470,
          -0.028036,  0.387470, 0.921456;
    tb << -26.9559, -96.1332, 19.4872;
    eye.push_back(Sophus::SE3d(Sophus::makeRotationMatrix(Ra), ta));
    link.push_back(Sophus::SE3d(Sophus::makeRotationMatrix(Rb), tb));


    Ra << 0.070737, 0., 0.997495,
                0., 1.,       0.,
          -0.997495, 0., 0.07037;
    ta << -400., 0., 400;

    Rb << 0.070737,  0.198172,  0.997612,
         -0.198172,  0.963323, -0.180936,
         -0.977612, -0.180936, 0.107415;
    tb << -309.543, 59.0244, 291.177;
    eye.push_back(Sophus::SE3d(Sophus::makeRotationMatrix(Ra), ta));
    link.push_back(Sophus::SE3d(Sophus::makeRotationMatrix(Rb), tb));

    Eigen::Matrix4d m;
    Eigen::Matrix3d Rm;
    Eigen::Vector3d tm;
    Rm << 1.0,       0.,        0.,
           0., 0.980067, -0.198669,
           0., 0.198669, 0.980067;
    tm << 10., 50, 100;
    expected_SE3 = Sophus::SE3d(Sophus::makeRotationMatrix(Rm), tm);
    expected_se3 = expected_SE3.log().matrix();
    expected_se3_inv = expected_SE3.inverse().log().matrix();
  }
};

} /* robcalib */
