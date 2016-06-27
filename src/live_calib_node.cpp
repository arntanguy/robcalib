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

#include <robcalib/calib_json.h>
#include <robcalib/logging.h>
#include <robcalib/pose_adapters.h>
#include <robcalib/ros_input_module.h>
#include <robcalib/time_utils.h>
#include <ros/ros.h>
#include <robcalib/calibration.hpp>

#include <boost/program_options.hpp>

using namespace robcalib;
using namespace std;

namespace po = boost::program_options;

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "robcalib");
  init_log();

  // Declare the supported options.
  po::options_description desc("Options");
  desc.add_options()("help,h", "produce help message")(
      "config,c", po::value<std::string>()->default_value("etc/robcalib.conf"), "json configuration file");

  po::variables_map vm;
  po::store(po::parse_command_line(argc, argv, desc), vm);
  po::notify(vm);

  if (vm.count("help"))
  {
    cout << desc << "\n";
    return 0;
  }

  std::string config_path = vm["config"].as<std::string>();
  // robcalib::HeadEyeCalib calib;
  // robcalib::ROSInputModule input;
  // input.set_data_callback([&calib, &input](const CalibData& d)
  //                          {
  //                          const auto& poses = d.poses_;
  //                          LOG(info) << "Received " << d.rel_poses_size() << " relative poses" << std::endl;
  //                          if(d.rel_poses_size() < 20) return;

  //                          auto avg = measure<>::duration([&calib, &d]() {
  //                           calib.createProblem(std::make_shared<CalibData>(d));
  //                           auto res = calib.run();
  //                           });
  //                          LOG(info) << "Time taken: " << avg.count() << "ms" << std::endl;
  //                          exit(0);
  //                          });

  // HRP
  // std::vector<Sophus::SE3d::Tangent> known_x(3);
  // known_x[0].setZero();
  // known_x[1] << -0.04, 0, 0.434, 0, 0, 0;
  // known_x[2].setZero();
  // robcalib::KinematicsEyeCalib calib({1,3}, known_x);

  CalibJson config(config_path);

  std::vector<size_t> estimate_x ;
  estimate_x = config.estimate_x;
  for (const auto& x : estimate_x)
  {
    std::cout << "Estimating x: " << x << std::endl;
  }
  std::vector<Sophus::SE3d::Tangent> known_x(config.joints.size() - estimate_x.size() - 1);
  // Set to actual value
  for (auto& v : known_x)
  {
    v.setZero();
  }

  robcalib::ROSInputModule input(config.joint_topic);
  input.setCameraFrame(config.camera_frame);
  input.setJoints(config.getJoints());
  input.data()->adapter.step = config.relative_step;
  tf2_ros::TransformBroadcaster tf_b;

  // CHEST, NECK, camera live calib from encoders
  if (config.method == CalibJson::Method::KINEMATICS)
  {
    for (const auto& x : estimate_x)
    {
      std::cout << "Estimating x: " << x << std::endl;
    }
    robcalib::KinematicsEyeCalib calib(estimate_x, known_x);

    // TODO Make independent so that calibration can be run in separate thread!
    // ie decouple input and calib
    input.set_data_callback([&calib, &input, &tf_b, &config](const std::shared_ptr<CalibData> d) {
      LOG(info) << "Ros Input callback: " << d->rel_poses_size() << std::endl;

      // LOG(info) << "Data: \n" << d << std::endl;
      if (d->rel_poses_size() >= config.n_pairs)
      {
        LOG(info) << "Running Robot-Eye calib with " << d->rel_poses_size() << " poses";
        // Keep only the last N poses (absolute)
        // d->rolling_window(config.n_pairs * 2 * 20);

        auto avg = measure<>::duration([&calib, &d]() {
          calib.createProblem(d);
          auto res = calib.run();
          roboptim::ResultWithWarnings& result = boost::get<roboptim::ResultWithWarnings>(res);
          LOG(info) << "Robot-Eye results: " << result.x << std::endl;

          tf2_ros::TransformBroadcaster tf_b;
          for (size_t i = 0; i < static_cast<size_t>(result.x.size()); i += 6)
          {
            const auto id = i / 6;
            const Sophus::SE3d::Tangent& x = result.x.block<6, 1>(i, 0);
            const auto& j_from = id;
            const auto& j_to = id+1;
            LOG(info) << "X" << id << " (" << j_from << " -> " << j_to << "): " << x.transpose().matrix() << std::endl;
            Eigen::Vector3d xyz, rpy;
            se3toXYZRPY(x, xyz, rpy);
            LOG(info) << "xyz: " << xyz.transpose() << ", rpy " << rpy.transpose();

            // Publish calibration results as TF
            Sophus::SE3d X = Sophus::SE3d::exp(x);
            geometry_msgs::TransformStamped calib_tf = tf2::eigenToTransform(Eigen::Affine3d(X.matrix()));
            if(j_from == 0)
            {
              calib_tf.header.frame_id = "/robot_map";
            }
            else
            {
              calib_tf.header.frame_id = "/robot_map";
            }
            calib_tf.child_frame_id = std::string("calib/") + std::to_string(j_from) + std::string("_") + std::to_string(j_to);
            calib_tf.header.stamp = ros::Time::now();
            tf_b.sendTransform(calib_tf);
          }
        });
        LOG(info) << "Time taken: " << avg.count() << "ms" << std::endl;
      }
    });
  }
  else if(config.method == CalibJson::Method::HEAD_EYE)
  {
    input.set_head_eye_data_callback([&tf_b, &input, &config](const std::shared_ptr<HeadEyeData> d) {
      LOG(info) << "Getting head eye data";
      std::shared_ptr<HeadEyeData> rel_data = std::make_shared<HeadEyeData>();
      for (size_t j = 0; j < d->eye.size(); j += 20)
      {
        rel_data->link.push_back(d->eye[0].inverse() * d->eye[j]);
        rel_data->eye.push_back(d->link[0].inverse() * d->link[j]);
      }

      Eigen::Affine3d X;
      LOG(info) << "Relative data size: " << rel_data->eye.size();
      if (rel_data->eye.size() > 10)
      {
        robcalib::HeadEyeCalib head_eye_calib;
        head_eye_calib.initSolverImpl(rel_data);
        auto res = head_eye_calib.runImpl();
        roboptim::ResultWithWarnings& result = boost::get<roboptim::ResultWithWarnings>(res);
        X = Eigen::Affine3d(Sophus::SE3d::exp(result.x).matrix());
        geometry_msgs::TransformStamped calib_tf = tf2::eigenToTransform(X);
        calib_tf.header.frame_id = "robot_map";
        calib_tf.child_frame_id = "xtion_calib_live";
        // Setting the timestamp is important!
        calib_tf.header.stamp = ros::Time::now();
        tf_b.sendTransform(calib_tf);
      }
      else
      {
        using namespace Eigen;
        Matrix3d m;
        m = AngleAxisd(2.5*M_PI, Vector3d::UnitX())
            * AngleAxisd(2.5*M_PI,  Vector3d::UnitY())
            * AngleAxisd(-2.5*M_PI, Vector3d::UnitZ());
        cout << m << endl << "is unitary: " << m.isUnitary() << endl;

        // Publish unknown guess tf for generating the paper video
        Eigen::Affine3d X = Eigen::Affine3d::Identity();
        X.matrix().block<3,1>(0,3) << 0.3, 0.3, 0.3;
        X.matrix().block<3,3>(0,0) << m;

        geometry_msgs::TransformStamped calib_tf = tf2::eigenToTransform(X);
        calib_tf.header.frame_id = "robot_map";
        calib_tf.child_frame_id = "xtion_calib_live";
        calib_tf.header.stamp = ros::Time::now();
        tf_b.sendTransform(calib_tf);
      }
    });
  }
  else
  {
    LOG(error) << "Unsupported calibration method";
    return 1;
  }

  return 0;
}
