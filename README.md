


Online Eye-Robot Self-Calibration
==

This is the code for the 2018 SIMPAR paper *Online Eye-Robot Self-Calibration* by [Arnaud TANGUY](http://www.i3s.unice.fr/robotvision/index.php/the-team-2/arnaud-tanguy), [Andrew Ian Comport](http://www.i3s.unice.fr/robotvision/index.php/the-team-2/andrew-comport) and [Abderrahmane Kheddar](http://www.lirmm.fr/lirmm_eng/users/utilisateurs-lirmm/equipes/idh/abderrahmane-kheddar).
The code is provided as-is

```
@article{tanguy:simpar:2018,
author = {Tanguy, Arnaud and Kheddar, Abderrahmane and Comport, Andrew Ian},
journal = {Proceedings - IEEE International Conference on Simulation, Modeling, and Programming for Autonomous Robots (SIMPAR)}
title = {{Online Eye-Robot Self-Calibration}},
year = {2018}
}
```

If you use this code, please cite this publication.


Description
==

`robcalib` is a library aiming at solving general robot calibration issues, through the use of a *Link-Eye* (See [1]) inspired approach.
The basic idea of Link-Eye approches is to derive a correlation between the relative motion of the camera (as measured by visual odometry for instance)
and the relative motion of any link it's rigdly attached to (typically obtained through encoders).

The approach provides:

- A *Hand-Eye* calibration method parametrized on SE(3), iteratively estimating both translation and rotation at the same time.
- A *Robot-Eye* calibration method, that extends the approach to whole body calibration, in which any link directly or indirectly responsible for moving the camera
may be estimated. It relies solely on encoder values, intrisic joint information (ie joint rotation axis in case of revolute joins), and on a visual odometry method responsible of tracking the camera motion.


Dependencies
==
We rely on the excellent [roboptim](http://roboptim.net) framework to solve the optimization problems.

The following modules are required:

- [roboptim-core 3.2](https://github.com/roboptim/roboptim-core/releases/tag/v3.2)
- [robotoptim-core-plugin-eigen git >3.2](https://github.com/roboptim/roboptim-core-plugin-eigen/tree/99f2ed76811b49bf3971103455338031693318f6) Note feel free to use your favorite solver instead.
The only requirement is that it supports "levenberg-marquart" non-linear optimization.
- [Sophus](https://github.com/strasdat/Sophus) Sophus is an Eigen-based library used for Lie Algebra computations. 
- jsoncpp : used for configuration

The following modules are optional:

- [ROS Indigo](http://wiki.ros.org/indigo) [Optional] Used to provide tools to simplify calibration of your own robot.
Additional modules are require: tf2-eigen
  The ROS module relies on the tf graph and an URDF model of the robot to gather the following information:
  - The camera odometry. 
  - Joint encoders
A test configurable application is also provided and will automatically set-up and run the calibration procedure for your robot.

How to intall
==

After installing the required dependencies above, the package may be cloned and installed in the following way:

```
git clone <.../robcalib.git>
git submodule init
mkdir build
cd build
cmake ..
make
```


Test Coverage
==

Unit tests are important in ensuring the proper behavior of any program. This is particularely important
for a complex problem such as calibration. Many sources of uncertainty can creep up during the process,
noisy odometry, ecoder values, local minimas... It is our reponsibility to ensure that the optimization
process is as robust as possible. As such care is taken to provide unit tests that check the validity where applicable:

- Link-Eye calibration procedure is tested on the two-poses example of Park [2], for which an exact solution is known.
- Kinematics chain calibration is tested on a simulated HRP-4 calibration dataset with known groundtruth

```
cd build
cmake -DBUILD_TESTS=ON ..
make test
```

How to use
==

Library
===

If you mean to use it as a calibration library, you will need to fill the `CalibData` structure with the appropriate data for your problem, and pass it to the appropriate solver, currently either `HeadEyeCalib` for standard Hand-Eye calibration or `KinematicsEyeCalib` for whole Robot-Eye calibration.


Test Application
==

If built with ROS support, the test application `test_ros_input` will be built. As with the library version, you need to define what you want to optimize. To help with that, a configuration file `etc/robcalib.conf` is provided.
Not that the application has only been tested in the specific use-case described in the paper.

For head-eye calibration, it should be something like

```
{
  // "kinematics" or "head_eye" calibration
  "method": "kinematics"
  "robot" : "NAO",
  // Joints used for calibration
  "joints" : {
    // Joint names
    "joints" : ["HeadYaw", "HeadPitch", "xtion_joint"],
    // RPY axis of the joint
    "axes" : [[0,0,1], [0,1,0], [0,0,0]]
  },
  // Camera pose from odometry
  "camera_frame": ["map", "camera_rgb_frame"]

  // Number of pairs of poses to use for estimation
  "n_pairs" : 500,
  // Compute relative pose for every absolute one.
  // Use a higher value to compute a relative pose every N pairs
  "relative_step": 1,

  // We are only interested in estimating the pose from joint 1 to joint 2: HeadPitch->xtion_joint
  // The rest is supposed known, and has values defined in known_x
  "estimate_x" : [1]
  // "known_x" : [0,0,0,0,0,0]
}
```


And here is an example for robot-eye kinematic chain calibration of an HRP4 torso and head links.
```
{
  "robot" : "HRP4",
  // Joints used for calibration
  "joints" : {
    "topic" : "/real/joint_states",
    // Joint names
    "joints" : ["CHEST_P", "CHEST_Y", "NECK_Y", "NECK_P", "xtion_joint"],
    // RPY axis of the joint (intrinsic joint parameters)
    "axes" : [[0,1,0], [0,0,1], [0,0,1], [0,1,0], [0,0,0]]
  },

  // Camera pose (retrieves pose from the TF-tree published by SLAM)
  "camera_frame": ["map", "camera_rgb_frame"],

  // Number of pairs of poses to use for estimation
  "n_pairs" : 20,

  // We are only interested in estimating the pose from joint 1 to joint 2: HeadPitch->xtion_joint
  // The rest is supposed known, and has values defined in known_x
  "estimate_x" : [1,3],
  "known_x" : [0,0,0,0,0,0, 0,0,0,0,0,0]
}
```

Here `CHEST_P` and `CHEST_Y` joints are considered to share the same rotation center, and act as aball-joint, so are `NECK_Y` and `NECK_P`. This provides sufficient constraints to fully constrain the estimation process. See the paper for details. 
The transform between `CHECK_Y -> NECK_Y` and `NECK_P -> xtion_joint` will be estimated, providing calibration for the upper-body part.

Main References
==

- [1] R. Y. Tsai and R. K. Lenz, “A New Technique for Fully Autonomous and Efficient 3D Robotics Hand/Eye Calibration,” IEEE Trans. Robot. Autom., vol. 5, no. 3, pp. 345–358, 1989.

- [2] F. C. Park and B. J. Martin, “Robot Sensor Calibration: Solving AX = XB on the Euclidean Group,” IEEE Trans. Robot. Autom., vol. 10, no. 5, pp. 717–721, 1994.
