
# OpenVINS Secondary Pose Graph Example


This is an example secondary thread which provides loop closure in a loosely coupled manner for [OpenVINS](https://github.com/rpng/open_vins).
This code was originally developed by the HKUST aerial robotics group and can be found in the [VINS-Fusion](https://github.com/HKUST-Aerial-Robotics/VINS-Fusion) repository.
Here we stress that this is a loosely coupled method, thus no information is returned to the estimator to improve the underlying OpenVINS odometry.
This codebase has been modified in a few key areas including: exposing more loop closure parameters, subscribing to camera intrinsics, simplifying configuration such that only topics need to be supplied, and some tweaks to the loop closure detection to improve frequency.
Please see the below sections on the dependencies of the system along with a discussion of how the loop closure logic works, its improvements, and limitations.



## Codebase Disclaimer

This code is provided as is as an example of how to use a secondary thread with the proposed system.
Thus, this code has been directly adapted from the [VINS-Fusion](https://github.com/HKUST-Aerial-Robotics/VINS-Fusion) repository without a thorough investigation of the underlying code.
Therefore we don't have any guarantee of the accuracy or correctness.
Additionally, for questions about the underlying implementation (besides discussing the changes introduced from the original codebase) we might not be able to answer in detail.



## Dependencies

* OpenVINS - https://docs.openvins.com/gs-installing.html
* Ceres Solver - https://github.com/ceres-solver/ceres-solver



## Installation Commands


```
# setup our workspace
mkdir -p catkin_ws_ov/src/
cd catkin_ws_ov
catkin init
# repositories to clone
cd src
git clone https://github.com/rpng/open_vins.git
git clone https://github.com/rpng/ov_secondary.git
# go back to root and build
cd ..
catkin build -j4
```











