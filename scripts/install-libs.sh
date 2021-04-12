#!/bin/bash

echo " - Tello ROS Node"
apt install libasio-dev ros-foxy-cv-bridge ros-foxy-camera-calibration-parsers
pip3 install empy lark

mkdir ../workspace/src/lib
cd ../workspace/src/lib
git clone https://github.com/clydemcqueen/tello_ros.git
git clone https://github.com/ptrmu/ros2_shared.git
