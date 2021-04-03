#!/bin/bash

./install-ros.sh

# Project dependencies
echo " - Python dependencies"
pip3 install catkin_pkg rospkg rosdep2 av

echo " - CPP dependencies"
apt install ros-foxy-ament-cmake* ros-foxy-tf2* ros-foxy-rclcpp*

echo " - Tello ROS Node"
apt install libasio-dev ros-foxy-cv-bridge ros-foxy-camera-calibration-parsers
pip3 install empy

mkdir ../workspace/src/lib
cd ../workspace/src/lib
git clone https://github.com/clydemcqueen/tello_ros.git
git clone https://github.com/ptrmu/ros2_shared.git

source /opt/ros/foxy/setup.bash
colcon build --event-handlers console_direct+ --packages-skip tello_gazebo