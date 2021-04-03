#!/bin/bash

./install-ros.sh

# Project dependencies
echo " - Install Python dependencies"
pip3 install catkin_pkg rospkg rosdep2 av

echo " - Install CPP dependencies"
apt install ros-foxy-ament-cmake* ros-foxy-tf2* ros-foxy-rclcpp*

echo " - Install Tello ROS Node"

apt install libasio-dev ros-foxy-cv-bridge ros-foxy-camera-calibration-parsers

cd ~ 
mkdir -p ~/tello_ros_ws/src
cd ~/tello_ros_ws/src
git clone https://github.com/clydemcqueen/tello_ros.git
git clone https://github.com/ptrmu/ros2_shared.git

cd ..
source /opt/ros/foxy/setup.bash
colcon build --event-handlers console_direct+ --packages-skip tello_gazebo