#!/bin/bash

./install-ros.sh

echo "Install Tello ROS Node"

apt install libasio-dev ros-eloquent-cv-bridge ros-eloquent-camera-calibration-parsers

mkdir -p ~/tello_ros_ws/src
cd ~/tello_ros_ws/src
git clone https://github.com/clydemcqueen/tello_ros.git
git clone https://github.com/ptrmu/ros2_shared.git
cd ..
source /opt/ros/eloquent/setup.bash
# If you didn't intall Gazebo, skip tello_gazebo while building:
colcon build --event-handlers console_direct+ --packages-skip tello_gazebo