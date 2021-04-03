#!/bin/bash

./install-ros.sh

echo "Install Tello ROS Node"

apt install libasio-dev ros-foxy-cv-bridge ros-foxy-camera-calibration-parsers

cd ~ 
mkdir -p ~/tello_ros_ws/src
cd ~/tello_ros_ws/src
git clone https://github.com/clydemcqueen/tello_ros.git
git clone https://github.com/ptrmu/ros2_shared.git

cd ..
source /opt/ros/foxy/setup.bash
colcon build --event-handlers console_direct+ --packages-skip tello_gazebo