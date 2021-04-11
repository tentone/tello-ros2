#!/bin/bash

./install-ros.sh

# Project dependencies
echo " - Python dependencies"
pip3 install catkin_pkg rospkg rosdep2 av

echo " - CPP dependencies"
apt install ros-foxy-ament-cmake* ros-foxy-tf2* ros-foxy-rclcpp*

./install-libs.sh