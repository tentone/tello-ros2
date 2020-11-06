#!/bin/bash

# ROS 2 Installation script

# Set UTF-8 charset
apt update && apt install locales
locale-gen en_US en_US.UTF-8
update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# Add ROS2 sources
apt update && apt install curl gnupg2 lsb-release
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add -
sh -c 'echo "deb [arch=$(dpkg --print-architecture)] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list'

# Install ROS 2 (foxy)
apt update
apt install ros-foxy-desktop

# Step envrioment
source /opt/ros/foxy/setup.bash

# Argcomplete
apt install -y python3-pip
pip3 install -U argcomplete

# Colcon build tools
apt install python3-colcon-common-extensions
apt install python3-rosdep2

# Update ROS dep
rosdep update