#!/bin/bash

if (( $EUID > 0 )); then
	echo " - Please run as root"
	exit
fi

#Install ROS 2
echo " - Installing ROS 2 Foxy"

echo " - Install Build Tools"

# C++ Build tools
apt install build-essential gdb

# Set UTF-8 charset
apt update
apt install -y locales
locale-gen en_US en_US.UTF-8
update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

echo " - ROS 2 sources"

# Add ROS2 sources
apt update
apt install -y curl gnupg2 lsb-release
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add -
sh -c 'echo "deb [arch=$(dpkg --print-architecture)] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list'

echo " - Install ROS 2"

# Install ROS 2 (foxy)
apt update
apt install -y ros-foxy-desktop

# Step envrioment
source /opt/ros/foxy/setup.bash

echo " - Install Python ROS 2"

# Argcomplete
apt install -y python3-pip
pip3 install -U argcomplete

# Colcon build tools
apt install -y python3-colcon-common-extensions python3-rosdep2

# Update ROS dep
rosdep update
rosdep fix-permissions

# Add to bashrc
echo " - Register ROS 2 in .bashrc"
echo "source /opt/ros/foxy/setup.bash" >> ~/.bashrc
echo "source /usr/share/colcon_cd/function/colcon_cd.sh" >> ~/.bashrc
echo "export _colcon_cd_root=~/ros2_install" >> ~/.bashrc
source ~/.bashrc

# Install project dependencies
echo " - Python dependencies"
pip3 install catkin_pkg rospkg av image opencv-python djitellopy2 pyyaml
apt install python3-tf*

echo " - CPP dependencies"
apt install ros-foxy-ament-cmake* ros-foxy-tf2* ros-foxy-rclcpp* ros-foxy-rosgraph*

echo " - Rviz and RQT Tools"
apt install ros-foxy-rviz* ros-foxy-rqt*
