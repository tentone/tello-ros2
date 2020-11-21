#!/bin/bash

# C++ Build tools
apt install build-essential gdb

#Install ROS 2
echo " - Installing ROS 2 Foxy"

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

echo " - Installing OpenCV"

# Install OpenCV from source
apt install cmake gcc g++
apt install python3-dev python3-numpy
apt install libavcodec-dev libavformat-dev libswscale-dev libgstreamer-plugins-base1.0-dev libgstreamer1.0-dev libgtk-3-dev
apt install libpng-dev libjpeg-dev libopenexr-dev libtiff-dev libwebp-dev

# Clone the repository
cd ~
apt-get install git
git clone https://github.com/opencv/opencv.git

# Building files
cd opencv
mkdir build
cd build
cmake ..
make install -j4