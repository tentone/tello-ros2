#!/bin/bash


echo " - Installing ROS 1 Noetic"
E
sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
apt update
apt install build-essential gdb ros-noetic-desktop-full


echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source ~/.bashrc

echo " - Python dependencies"

apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential

rosdep init
rosdep update

echo " - ROS 1 to ROS2 bridge"

apt install -y ros-foxy-rosbag2-bag-v2-plugins