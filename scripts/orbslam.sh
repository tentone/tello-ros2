#!/bin/bash

echo " - Install dependencies"
apt install ros-foxy-vision-opencv ros-foxy-message-filters libeigen3-dev

echo "- Create libs folder"
mkdir -p ../libs
cd ../libs

echo " - Install G2O"
apt install cmake libeigen3-dev libsuitesparse-dev qtdeclarative5-dev qt5-qmake libqglviewer-dev
git clone https://github.com/RainerKuemmerle/g2o.git
cd g2o
mkdir build
cd build
cmake ..
make install

echo " - Download ORB SLAM 2"
git clone https://github.com/alsora/ORB_SLAM2
cd ORB_SLAM2
chmod +x build.sh
./build.sh

echo " - Download ORB SLAM 2 ROS node"
cd workspace/src
git clone https://github.com/alsora/ros2-ORB_SLAM2


# ros2 run orbslam mono ~/ORB_SLAM2/Vocabulary/ORBvoc.txt ~/ORB_SLAM2/ros2-ORB_SLAM2/src/monocular/TUM1.yaml