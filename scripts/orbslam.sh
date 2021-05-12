#!/bin/bash

if (( $EUID > 0 )); then
	echo " - Please run as root"
	exit
fi

echo " - Install dependencies"
apt install ros-foxy-vision-opencv ros-foxy-message-filters libeigen3-dev

echo "- Create libs folder"
mkdir -p ../libs
cd ../libs

echo " - Install Pangolin"
apt install libgl1-mesa-dev libglew-dev cmake libegl1-mesa-dev libwayland-dev libxkbcommon-dev wayland-protocols ffmpeg libavcodec-dev libavutil-dev libavformat-dev libswscale-dev libavdevice-dev libjpeg-dev libpng-dev libtiff5-dev libopenexr-dev
git clone https://github.com/stevenlovegrove/Pangolin.git
cd Pangolin
mkdir build
cd build
cmake ..
make install -j8

echo " - Install G2O"
apt install cmake libeigen3-dev libsuitesparse-dev qtdeclarative5-dev qt5-qmake libqglviewer-dev-qt5
git clone https://github.com/RainerKuemmerle/g2o.git
cd g2o
mkdir build
cd build
cmake ..
make install -j8
cd ../../libs

echo " - Install ORB SLAM 2"
git clone https://github.com/raulmur/ORB_SLAM2
cd ORB_SLAM2
chmod +x build.sh
./build.sh

echo "export ORB_SLAM2_ROOT_DIR=$(pwd)" >> ~/.bashrc

echo " - Download ORB SLAM 2 Wrapper"
mkdir -p ../workspace/src/libs
cd ../workspace/src/libs
git clone https://github.com/alsora/ros2-ORB_SLAM2

