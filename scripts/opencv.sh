#!/bin/bash

if (( $EUID > 0 )); then
	echo " - Please run as root"
	exit
fi

# Branch used for opencv
CHANNEL='3.4' # 2.4 | 3.4 | master

# Create libs folder
echo "- Create libs folder"
mkdir -p ../libs
cd ../libs

# Install OpenCV from source
echo " - Installing Dependencies"
apt install -y cmake gcc g++ python3-dev python3-numpy libavcodec-dev libavformat-dev libswscale-dev libgstreamer-plugins-base1.0-dev libgstreamer1.0-dev libgtk-3-dev libpng-dev libjpeg-dev libopenexr-dev libtiff-dev libwebp-dev

# Clone the repository
echo " - Clone OpenCV repository"
cd ~
apt install -y git
git clone https://github.com/opencv/opencv.git
cd opencv

# Checkout to version branch
echo " - Fetch branch $CHANNEl"
git fetch
git checkout $CHANNEL

# Building files
echo " - Build OpenCV and Install"
mkdir build
cd build
cmake ..
make install -j8