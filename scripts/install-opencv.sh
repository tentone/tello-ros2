#!/bin/bash

echo " - Install Build Tools"

# C++ Build tools
apt install build-essential gdb

# Install OpenCV from source
echo " - Installing OpenCV"
apt install -y cmake gcc g++ python3-dev python3-numpy libavcodec-dev libavformat-dev libswscale-dev libgstreamer-plugins-base1.0-dev libgstreamer1.0-dev libgtk-3-dev libpng-dev libjpeg-dev libopenexr-dev libtiff-dev libwebp-dev

# Clone the repository
cd ~
apt install -y git
git clone https://github.com/opencv/opencv.git

# Building files
cd opencv
mkdir build
cd build
cmake ..
make install -j4
