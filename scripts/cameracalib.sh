#!/bin/bash

if (( $EUID > 0 )); then
	echo " - Please run as root"
	exit
fi

echo " - Install Camera Calibration"
apt install ros-foxy-camera-calibration ros-foxy-camera-calibration-parsers ros-foxy-camera-info-manager ros-foxy-launch-testing-ament-cmake
