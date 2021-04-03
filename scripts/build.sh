#!/bin/bash

cd ../workspace
rm -rf build install log

touch *
rosdep install -i --from-path src
colcon build --symlink-install