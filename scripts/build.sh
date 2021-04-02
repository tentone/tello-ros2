#!/bin/bash

cd ../workspace

rosdep install -i --from-path src
colcon build #--symlink-install