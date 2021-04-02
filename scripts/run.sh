#!/bin/bash

# Source the workspace
cd ../workspace
. install/setup.bash

# Run packages
cd src
ros2 launch launch.py