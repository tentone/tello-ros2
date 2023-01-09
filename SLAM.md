# Tello OrbSLAM

 - If the version of ORBSlam is not working, you might attempt to use these instructions provided by @zoldaten
 - More detail about these instruction available in the [issues section](https://github.com/tentone/tello-ros2/issues/7).
 - These instructions exaplain the entire process to install and setup ROS and install the Tello-ROS2 library alonside with ORB Slam 2


## Install ROS Galactic

``` bash
sudo apt update
sudo apt install curl gnupg2 lsb-release
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS Foxy
sudo apt install ros-foxy-desktop python3-argcomplete ros-foxy-ros-base python3-argcomplete ros-dev-tools

# Setup ROS Foxy
source /opt/ros/foxy/setup.bash
ros2 run demo_nodes_cpp talker
echo "source /opt/ros/foxy/setup.bash" >> ~/.bashrc
printenv | grep -i ROS
sudo rm -r ros2_galactic/
printenv | grep -i ROS
mkdir -p ~/ros2_ws/src

# Create a workspace
cd ~/ros2_ws
cd src/
cd ..
git clone https://github.com/ros2/examples src/examples -b foxy
colcon build --symlink-install
colcon test
. install/setup.bash
```

### Install Tello ROS2

 - In this section we get the code from the github repository and configure it in a ROS workspace.

``` bash
git clone https://github.com/tentone/tello-ros2.git
put part of ROS to ros2_ws/src
cd ros2_ws/
colcon build --symlink-install
. install/setup.bash
cd src/
ls
ros2 launch launch.py
pip3 install av
sudo apt install python3-pip
ros2 launch launch.py
pip3 install djitellopy
```

## ORBSLAM 2

 - This step assumes that OpenCV 4.# is installed.
 - First we need to clone the orbSLAM 2 project.

``` bash
git clone https://github.com/raulmur/ORB_SLAM2.git ORB_SLAM2
cd ORB_SLAM2/
chmod +x build.sh
./build.sh
sed -i 's/++11/++14/g' CMakeLists.txt
```

 - Then we need to add the following code to the files bellow:
    - ORB_SLAM2/src/LoopClosing.cc
    - ORB_SLAM2/src/Viewer.cc
    - ORB_SLAM2/src/System.cc
    - ORB_SLAM2/src/Tracking.cc
    - ORB_SLAM2/src/System.cc
    - ORB_SLAM2/src/LocalMapping.cc
    - ORB_SLAM2/Examples/Monocular/mono_kitti.cc
    - ORB_SLAM2/Examples/Stereo/stereo_kitti.cc
    - ORB_SLAM2/Examples/Monocular/mono_tum.cc
    - ORB_SLAM2/Examples/RGB-D/rgbd_tum.cc
    - ORB_SLAM2/Examples/Monocular/mono_euroc.cc
    - ORB_SLAM2/Examples/Stereo/stereo_euroc.cc
``` c
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
```

 - Change the code in the file  `include/LoopClosing.h` from:

``` c++
Eigen::aligned_allocator<std::pair<const KeyFrame*, g2o::Sim3> > > KeyFrameAndPose;
```
to
``` c++
Eigen::aligned_allocator<std::pair<KeyFrame* const, g2o::Sim3> > > KeyFrameAndPose;
```

changes concern opencv

``` bash
*raulmur/ORB_SLAM2@379c763
*ducha-aiki/pymagsac#4
sed -i 's/++11/++14/g' CMakeLists.txt
```

 - Then we need to replace come content in the `CMakeLists.txt` replace:
``` cmake
find_package(Eigen3 REQUIRED)
```
to

``` cmake
list(APPEND CMAKE_INCLUDE_PATH "/usr/local/include")
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(OpenCV 4 QUIET)
```

 - Finally we need to declare the `ost.yaml` file in the `ros2_ws/build/tello/tello/node.py` file and change:
``` python
self.node.declare_parameter('camera_info_file', '')
```
to
``` python
self.node.declare_parameter('camera_info_file', '/ros2_ws/src/tello/resource/ost.yaml')
```
 - The same parameters should also be placed in the file `~/ORB_SLAM2/Examples/Monocular/TUM1.yaml`