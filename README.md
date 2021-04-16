# Tello SLAM

- DJI Tello visual SLAM for mapping of indoor environments using ROS.

- Project workspace is divided into sub-workspaces that contain different logic packages.
  - gui: Contain User interface related packages
  - driver: Access to hardware control to collect data from hardware
  - control: Receives inputs (sensors, vision systems, etc) and defines how the drone is controlled.
  - vision: Vision algorithms for visual information processing.



### ROS 2 Foxy

- Run the install script to setup the ROS 2 (Foxy Fitzroy) environment. 
- Check the [ROS2 Tutorials](https://index.ros.org/doc/ros2/Tutorials/) page to learn how to setup workspace and create packages.

##### Workspace

- To install dependencies of the packages available in a workspace directory `src` run `rosdep install -i --from-path src --rosdistro foxy -y`
- To build workspace you can use the command `colcon build`,  some useful arguments for `colcon build`:

  - `--packages-up-to` builds the package you want, plus all its dependencies, but not the whole workspace (saves time)
  - `--symlink-install` saves you from having to rebuild every time you tweak python scripts
  - `--event-handlers console_direct+` shows console output while building (can otherwise be found in the `log` directory)

##### Packages

- To create a new ROS2 package (C++ or Python) for development move to the `src` package and run

```bash
# CPP Package
ros2 pkg create --build-type ament_cmake --node-name <node_name> <package_name>

# Python Package
ros2 pkg create --build-type ament_python --node-name <node_name> <package_name>
```

##### Tools

- `rqt_topic` Used to monitor topics and their values in a list
- `rqt_graph` Draw the graph of connection between the currently active nodes and explore communication between them
- `rviz` Visualize topics in 3D space.

##### Bags

- Bags can be used to record data from topics that can be later replayed for off-line testing. Bags can be manipulated using the `ros2 bag` command. To 

```bash
# Record a bag containing data from some topics into a file
ros2 bag record -o <bag_file_name> /turtle1/cmd_vel /turtle1/pose ...

# Check the content of a bag run the command
ros2 bag info <bag_file_name>

# Replay the content of some topics recorded into a bag file
 ros2 bag play <bag_file_name>
```

- To play ROS 1 bags in ROS 2 you will need to first install ROS 1, and the ROS bag adapter plugin. The the bags can be run using the command.

```bash
ros2 bag info -s rosbag_v2 <path_to_bagfile>
```

##### Camera calibration

- Calibration files provided were obtained using our test drone.
- To get your own calibration file use the [ROS camera calibration tool]()



### Linux 

- When installing on ubuntu based distros it might be required to change the distro codename so that the `lsb_release -cs` command returns the correct ubuntu base distribution.
- To change the output of the `lsb_release` command edit the `/etc/os-release` file. For ubuntu 20.04 the codename should be `focal`.
- Edit the file to contain the value `UBUNTU_CODENAME=focal`.



### Windows Subsystem for Linux (WSL)

- Install WSL2 from the windows store or using the commands bellow, install Ubuntu 20.04 as the SO over the WSL overlay.

```powershell
# Install WSL 2
Enable-WindowsOptionalFeature -Online -FeatureName Microsoft-Windows-Subsystem-Linux

# Enable WSL 2
dism.exe /online /enable-feature /featurename:VirtualMachinePlatform /all /norestart

# Check WSL version
wsl.exe --set-default-version 2
wsl -l -v
```

- Install a [VcXsrv Windows X Server](https://sourceforge.net/projects/vcxsrv/) to be used as a X11 display server required to run GUI applications.
  - "Native opengl" unchecked
  - "Disable access control" checked
- Create a shortcut for VcXSrv with the following parameters

```powershell
"C:\Program Files\VcXsrv\vcxsrv.exe" :0 -ac -terminate -lesspointer -multiwindow -clipboard -wgl -dpi auto
```

- To enable the access to the installed server add the display address to the `.bashrc` file

```bash
export DISPLAY="`grep nameserver /etc/resolv.conf | sed 's/nameserver //'`:0"
export LIBGL_ALWAYS_INDIRECT=0
```

- If you are using Visual Studio Code as and IDE you can configure for [remote WSL development](https://code.visualstudio.com/docs/cpp/config-wsl), allowing to debug code and interact with the WSL terminal.
- If you require CUDA acceleration you can also install [NVidia CUDA drivers for WSL2](https://developer.nvidia.com/blog/announcing-cuda-on-windows-subsystem-for-linux-2/)

- If you get `Clock skew detected. Your build may be incomplete.` while compiling the code run the following commands or install the [wsl-clock](https://github.com/stuartleeks/wsl-clock) tool to automatically fix the clock drift problems.

```bash
sudo apt install ntpdate
sudo ntpdate time.windows.com
```

