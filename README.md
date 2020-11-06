# Drone Visual SLAM

- Collaborative drone based visual SLAM for mapping of indoor environments using ROS.



### Project

- Project workspace is divided into sub-workspaces that contain different logic packages.
  - gui: Contain User interface related packages
  - driver: Access to hardware control to collect data from hardware
  - control: Receives inputs (sensors, vision systems, etc) and defines how the drone is controlled.
  - vision: Vision algorithms for visual information processing.



### ROS 2

- Run the install script to setup the ROS 2 (Foxy Fitzroy) environment. 
- Check the [ROS2 Tutorials](https://index.ros.org/doc/ros2/Tutorials/) page to learn how to setup workspace and create packages.

##### Workspace

- To install dependencies of the packages available in a workspace directory `src` run `rosdep install -i --from-path src --rosdistro foxy -y`
- To build workspace you can use the command `colcon build`,  some useful arguments for `colcon build`:

  - `--packages-up-to` builds the package you want, plus all its dependencies, but not the whole workspace (saves time)
  - `--symlink-install` saves you from having to rebuild every time you tweak python scripts
  - `--event-handlers console_direct+` shows console output while building (can otherwise be found in the `log` directory)

##### Packages

- To create a new ROS2 package (for C++ or Python) development

```bash
ros2 pkg create --build-type ament_cmake --node-name <node_name> <package_name>
ros2 pkg create --build-type ament_python --node-name <node_name> <package_name>
```



### Windows Subsystem for Linux (WSL)

- Install WSL2 from the windows store or using the commands bellow, install Ubuntu 20.04 as the SO over the WSL overlay.

```powershell
# Install WSL 2
Enable-WindowsOptionalFeature -Online -FeatureName Microsoft-Windows-Subsystem-Linux

# Enable WSL 2
dism.exe /online /enable-feature /featurename:VirtualMachinePlatform /all /norestart

# Check WSL version
wsl -l -v
```

- Install a [VcXsrv Windows X Server](https://sourceforge.net/projects/vcxsrv/) to be used as a X11 display server required to run GUI applications.
- To enable the access to the installed server add the display address to the `.bashrc` file

```bash
export DISPLAY="`grep nameserver /etc/resolv.conf | sed 's/nameserver //'`:0"
export LIBGL_ALWAYS_INDIRECT=1
```

- If you are using Visual Studio Code as and IDE you can configure for [remote WSL development](https://code.visualstudio.com/docs/cpp/config-wsl), allowing to debug code and interact with the WSL terminal.

