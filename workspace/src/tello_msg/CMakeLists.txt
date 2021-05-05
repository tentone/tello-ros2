# Set minimum required version of cmake, project name and compile options
cmake_minimum_required(VERSION 3.5)
project(tello_msg)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wpedantic")
endif()

# Find and load build settings from external packages
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# Declare ROS messages, services and actions
set(msg_files
  "msg/TelloStatus.msg"
  "msg/TelloID.msg"
  "msg/TelloWifiConfig.msg"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  DEPENDENCIES std_msgs geometry_msgs
  ADD_LINTER_TESTS
)

# Macro for ament package
ament_export_dependencies(rosidl_default_runtime)
ament_package()
