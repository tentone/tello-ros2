# Set minimum required version of cmake, project name and compile options
cmake_minimum_required(VERSION 3.5)
project(tellopy_msg)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wpedantic")
endif()

# Find and load build settings from external packages
find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Declare ROS messages, services and actions
set(msg_files
  "msg/TestMsg.msg"
  "msg/FlightStatus.msg"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  DEPENDENCIES builtin_interfaces
)

# Macro for ament package
ament_export_dependencies(rosidl_default_runtime)
ament_package()
