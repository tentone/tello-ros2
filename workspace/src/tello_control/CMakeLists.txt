cmake_minimum_required(VERSION 3.5)
project(tello_control)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(tello_control src/main.cpp)
ament_target_dependencies(tello_control rclcpp std_msgs)

install(TARGETS tello_control DESTINATION lib/${PROJECT_NAME})

ament_package()