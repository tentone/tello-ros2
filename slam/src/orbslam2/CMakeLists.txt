cmake_minimum_required(VERSION 3.5)
project(orbslam2)

# Cmake modules folder
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}/CMakeModules/")

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(message_filters REQUIRED)

find_package(Boost COMPONENTS system) 
find_package(OpenCV 3 REQUIRED)
find_package(Eigen REQUIRED)
find_package(ORB_SLAM2 REQUIRED)

include_directories(
  ${ORB_SLAM2_INCLUDE_DIRS}
  ${message_filters_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${EIGEN_INCLUDE_DIR}
)

# Monocular
add_executable(
    mono 
    src/monocular/mono.cpp
    src/monocular/monocular-slam-node.cpp
)

target_link_libraries(mono
  g2o
  DBoW2
  ${ORB_SLAM2_LIBRARIES}
  ${Boost_LIBRARIES}
)

ament_target_dependencies(mono rclcpp sensor_msgs visualization_msgs cv_bridge ORB_SLAM2 Boost)

install(TARGETS mono DESTINATION lib/${PROJECT_NAME})
ament_package()
