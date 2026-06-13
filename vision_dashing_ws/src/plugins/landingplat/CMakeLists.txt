cmake_minimum_required(VERSION 3.5)
project(landingplat)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(ament_cmake REQUIRED)
find_package(gazebo_dev REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(gazebo REQUIRED)

include_directories(include ${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
# Append (as a string) Gazebo's own compile flags, e.g. -fPIC -std=c++17.
# NOTE: do not use list(APPEND ...) here — that makes CMAKE_CXX_FLAGS a
# semicolon-separated list and the flags get split into bogus commands
# (e.g. "/bin/sh: 1: -fPIC: not found").
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

add_library(platformPlugin SHARED
  src/plugin.cpp
  src/WaveGen.cpp)

ament_target_dependencies(platformPlugin
  rclcpp
  gazebo_ros
  nav_msgs
  std_msgs
  px4_msgs
  sensor_msgs
  geometry_msgs)

target_link_libraries(platformPlugin ${GAZEBO_LIBRARIES})

install(TARGETS platformPlugin
  LIBRARY DESTINATION lib)

ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/landingPlat.dsv.in")
ament_package()
