cmake_minimum_required(VERSION 3.5)
project(Payload)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(ament_cmake REQUIRED)
find_package(gazebo_dev REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(gazebo REQUIRED)

include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

add_library(payloadPlugin SHARED src/plugin.cpp)

ament_target_dependencies(payloadPlugin
  rclcpp
  gazebo_ros
  std_msgs
  px4_msgs)

target_link_libraries(payloadPlugin ${GAZEBO_LIBRARIES})

install(TARGETS payloadPlugin
  LIBRARY DESTINATION lib)

ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/Payload.dsv.in")
ament_package()
