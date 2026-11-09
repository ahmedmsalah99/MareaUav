cmake_minimum_required(VERSION 3.8)
project(mocap_bridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(ulisse_msgs REQUIRED)

add_executable(mocap_bridge_node src/mocap_bridge_node.cpp)
target_compile_features(mocap_bridge_node PUBLIC cxx_std_17)
ament_target_dependencies(mocap_bridge_node rclcpp geometry_msgs sensor_msgs px4_msgs ulisse_msgs)

install(TARGETS mocap_bridge_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

ament_package()
