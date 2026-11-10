cmake_minimum_required(VERSION 3.8)
project(px4_global_origin)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(px4_msgs REQUIRED)

add_executable(px4_global_origin_node src/px4_global_origin_node.cpp)
target_compile_features(px4_global_origin_node PUBLIC cxx_std_17)
ament_target_dependencies(px4_global_origin_node rclcpp px4_msgs)

install(TARGETS px4_global_origin_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

ament_package()
