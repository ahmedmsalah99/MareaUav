cmake_minimum_required(VERSION 3.5)
project(examples_rclcpp_minimal_subscriber)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

#if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
#  add_compile_options(-Wall -Wextra -Wpedantic)
#endif()

#set(CMAKE_CXX_FLAGS "-Wall -ffunction-sections  -fomit-frame-pointer -O2 -ffast-math -DNDEBUG -mcpu=arm1176jzf-s  -mfpu=vfp -mfloat-abi=hard -ftree-vectorize")

set(CMAKE_CXX_FLAGS "-Wall -ffunction-sections  -fomit-frame-pointer -O2 -ffast-math -DNDEBUG -mfpu=vfp -mfloat-abi=hard -ftree-vectorize")

set(CMAKE_MODULE_PATH "/usr/local/lib/cmake/;${CMAKE_MODULE_PATH}")

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rmw_implementation REQUIRED)
find_package(class_loader REQUIRED)

include_directories(
${rclcpp_INCLUDE_DIRS}
${rmw_implementation_INCLUDE_DIRS}
${std_msgs_INCLUDE_DIRS}
${geometry_msgs_INCLUDE_DIRS}
)


add_executable(subscriber_lambda lambda.cpp)
ament_target_dependencies(subscriber_lambda rclcpp std_msgs sensor_msgs class_loader)

add_executable(subscriber_member_function member_function.cpp)
ament_target_dependencies(subscriber_member_function rclcpp std_msgs sensor_msgs class_loader)

add_executable(subscriber_not_composable not_composable.cpp)
ament_target_dependencies(subscriber_not_composable rclcpp std_msgs sensor_msgs class_loader)

add_executable(mine_node mine.cpp)
ament_target_dependencies(mine_node rclcpp std_msgs sensor_msgs class_loader)

install(TARGETS
  mine_node
  subscriber_lambda
  subscriber_member_function
  subscriber_not_composable
  DESTINATION lib/${PROJECT_NAME})

ament_package()
