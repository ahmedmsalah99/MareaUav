cmake_minimum_required(VERSION 3.5)
project(px4_bridge)

#add_definitions(-std=c++11)

set(CMAKE_CXX_STANDARD 14)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
if(DEFINED ENV{ROS2_DISTRO})
  set(ROS_DISTRO $ENV{ROS2_DISTRO})
  if(${ROS_DISTRO} IN_LIST ROS_DISTROS)
    add_definitions(-DROS_DEFAULT_API=1)
  endif()
elseif(DEFINED ENV{ROS_DISTRO})
  set(ROS_DISTRO $ENV{ROS_DISTRO})
  if(${ROS_DISTRO} IN_LIST ROS_DISTROS)
    add_definitions(-DROS_DEFAULT_API=1)
  endif()
else()
  message(FATAL_ERROR "No ROS Distro set")
endif()
#find_package(rosidl_default_generators REQUIRED)#maybe not necessary

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rmw_implementation REQUIRED)
find_package(std_msgs REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(ulisse_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
###########
## Build ##
###########



## Specify additional locations of header files
## Your package locations should be listed before other locations

find_package(Eigen 3 QUIET)
if(NOT Eigen_FOUND)
 include(cmake/FindEigen3.cmake)
 set(Eigen_INCLUDE_DIR ${EIGEN3_INCLUDE_DIR})
endif()

include_directories(
${rclcpp_INCLUDE_DIRS}
${rmw_implementation_INCLUDE_DIRS}
${Eigen_INCLUDE_DIR}
${CMAKE_CURRENT_SOURCE_DIR}/../common/include
)

## Declare a C++ executable
add_executable(bridge_node src/bridge_node.cpp)

ament_target_dependencies(bridge_node rclcpp std_msgs px4_msgs ulisse_msgs nav_msgs)


## Specify libraries to link a library or executable target against

target_link_libraries(bridge_node
        ${rclcpp_LIBRARIES}
        ${rmw_implementation_LIBRARIES}
        ${PROJECT_LINK_LIBS}
	)

install(TARGETS bridge_node  DESTINATION lib/${PROJECT_NAME})
ament_package()
