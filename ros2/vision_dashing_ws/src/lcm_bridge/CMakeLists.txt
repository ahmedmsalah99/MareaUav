cmake_minimum_required(VERSION 3.5)
project(lcm_bridge)

#add_definitions(-std=c++11)

set(CMAKE_CXX_STANDARD 14)
set(apriltag_msgs_DIR "/home/andrea/Marea/mar_frankenrotor_ros/ros2SITL/vision_dashing_ws/install/apriltag_msgs/share/apriltag_msgs/cmake")

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
find_package(apriltag_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
## find_package(mavros_msgs REQUIRED)
find_package(class_loader REQUIRED)
###########
## Build ##
###########

#Set LCM
find_package(PkgConfig REQUIRED)
pkg_search_module(LCM REQUIRED lcm)


set(M2M_INCLUDE "$ENV{M2M_INCLUDE}")
if(M2M_INCLUDE)            
 include_directories(${M2M_INCLUDE})
else()
 message(STATUS "INCLUDE at ${M2M_INCLUDE}")
 message(FATAL_ERROR "Undefined M2M_INCLUDE env var. HINT: export M2M_INCLUDE=/path/to/mocap2mav/include")
endif()

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
${std_msgs_INCLUDE_DIRS}
${geometry_msgs_INCLUDE_DIRS}
${apriltag_msgs_INCLUDE_DIRS}
${Eigen_INCLUDE_DIR}
${px4_msgs_LIBRARIES}
${sensor_msgs_LIBRARIES}
## ${mavros_msgs_INCLUDE_DIR}
)

## Declare a C++ executable
add_executable(bridge_node src/bridge_node.cpp ${M2M_INCLUDE}/../src/MavState.cpp)

ament_target_dependencies(bridge_node rclcpp std_msgs sensor_msgs px4_msgs geometry_msgs apriltag_msgs class_loader)


## Specify libraries to link a library or executable target against

target_link_libraries(bridge_node
        ${rclcpp_LIBRARIES}
        ${rmw_implementation_LIBRARIES}
        ${std_msgs_LIBRARIES}
        ${PROJECT_LINK_LIBS}
        ${LCM_LIBRARIES}
        ${geometry_msgs_LIBRARIES}
        ${apriltag_msgs_LIBRARIES}
        ${px4_msgs_LIBRARIES}
	${sensor_msgs_LIBRARIES}
        ## ${mavros_msgs_LIBRARIES}
	)

install(TARGETS bridge_node  DESTINATION lib/${PROJECT_NAME})
ament_package()
