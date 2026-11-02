cmake_minimum_required(VERSION 3.5)
project(ultrasonic_sensor)


set(CMAKE_CXX_STANDARD 17) # Change 14 to 17
#add_definitions(-std=c++11)
## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)


find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rmw_implementation REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
#find_package(cmake_modules REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

find_package(PkgConfig REQUIRED)
find_package(rcl_interfaces REQUIRED)

################################################
## Declare ROS messages, services and actions ##
################################################

set(msg_files "msg/UltrasonicMsg.msg")

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  DEPENDENCIES builtin_interfaces
)

ament_export_dependencies(rosidl_default_runtime)

###########
## Build ##
###########

include_directories(
${rclcpp_INCLUDE_DIRS}
${rmw_implementation_INCLUDE_DIRS}
${std_msgs_INCLUDE_DIRS}
${geometry_msgs_INCLUDE_DIRS}
${sensor_msgs_INCLUDE_DIRS}
)


# TimeManager (utils/TimeHelpers.hpp) is now vendored under include/, so this
# package no longer needs M2M_INCLUDE (mocap2mav/include) or LCM.
include_directories(include)

# M2M_MODE: 0 = sim, 1 = mocap, 2 = hw. Mocap and hw both use the real sensor.
set(M2M_MODE "$ENV{M2M_MODE}")
if("${M2M_MODE}" STREQUAL "0")
 set(SOURCE FILES source/UltrasonicSensorSim.cpp include/UltrasonicSensor.h)
 add_library(UltrasonicSensor source/UltrasonicSensorSim.cpp include/UltrasonicSensor.h)
elseif("${M2M_MODE}" STREQUAL "1" OR "${M2M_MODE}" STREQUAL "2")
 set(SOURCE FILES source/UltrasonicSensorReal.cpp include/UltrasonicSensor.h)
 add_library(UltrasonicSensor source/UltrasonicSensorReal.cpp include/UltrasonicSensor.h)
 find_library(wiringPi_LIB wiringPi)
 target_link_libraries(UltrasonicSensor ${wiringPi_LIB})
else()
 message(FATAL_ERROR "M2M_MODE must be set to 0 (sim), 1 (mocap) or 2 (hw); got '${M2M_MODE}'")
endif()



ament_target_dependencies(UltrasonicSensor rclcpp std_msgs sensor_msgs geometry_msgs rcl_interfaces)

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide

add_executable(${PROJECT_NAME}_node_Mocap2mav src/mainMocap2mav.cpp)
target_link_libraries(${PROJECT_NAME}_node_Mocap2mav UltrasonicSensor)
ament_target_dependencies(${PROJECT_NAME}_node_Mocap2mav 
    rclcpp 
    std_msgs 
    sensor_msgs 
    geometry_msgs 
    rcl_interfaces
)
# rosidl_target_interfaces(${PROJECT_NAME}_node_Mocap2mav ${PROJECT_NAME} "rosidl_typesupport_cpp" )

# 1. Get the specific typesupport target for your message package
rosidl_get_typesupport_target(typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")

# 2. Link that target directly to your node
target_link_libraries(${PROJECT_NAME}_node_Mocap2mav "${typesupport_target}")

ament_target_dependencies(${PROJECT_NAME}_node_Mocap2mav rclcpp std_msgs sensor_msgs geometry_msgs rcl_interfaces)

install(TARGETS ${PROJECT_NAME}_node_Mocap2mav DESTINATION lib/${PROJECT_NAME})
install(
  TARGETS UltrasonicSensor
  DESTINATION lib
)
ament_package()