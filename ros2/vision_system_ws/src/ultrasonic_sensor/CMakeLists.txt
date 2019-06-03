cmake_minimum_required(VERSION 3.5)
project(ultrasonic_sensor)


set(CMAKE_CXX_STANDARD 14)

#add_definitions(-std=c++11)
## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)


find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rmw_implementation REQUIRED)
find_package(std_msgs REQUIRED)
#find_package(cmake_modules REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

find_package(PkgConfig REQUIRED)
pkg_search_module(LCM REQUIRED lcm)

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
)


set(M2M_INCLUDE "$ENV{M2M_INCLUDE}")
if(M2M_INCLUDE)            
 include_directories(${M2M_INCLUDE})
else()
 message(STATUS "INCLUDE at ${M2M_INCLUDE}")
 message(FATAL_ERROR "Undefined M2M_INCLUDE env var. HINT: export M2M_INCLUDE=/path/to/mocap2mav/include")
endif()

include_directories(include)

set(SOURCE FILES source/UltrasonicSensor.cpp include/UltrasonicSensor.h)

add_library(UltrasonicSensor source/UltrasonicSensor.cpp include/UltrasonicSensor.h)

find_library(wiringPi_LIB wiringPi)

target_link_libraries(UltrasonicSensor ${wiringPi_LIB} 
				       ${LCM_LIBRARIES})


## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide

add_executable(${PROJECT_NAME}_node_Mocap2mav src/mainMocap2mav.cpp)

target_link_libraries(${PROJECT_NAME}_node_Mocap2mav UltrasonicSensor)

rosidl_target_interfaces(${PROJECT_NAME}_node_Mocap2mav ${PROJECT_NAME} "rosidl_typesupport_cpp" )

ament_target_dependencies(${PROJECT_NAME}_node_Mocap2mav rclcpp std_msgs geometry_msgs)

ament_package()
