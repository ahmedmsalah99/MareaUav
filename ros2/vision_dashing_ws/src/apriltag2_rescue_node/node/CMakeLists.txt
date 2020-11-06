cmake_minimum_required(VERSION 3.5)

project(apriltag2_rescue_node)

set(CMAKE_CXX_STANDARD 14)

find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(apriltag_msgs REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(class_loader REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(Eigen3 REQUIRED)
INCLUDE_DIRECTORIES(${EIGEN3_INCLUDE_DIR})

find_package(Threads REQUIRED)

find_package(PkgConfig REQUIRED)
pkg_search_module(apriltag REQUIRED apriltag)
pkg_search_module(LCM REQUIRED lcm)

set(M2M_INCLUDE "$ENV{M2M_INCLUDE}")
if(M2M_INCLUDE)            
 include_directories(${M2M_INCLUDE})
else()
 message(STATUS "INCLUDE at ${M2M_INCLUDE}")
 message(FATAL_ERROR "Undefined M2M_INCLUDE env var. HINT: export M2M_INCLUDE=/path/to/mocap2mav/include")
endif()

set(M2M_SIM "$ENV{M2M_SIM}")

include_directories(${apriltag_INCLUDE_DIRS})
link_directories(${apriltag_LIBDIR})

include_directories(include)

add_library(AprilTag2RescueNode SHARED src/AprilTag2RescueNode2.cpp)
ament_target_dependencies(AprilTag2RescueNode rclcpp sensor_msgs px4_msgs rclcpp_components apriltag_msgs class_loader tf2_msgs)
target_link_libraries(AprilTag2RescueNode ${apriltag_LIBRARIES} ${LCM_LIBRARIES})
rclcpp_components_register_nodes(AprilTag2RescueNode "AprilTag2RescueNode")

add_executable(apriltag2_rescue_node src/apriltag2_rescue_node.cpp)
target_link_libraries(apriltag2_rescue_node AprilTag2RescueNode)

ament_environment_hooks(${ament_cmake_package_templates_ENVIRONMENT_HOOK_LIBRARY_PATH})

install(TARGETS
    AprilTag2RescueNode
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

install(TARGETS apriltag2_rescue_node DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY cfg/ DESTINATION share/${PROJECT_NAME}/cfg)

ament_package()
