cmake_minimum_required(VERSION 3.5)
project(heave_estimator)

set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(ulisse_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)

###########
## Build ##
###########

include_directories(include)

# Core estimation library (FFT identifier + EKF)
add_library(heave_estimator_lib
    src/fft_identifier.cpp
    src/heave_ekf.cpp
)
target_include_directories(heave_estimator_lib PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(heave_estimator_lib Eigen3)
target_link_libraries(heave_estimator_lib Eigen3::Eigen)

# ROS2 node
add_executable(heave_estimator_node src/heave_estimator_node.cpp)
target_link_libraries(heave_estimator_node heave_estimator_lib)
ament_target_dependencies(heave_estimator_node
    rclcpp
    std_msgs
    ulisse_msgs
    geometry_msgs
    Eigen3
)

#############
## Install ##
#############

install(TARGETS heave_estimator_lib DESTINATION lib)
install(TARGETS heave_estimator_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)

ament_package()
