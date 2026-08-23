cmake_minimum_required(VERSION 3.5)
project(spa_predictor)

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

# Core library (FFT identifier + SPA observer)
add_library(spa_predictor_lib
    src/fft_identifier.cpp
    src/spa_observer.cpp
)
target_include_directories(spa_predictor_lib PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(spa_predictor_lib Eigen3)
target_link_libraries(spa_predictor_lib Eigen3::Eigen)

# ROS2 node
add_executable(spa_predictor_node src/spa_predictor_node.cpp)
target_link_libraries(spa_predictor_node spa_predictor_lib)
ament_target_dependencies(spa_predictor_node
    rclcpp
    std_msgs
    ulisse_msgs
    geometry_msgs
    Eigen3
)

#############
## Install ##
#############

install(TARGETS spa_predictor_lib DESTINATION lib)
install(TARGETS spa_predictor_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)

ament_package()
