cmake_minimum_required(VERSION 3.8)
project(safety_handler CXX)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Local source dir (same as the original)
set(PROJECT_SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/src)

# Shared common code lives outside this package, at the workspace root.
# Adjust the relative path if your workspace layout differs.
set(PROJECT_SRC_DIR     ${CMAKE_CURRENT_SOURCE_DIR}/../common/src)
set(PROJECT_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../common/include)

set(CMAKE_AUTOMOC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(M2M_SIM "$ENV{M2M_SIM}")

# -------- Dependencies --------
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(ulisse_exec_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

# -------- Library (Safetylib) --------
set(PROJECT_SRCS
  ${PROJECT_SOURCE_DIR}/SafetyNode.cpp
  ${PROJECT_SOURCE_DIR}/SafetyParser.cpp
)

add_library(Safetylib
  libs/SafetyHandler.cpp
  ${PROJECT_SRC_DIR}/MavState.cpp
)

target_include_directories(Safetylib PUBLIC
  ${PROJECT_INCLUDE_DIR}
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${CMAKE_CURRENT_SOURCE_DIR}/libs
  ${EIGEN3_INCLUDE_DIR}
)

ament_target_dependencies(Safetylib
  rclcpp
  std_msgs
  sensor_msgs
  nav_msgs
  geometry_msgs
  px4_msgs
  ulisse_exec_msgs
)

# -------- Executable --------
add_executable(safety_handler_node ${PROJECT_SRCS})

target_link_libraries(safety_handler_node Safetylib)

ament_target_dependencies(safety_handler_node
  rclcpp
  std_msgs
  sensor_msgs
  nav_msgs
  geometry_msgs
  px4_msgs
  ulisse_exec_msgs
)

# -------- Install --------
install(TARGETS safety_handler_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
