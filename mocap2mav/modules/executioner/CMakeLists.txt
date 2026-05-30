cmake_minimum_required(VERSION 3.8)
project(executioner CXX)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Local source dir
set(PROJECT_SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/src)

# Shared common code outside this package — adjust path if needed.
set(PROJECT_SRC_DIR     ${CMAKE_CURRENT_SOURCE_DIR}/../common/src)
set(PROJECT_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../common/include)

set(CMAKE_AUTOMOC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(M2M_SIM "$ENV{M2M_SIM}")

# -------- Dependencies --------
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(ulisse_msgs REQUIRED)
find_package(ulisse_exec_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

# -------- Library (Execlib) --------
set(PROJECT_SRCS
  ${PROJECT_SOURCE_DIR}/ExecNode.cpp
  ${PROJECT_SOURCE_DIR}/Parser.cpp
)

add_library(Execlib
  libs/Executioner.cpp
  ${PROJECT_SRC_DIR}/MavState.cpp
)

target_include_directories(Execlib PUBLIC
  ${PROJECT_INCLUDE_DIR}
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${CMAKE_CURRENT_SOURCE_DIR}/libs
  ${EIGEN3_INCLUDE_DIR}
)

ament_target_dependencies(Execlib
  rclcpp
  std_msgs
  nav_msgs
  px4_msgs
  ulisse_msgs
  ulisse_exec_msgs
)

# -------- Executable --------
add_executable(executioner_node ${PROJECT_SRCS})

target_link_libraries(executioner_node Execlib)

ament_target_dependencies(executioner_node
  rclcpp
  std_msgs
  nav_msgs
  px4_msgs
  ulisse_msgs
  ulisse_exec_msgs
)

# -------- Install --------
install(TARGETS executioner_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
