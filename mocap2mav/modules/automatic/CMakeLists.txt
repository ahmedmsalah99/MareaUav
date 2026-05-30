cmake_minimum_required(VERSION 3.8)
project(automatic CXX)

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
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(ulisse_exec_msgs REQUIRED)
find_package(ultrasonic_sensor REQUIRED)
find_package(Eigen3 REQUIRED)
pkg_search_module(LCM REQUIRED lcm)

# Header globbing kept from the original (Command/*.hpp).
file(GLOB_RECURSE COMM include/Command/*.hpp)

# -------- Library (Automlib) --------
set(PROJECT_SRCS
  ${PROJECT_SOURCE_DIR}/AutomaticNode.cpp
)

add_library(Automlib
  libs/Automatic.cpp
  libs/Lander.cpp
  libs/Searcher.cpp
  libs/Parameters.cpp
  libs/StateClasses.cpp
  libs/SearchStateClasses.cpp
  ${PROJECT_SRC_DIR}/MavState.cpp
)

target_include_directories(Automlib PUBLIC
  ${PROJECT_INCLUDE_DIR}
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${CMAKE_CURRENT_SOURCE_DIR}/libs
  ${EIGEN3_INCLUDE_DIR}
)

ament_target_dependencies(Automlib
  rclcpp
  std_msgs
  sensor_msgs
  nav_msgs
  geometry_msgs
  px4_msgs
  ulisse_exec_msgs
  ultrasonic_sensor
)

# External libs from the original CMakeLists (StateMachine + miniPID).
# Same as before — they have to be findable on the link path.
target_link_libraries(Automlib StateMachine miniPID)

# -------- Executable --------
add_executable(automatic_node ${PROJECT_SRCS} ${COMM})

target_link_libraries(automatic_node Automlib ${LCM_LIBRARIES})

ament_target_dependencies(automatic_node
  rclcpp
  std_msgs
  sensor_msgs
  nav_msgs
  geometry_msgs
  px4_msgs
  ulisse_exec_msgs
  ultrasonic_sensor
)

# -------- Install --------
install(TARGETS automatic_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
