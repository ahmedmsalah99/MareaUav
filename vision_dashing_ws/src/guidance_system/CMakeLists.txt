cmake_minimum_required(VERSION 3.8)
project(guidance_system)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(ulisse_msgs REQUIRED)
find_package(Eigen3 REQUIRED)# uncomment the following section in order to fill in
find_package(ament_index_cpp REQUIRED)
find_package(ultrasonic_sensor REQUIRED)
# further dependencies manually.
# find_package(<dependency> REQUIRED)
pkg_search_module(LCM REQUIRED lcm)

# Local source dir (same as the original)
set(PROJECT_SRC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/src)

# Shared common code lives outside this package, at the workspace root.
# Adjust the relative path if your workspace layout differs.
set(COMMON_SRC_DIR     ${CMAKE_CURRENT_SOURCE_DIR}/common/src)
set(COMMON_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/common/include)

set(CMAKE_AUTOMOC ON)
set(M2M_SIM "$ENV{M2M_SIM}")


add_subdirectory(libs/StateMachine)
add_subdirectory(libs/miniPID)

# Libs
add_library(Safetylib
  libs/SafetyHandler.cpp
  ${COMMON_SRC_DIR}/MavState.cpp
)
add_library(Execlib
  libs/Executioner.cpp
  ${COMMON_SRC_DIR}/MavState.cpp
)
add_library(Automlib
  libs/autom/Automatic.cpp
  libs/autom/Lander.cpp
  libs/autom/Searcher.cpp
  libs/autom/Parameters.cpp
  libs/autom/StateClasses.cpp
  libs/autom/SearchStateClasses.cpp
  ${COMMON_SRC_DIR}/MavState.cpp
)

target_include_directories(Safetylib PUBLIC
  ${COMMON_INCLUDE_DIR}
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${CMAKE_CURRENT_SOURCE_DIR}/libs
  ${EIGEN3_INCLUDE_DIR}
)
target_include_directories(Execlib PUBLIC
  ${COMMON_INCLUDE_DIR}
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${CMAKE_CURRENT_SOURCE_DIR}/libs
  ${EIGEN3_INCLUDE_DIR}
)
target_include_directories(Automlib PUBLIC
  ${COMMON_INCLUDE_DIR}
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
  ulisse_msgs
)
ament_target_dependencies(Execlib
  rclcpp
  std_msgs
  nav_msgs
  px4_msgs
  ulisse_msgs
)
ament_target_dependencies(Automlib
  rclcpp
  std_msgs
  sensor_msgs
  nav_msgs
  geometry_msgs
  px4_msgs
  ultrasonic_sensor
  ulisse_msgs
)
target_link_libraries(Automlib StateMachine miniPID)

set(SAFETY_SRCS
  ${PROJECT_SRC_DIR}/SafetyNode.cpp
  ${PROJECT_SRC_DIR}/SafetyParser.cpp
)
set(EXEC_SRCS
  ${PROJECT_SRC_DIR}/ExecNode.cpp
  ${PROJECT_SRC_DIR}/Parser.cpp
)
set(AUTOM_SRCS
  ${PROJECT_SRC_DIR}/AutomaticNode.cpp
)
# nodes

add_executable(safety_handler_node ${SAFETY_SRCS})
add_executable(executioner_node ${EXEC_SRCS})
add_executable(autom_node ${AUTOM_SRCS})

target_link_libraries(safety_handler_node Safetylib)
target_link_libraries(executioner_node Execlib)
target_link_libraries(autom_node Automlib ${LCM_LIBRARIES})

ament_target_dependencies(safety_handler_node
  rclcpp
  std_msgs
  sensor_msgs
  nav_msgs
  geometry_msgs
  px4_msgs
  ulisse_msgs
  ament_index_cpp
)
ament_target_dependencies(executioner_node
  rclcpp
  std_msgs
  nav_msgs
  px4_msgs
  ulisse_msgs
  ament_index_cpp
)
ament_target_dependencies(autom_node
  rclcpp
  std_msgs
  sensor_msgs
  nav_msgs
  geometry_msgs
  px4_msgs
  ulisse_msgs
  ultrasonic_sensor
  ament_index_cpp
)
# -------- Install --------

install(TARGETS Safetylib DESTINATION lib/${PROJECT_NAME})
install(TARGETS Execlib DESTINATION lib/${PROJECT_NAME})
install(TARGETS Automlib DESTINATION lib/${PROJECT_NAME})


install(TARGETS safety_handler_node
  DESTINATION lib/${PROJECT_NAME}
)
install(TARGETS executioner_node
  DESTINATION lib/${PROJECT_NAME}
)
install(TARGETS autom_node
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY
  common/lists
  common/params
  DESTINATION share/${PROJECT_NAME}
)
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
