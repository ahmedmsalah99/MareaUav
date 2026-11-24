cmake_minimum_required(VERSION 3.5)
project(mocap_boundaries_check)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(rclcpp REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(ros2_muavp_interface REQUIRED)
find_package(Eigen3 REQUIRED)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

include_directories(include)
INCLUDE_DIRECTORIES(${EIGEN3_INCLUDE_DIR})

add_library(MoCapBoundariesCheck SHARED src/mocap_boundaries_check_handler.cpp)
target_link_libraries(MoCapBoundariesCheck ${rclcpp_LIBRARIES})

ament_target_dependencies(MoCapBoundariesCheck rclcpp rclcpp_components px4_msgs ros2_muavp_interface)

rclcpp_components_register_nodes(MoCapBoundariesCheck "MoCapBoundariesCheck")

add_executable(mocap_boundaries_check_main src/mocap_boundaries_check_main.cpp)
target_include_directories(mocap_boundaries_check_main PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)


target_link_libraries(mocap_boundaries_check_main MoCapBoundariesCheck)
  
# Standalone nodes need their linked shared libraries in the installed workspace.
ament_export_libraries(MoCapBoundariesCheck)
install(TARGETS MoCapBoundariesCheck
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

ament_export_dependencies(rclcpp)

install(TARGETS mocap_boundaries_check_main
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
