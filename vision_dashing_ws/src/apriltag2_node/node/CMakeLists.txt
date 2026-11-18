cmake_minimum_required(VERSION 3.5)

project(apriltag2_node)

set(CMAKE_CXX_STANDARD 14)

find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(apriltag_msgs REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(class_loader REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core calib3d)

INCLUDE_DIRECTORIES(${EIGEN3_INCLUDE_DIR})

find_package(Threads REQUIRED)

find_package(PkgConfig REQUIRED)
pkg_search_module(apriltag REQUIRED apriltag)



set(M2M_MODE "$ENV{M2M_MODE}")

include_directories(${apriltag_INCLUDE_DIRS})
# The code includes the library's headers flat (<apriltag.h>, <tag36h11.h>,
# <common/pjpeg.h>). The AprilRobotics install puts them under
# <prefix>/include/apriltag/, while its pkg-config only lists <prefix>/include,
# so add that subfolder too.
foreach(_dir ${apriltag_INCLUDE_DIRS} ${apriltag_INCLUDEDIR})
  if(EXISTS "${_dir}/apriltag/apriltag.h")
    include_directories("${_dir}/apriltag")
  endif()
endforeach()
link_directories(${apriltag_LIBDIR})

include_directories(include)

add_library(AprilTag2Node SHARED src/AprilTag2Node2_0.cpp)
ament_target_dependencies(AprilTag2Node rclcpp sensor_msgs rclcpp_components apriltag_msgs px4_msgs class_loader tf2_msgs)
target_link_libraries(AprilTag2Node ${apriltag_LIBRARIES} opencv_calib3d)

message( "INCLUDE_DIRS=${apriltag_INCLUDE_DIRS}")
message( "INCLUDEDIR=${apriltag_INCLUDEDIR}")
message( "CFLAGS=${apriltag_CFLAGS}")
message( "LIBRARIES=${apriltag_LIBRARIES}")
message( "PKG_CONFIG_PATH=$ENV{PKG_CONFIG_PATH}")
target_include_directories(AprilTag2Node PRIVATE ${apriltag_INCLUDE_DIRS})

rclcpp_components_register_nodes(AprilTag2Node "AprilTag2Node")

add_executable(apriltag2_node src/apriltag2_node.cpp)
target_link_libraries(apriltag2_node AprilTag2Node)

ament_environment_hooks(${ament_cmake_package_templates_ENVIRONMENT_HOOK_LIBRARY_PATH})

install(TARGETS
    AprilTag2Node
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

install(TARGETS apriltag2_node DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY cfg/ DESTINATION share/${PROJECT_NAME}/cfg)

ament_package()
