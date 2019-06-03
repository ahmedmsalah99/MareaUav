#####################################
cmake_minimum_required (VERSION 3.5) 
project(ros2_raspicam)

set(CMAKE_CXX_STANDARD 14)


#set(CMAKE_CXX_FLAGS "-Wall -ffunction-sections  -fomit-frame-pointer -O2 -ffast-math -DNDEBUG -mcpu=arm1176jzf-s  -mfpu=vfp -mfloat-abi=hard -ftree-vectorize")

set(CMAKE_CXX_FLAGS "-Wall -ffunction-sections  -fomit-frame-pointer -O2 -ffast-math -DNDEBUG -mfpu=vfp -mfloat-abi=hard -ftree-vectorize")

set(CMAKE_MODULE_PATH "/usr/local/lib/cmake/;${CMAKE_MODULE_PATH}")

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rmw_implementation REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(raspicam REQUIRED)
find_package(class_loader REQUIRED)

include_directories(
${rclcpp_INCLUDE_DIRS}
${rmw_implementation_INCLUDE_DIRS}
${std_msgs_INCLUDE_DIRS}
${geometry_msgs_INCLUDE_DIRS}
)

add_executable(raspicam_raw_node src/raspicam_raw.cpp)  

add_executable(raspicam_compressed_node src/raspicam_compressed.cpp)  

add_executable(checks_rate_node src/checks_rate.cpp) 

add_executable(trial_node src/not_composable.cpp) 


ament_target_dependencies(raspicam_raw_node rclcpp std_msgs sensor_msgs class_loader)

ament_target_dependencies(raspicam_compressed_node rclcpp std_msgs sensor_msgs class_loader)

ament_target_dependencies(checks_rate_node rclcpp std_msgs sensor_msgs class_loader)

ament_target_dependencies(trial_node rclcpp std_msgs sensor_msgs class_loader)


target_link_libraries(raspicam_raw_node
        ${rclcpp_LIBRARIES}
        ${rmw_implementation_LIBRARIES}
        ${std_msgs_LIBRARIES}
        ${PROJECT_LINK_LIBS}
        ${sensor_msgs_LIBRARIES}
        ${class_loader_LIBRARIES}
        ${raspicam_LIBS}
        jpeg
        )

target_link_libraries(raspicam_compressed_node
        ${rclcpp_LIBRARIES}
        ${rmw_implementation_LIBRARIES}
        ${std_msgs_LIBRARIES}
        ${PROJECT_LINK_LIBS}
        ${sensor_msgs_LIBRARIES}
        ${class_loader_LIBRARIES}
        ${raspicam_LIBS}
        )

target_link_libraries(checks_rate_node
        ${rclcpp_LIBRARIES}
        ${rmw_implementation_LIBRARIES}
        ${std_msgs_LIBRARIES}
        ${PROJECT_LINK_LIBS}
        ${sensor_msgs_LIBRARIES}
        ${class_loader_LIBRARIES}
        ${raspicam_LIBS}
        )

target_link_libraries(trial_node
        ${rclcpp_LIBRARIES}
        ${rmw_implementation_LIBRARIES}
        ${std_msgs_LIBRARIES}
        ${PROJECT_LINK_LIBS}
        ${sensor_msgs_LIBRARIES}
        ${class_loader_LIBRARIES}
        ${raspicam_LIBS}
        )

install(TARGETS raspicam_raw_node DESTINATION lib/${PROJECT_NAME})

install(TARGETS raspicam_compressed_node DESTINATION lib/${PROJECT_NAME})

install(TARGETS checks_rate_node DESTINATION lib/${PROJECT_NAME})

install(TARGETS trial_node DESTINATION lib/${PROJECT_NAME})

ament_package()
