#####################################
cmake_minimum_required (VERSION 3.5) 
project(fsm_raspicam)

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

include_directories(include)

add_executable(camera_main_node src/camera_main.cpp src/FiniteStateMachine.cpp src/State.cpp src/CameraStates.cpp )  

ament_target_dependencies(camera_main_node rclcpp std_msgs sensor_msgs class_loader)

target_link_libraries(camera_main_node
        ${rclcpp_LIBRARIES}
        ${rmw_implementation_LIBRARIES}
        ${std_msgs_LIBRARIES}
        ${PROJECT_LINK_LIBS}
        ${sensor_msgs_LIBRARIES}
        ${class_loader_LIBRARIES}
        ${raspicam_LIBS}
        jpeg
        )

install(TARGETS camera_main_node DESTINATION lib/${PROJECT_NAME})

ament_package()
