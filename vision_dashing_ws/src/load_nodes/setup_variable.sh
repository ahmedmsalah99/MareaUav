#!/bin/bash

echo "setting gazebo stuff, put correct path of Firmware directory"
source /home/andrea/Marea/mar_frankenrotor_ros/ros2SITL/Firmware/Tools/setup_gazebo.bash /home/andrea/Marea/mar_frankenrotor_ros/ros2SITL/Firmware /home/andrea/Marea/mar_frankenrotor_ros/ros2SITL/Firmware/build/px4_sitl_rtps

echo "link mocap2mav lcm"
export M2M_INCLUDE=/home/andrea/Marea/mar_frankenrotor_ros/ros2SITL/mocap2mav/include

echo "exporting run mode: 0=sim, 1=mocap, 2=hw, 3=mocap in sim"
export M2M_MODE=0

echo "load gazebo plugins (landingPlat / Payload)"
# The plugins are now ament packages that talk ROS 2 directly. After building
# them with colcon they install their .so into <install>/<pkg>/lib, and each
# ships an env-hook that appends that dir to GAZEBO_PLUGIN_PATH when you source
# the workspace install/setup.bash. If you don't source that setup, point
# GAZEBO_PLUGIN_PATH at the install lib dirs explicitly (fix the root to yours):

export GAZEBO_PLUGIN_PATH=${GAZEBO_PLUGIN_PATH}:/home/andrea/Marea/mar_frankenrotor_ros/ros2SITL/install/landingPlat/lib
export GAZEBO_PLUGIN_PATH=${GAZEBO_PLUGIN_PATH}:/home/andrea/Marea/mar_frankenrotor_ros/ros2SITL/install/Payload/lib

echo "done!!:)"
