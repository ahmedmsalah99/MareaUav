#!/bin/bash

echo "setting gazebo stuff, put correct path of Firmware directory"
source /home/andrea/Desktop/Marea/VisionSystem/ros/Firmware/Tools/setup_gazebo.bash /home/andrea/Desktop/Marea/VisionSystem/ros/Firmware /home/andrea/Desktop/Marea/VisionSystem/ros/Firmware/build/px4_sitl_default

echo "link mocap2mav lcm"
export M2M_INCLUDE=/home/andrea/Desktop/Marea/VisionSystem/ros/mocap2mav/include

echo "load gazebo plugin for the platform, put the correct path of the landingPlat directory "

export GAZEBO_PLUGIN_PATH=${GAZEBO_PLUGIN_PATH}:/home/andrea/Desktop/Marea/VisionSystem/ros/landingPlat/build

echo "done!!:)"
