#!/bin/bash

echo "link mocap2mav lcm"
export M2M_INCLUDE=/home/andrea/Marea/mar_frankenrotor_ros/ros2SITL/mocap2mav/include

echo "exporting simulation variable: 1=sim, 0=no-sim"
export M2M_SIM=0

echo "done!!:)"
