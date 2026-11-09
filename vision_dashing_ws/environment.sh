echo "setting up workspace..."
source /opt/ros/humble/setup.bash
source ./install/setup.bash
#for opencv-3.1 with pkg_config
export PKG_CONFIG_PATH=$PKG_CONFIG_PATH:/usr/local/lib/pkgconfig
# 0 = sim, 1 = mocap, 2 = hw, 3 = mocap in sim (Gazebo flown like the mocap lab)
export M2M_MODE=0
export TURTLEBOT3_MODEL=waffle

echo "done!!!"
