echo "setting up workspace..."
source /opt/ros/humble/setup.bash
source ./install/setup.bash
#for opencv-3.1 with pkg_config
export PKG_CONFIG_PATH=$PKG_CONFIG_PATH:/usr/local/lib/pkgconfig
export M2M_SIM=1
export TURTLEBOT3_MODEL=waffle

echo "done!!!"
