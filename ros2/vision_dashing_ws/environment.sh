echo "setting up workspace..."
source /opt/ros/humble/setup.bash
source ./install/setup.bash
#for opencv-3.1 with pkg_config
PKG_CONFIG_PATH=$PKG_CONFIG_PATH:/usr/local/lib/pkgconfig
export PKG_CONFIG_PATH

export TURTLEBOT3_MODEL=waffle

echo "done!!!"
