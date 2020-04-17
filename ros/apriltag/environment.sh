echo "setting up apriltags_ws..."
source /opt/ros/melodic/setup.bash
source /home/andrea/Desktop/Marea/VisionSystem/ros/apriltag/catkin_ws/devel/setup.bash
#for opencv-3.1 with pkg_config
PKG_CONFIG_PATH=$PKG_CONFIG_PATH:/usr/local/lib/pkgconfig
export PKG_CONFIG_PATH

export TURTLEBOT3_MODEL=waffle

echo "done!!!"
