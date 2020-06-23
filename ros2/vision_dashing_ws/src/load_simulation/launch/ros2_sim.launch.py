import launch
import launch.actions
import launch.events
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument
from launch.substitutions import EnvironmentVariable
import launch_ros.actions
import os
 
def generate_launch_description():
    parameters_file_path = "/home/andrea/Marea/mar_frankenrotor_ros/ros2SITL/vision_dashing_ws/src/apriltag2_node/node/cfg/platform.yaml"
    argument_path="/home/andrea/Marea/mar_frankenrotor_ros/ros2SITL/Firmware/Tools/sitl_gazebo/worlds/iris.world"
    return LaunchDescription([
        launch_ros.actions.Node(
            # the name of the executable is set in CMakeLists.txt, towards the end of
            # the file, in add_executable(...) and the directives following it
           package='lcm_bridge', node_executable='bridge_node', output='screen'),
	launch_ros.actions.Node(package='ultrasonic_sensor', node_executable='ultrasonic_sensor_node_Mocap2mav', 					output='screen'),

	launch_ros.actions.Node(package='apriltag2_node', node_executable='apriltag2_node', output='screen',
            parameters=[
                parameters_file_path
      		],
	),
	
	# DeclareLaunchArgument(
		# 'world_name',
		# default_value = argument_path,
		# description = 'Argument for Gazebo launch file'),

	# IncludeLaunchDescription( 
	# PythonLaunchDescriptionSource(['/opt/ros/dashing/share/gazebo_ros/launch/empty_world.launch.py']),
	# launch_arguments={'world_name': argument_path}.items()
       #  ),
    ])
