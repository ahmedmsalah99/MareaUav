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
    parameters_file_path2 = "/home/andrea/Marea/mar_frankenrotor_ros/ros2SITL/vision_dashing_ws/src/apriltag2_rescue_node/node/cfg/tag.yaml"
    test_config="/home/andrea/Marea/mar_frankenrotor_ros/ros2SITL/vision_dashing_ws/test_config.yaml"
    return LaunchDescription([
        launch_ros.actions.Node(
            # the name of the executable is set in CMakeLists.txt, towards the end of
            # the file, in add_executable(...) and the directives following it
           package='px4_bridge', node_executable='bridge_node', output='screen'),
	launch_ros.actions.Node(package='ultrasonic_sensor', node_executable='ultrasonic_sensor_node_Mocap2mav', output='screen'),

	launch_ros.actions.Node(package='payload_releaser', node_executable='payload_releaser', output='screen'),

	launch_ros.actions.Node(package='apriltag2_node', node_executable='apriltag2_node', output='screen',
            parameters=[
                parameters_file_path
      		],
	),

	launch_ros.actions.Node(package='apriltag2_rescue_node', node_executable='apriltag2_rescue_node', output='screen',
            parameters=[
                parameters_file_path2
      		],
	),

	launch_ros.actions.Node(package='fsm_raspicam', node_executable='camera_main_node', output='screen',
	),
    ])
