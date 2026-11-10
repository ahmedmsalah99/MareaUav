import os

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess,
                            IncludeLaunchDescription, SetEnvironmentVariable)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


# Mocap lab (M2M_MODE=1): the real drone (Raspberry Pi + Pixracer) flown on
# OptiTrack.
#   OptiTrack/Motive --NatNet--> natnet_ros2 --> /<rigid body>/pose
#                                 mocap_bridge --> PX4 external vision
#                                              --> pad nav_filter/data + IMU
#   Pi camera (v4l2_camera) --> apriltag2_node --> guidance
#   MicroXRCEAgent <--serial--> PX4 (uXRCE-DDS)
#
# Build with M2M_MODE=1 exported (real ultrasonic / payload code).
# PX4 must fuse vision instead of GPS: see load_nodes/px4/mocap_ekf2.params.
# In Motive: rigid bodies named like mocap_bridge.yaml's topics (default
# "drone" and "pad"), Up Axis = Z, NatNet streaming on.
#
# Example:
#   ros2 launch load_nodes ros2_mocap.launch.py server_ip:=192.168.1.10 client_ip:=192.168.1.20
# Pieces that run elsewhere or are not installed can be switched off:
#   start_natnet:=false start_camera:=false start_agent:=false
def generate_launch_description():

    args = [
        DeclareLaunchArgument('start_natnet', default_value='true',
                              description='Start natnet_ros2 (OptiTrack client) here'),
        DeclareLaunchArgument('server_ip', default_value='192.168.0.100',
                              description='Motive PC IP (its Local Interface in Motive)'),
        DeclareLaunchArgument('client_ip', default_value='192.168.0.103',
                              description='IP of this machine on the mocap network'),
        DeclareLaunchArgument('server_type', default_value='multicast',
                              description='NatNet streaming: multicast or unicast'),
        DeclareLaunchArgument('start_camera', default_value='true',
                              description='Start the Pi camera driver (v4l2_camera)'),
        DeclareLaunchArgument('video_device', default_value='/dev/video0',
                              description='Camera device (legacy Pi camera stack)'),
        DeclareLaunchArgument('start_agent', default_value='true',
                              description='Start MicroXRCEAgent (PX4 <-> ROS 2 link)'),
        DeclareLaunchArgument('agent_device', default_value='/dev/ttyAMA0',
                              description='Serial port wired to the flight controller'),
        DeclareLaunchArgument('agent_baud', default_value='921600',
                              description='Must match the PX4 serial port baud (e.g. SER_TEL2_BAUD)'),
    ]

    # ---------- Parameter files ----------
    apriltag_params = os.path.join(get_package_share_directory('apriltag2_node'),
                                   'cfg', 'apriltag_mocap.yaml')
    rescue_params = os.path.join(get_package_share_directory('apriltag2_rescue_node'),
                                 'cfg', 'tag_real.yaml')
    mocap_bridge_params = os.path.join(get_package_share_directory('mocap_bridge'),
                                       'config', 'mocap_bridge.yaml')

    # ---------- External pieces (optional) ----------
    # OptiTrack client: publishes each Motive rigid body as PoseStamped on
    # /<name>/pose. pub_rigid_body must be true (its default is false).
    # _share_or_empty(): the launch file still loads when natnet_ros2 is not
    # installed, as long as start_natnet:=false.
    natnet = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(_share_or_empty('natnet_ros2'), 'launch', 'natnet_ros2.launch.py')),
        launch_arguments={
            'serverIP': LaunchConfiguration('server_ip'),
            'clientIP': LaunchConfiguration('client_ip'),
            'serverType': LaunchConfiguration('server_type'),
            'pub_rigid_body': 'true',
            'activate': 'true',
        }.items(),
        condition=IfCondition(LaunchConfiguration('start_natnet')),
    )

    # Pi camera v2 on the legacy camera stack (/dev/video0). The compressed
    # transport publishes /apriltag/image/compressed/compressed with format
    # "rgb8; jpeg compressed bgr8", which is what apriltag2_node decodes.
    camera = Node(
        package='v4l2_camera',
        executable='v4l2_camera_node',
        output='screen',
        parameters=[{'video_device': LaunchConfiguration('video_device'),
                     'image_size': [640, 480],
                     'output_encoding': 'rgb8'}],
        remappings=[('image_raw', '/apriltag/image/compressed'),
                    ('camera_info', '/apriltag/image/camera_info')],
        condition=IfCondition(LaunchConfiguration('start_camera')),
    )

    # PX4 <-> ROS 2 link (uXRCE-DDS client on the Pixracer, agent here).
    agent = ExecuteProcess(
        cmd=['MicroXRCEAgent', 'serial', '--dev', LaunchConfiguration('agent_device'),
             '-b', LaunchConfiguration('agent_baud')],
        output='screen',
        condition=IfCondition(LaunchConfiguration('start_agent')),
    )

    # ---------- Our nodes ----------
    px4_bridge_node = Node(package='px4_bridge', executable='bridge_node', output='screen')

    ultrasonic_node = Node(package='ultrasonic_sensor',
                           executable='ultrasonic_sensor_node_Mocap2mav', output='screen')

    payload_node = Node(package='payload_releaser', executable='payload_releaser', output='screen')

    safety_node = Node(package='guidance_system', executable='safety_handler_node', output='screen')

    exec_node = Node(package='guidance_system', executable='executioner_node', output='screen',
                     parameters=[{'mission_list': 'list_mocap.txt'}])

    autom_node = Node(package='guidance_system', executable='autom_node', output='screen',
                      # +90 deg: the camera is turned 90 deg from the nose (rescue/SEARCH heading only).
                      parameters=[{'rescue_camera_heading_offset_rad': 1.57}])

    apriltag_node = Node(package='apriltag2_node', executable='apriltag2_node', output='screen',
                         parameters=[apriltag_params])

    rescue_node = Node(package='apriltag2_rescue_node', executable='apriltag2_rescue_node',
                       output='screen', parameters=[rescue_params])

    # Same estimator settings as the sim launch files (10 Hz input).
    spa_node = Node(package='spa_predictor', executable='spa_predictor_node', output='screen',
                    parameters=[{'sample_rate': 10.0, 'fft_window_size': 256, 'peak_threshold': 0.2}])

    heave_node = Node(package='heave_estimator', executable='heave_estimator_node', output='screen',
                      parameters=[{'sample_rate': 10.0, 'fft_window_size': 256}])

    # Drone/pad rigid-body topics come from mocap_bridge.yaml (/drone/pose, /pad/pose).
    mocap_bridge_node = Node(package='mocap_bridge', executable='mocap_bridge_node', output='screen',
                             parameters=[mocap_bridge_params])

    return LaunchDescription(args + [
        SetEnvironmentVariable(name='M2M_MODE', value='1'),
        agent,
        natnet,
        camera,
        px4_bridge_node,
        ultrasonic_node,
        payload_node,
        apriltag_node,
        rescue_node,
        safety_node,
        exec_node,
        autom_node,
        spa_node,
        heave_node,
        mocap_bridge_node,
    ])


def _share_or_empty(package):
    """Share dir of a package, or '' if it is not installed (so the launch file
    still loads with that piece switched off)."""
    try:
        return get_package_share_directory(package)
    except Exception:
        return ''
