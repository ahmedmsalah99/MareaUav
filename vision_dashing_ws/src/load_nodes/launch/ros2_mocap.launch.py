import os

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess,
                            SetEnvironmentVariable)
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


# Mocap lab (M2M_MODE=1): the real drone (Raspberry Pi + Pixracer) flown on
# OptiTrack.
#   OptiTrack/Motive --NatNet--> mocap_optitrack --> /drone/pose, /pad/pose
#        /drone/pose --> mocap_px4_bridge --> PX4 external vision
#        /pad/pose   --> mocap_pad_bridge --> pad nav_filter/data + IMU
#   time_synchro_gps_sys --> PX4 global origin (+ clock from GPS when there is a fix)
#   Pi camera (v4l2_camera) --> apriltag2_node --> guidance
#   MicroXRCEAgent <--serial--> PX4 (uXRCE-DDS)
#
# Build with M2M_MODE=1 exported (real ultrasonic / payload code).
# PX4 must fuse vision instead of GPS: see load_nodes/px4/mocap_ekf2.params.
# In Motive: Up Axis = Z (mocap_px4_bridge's axis conversion assumes it),
# NatNet streaming (multicast) on, and the drone/pad Streaming IDs matching
# mocap_optitrack/config/mocap.yaml.
#
# Example:
#   ros2 launch load_nodes ros2_mocap.launch.py
# Pieces that run elsewhere or are not installed can be switched off:
#   start_optitrack:=false start_camera:=false start_agent:=false
def generate_launch_description():

    args = [
        DeclareLaunchArgument('start_optitrack', default_value='true',
                              description='Start mocap_optitrack (OptiTrack NatNet client) here'),
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
    optitrack_params = os.path.join(get_package_share_directory('mocap_optitrack'),
                                    'config', 'mocap.yaml')
    mocap_px4_bridge_params = os.path.join(get_package_share_directory('mocap_px4_bridge'),
                                           'config', 'params.yaml')
    mocap_pad_bridge_params = os.path.join(get_package_share_directory('mocap_pad_bridge'),
                                           'config', 'mocap_pad_bridge.yaml')

    # ---------- External pieces (optional) ----------
    # OptiTrack client (lab package): one PoseStamped per rigid body listed in
    # mocap_optitrack/config/mocap.yaml (drone -> /drone/pose, pad -> /pad/pose).
    optitrack = Node(
        package='mocap_optitrack',
        executable='mocap_node',
        name='mocap_node',
        output='screen',
        parameters=[optitrack_params],
        condition=IfCondition(LaunchConfiguration('start_optitrack')),
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

    # Drone pose -> PX4 external vision (lab package, config/params.yaml).
    mocap_px4_bridge_node = Node(package='mocap_px4_bridge', executable='mocap_px4_bridge',
                                 name='mocap_px4_bridge', output='screen',
                                 parameters=[mocap_px4_bridge_params])

    # Pad pose -> catamaran-style sea-state topics for the guidance.
    mocap_pad_bridge_node = Node(package='mocap_pad_bridge', executable='mocap_pad_bridge_node',
                                 output='screen', parameters=[mocap_pad_bridge_params])

    # Lab node, unchanged: sets PX4's global origin (lab lat/lon) and, once PX4's
    # GPS reports UTC time, the computer's clock (needs root; indoors there is
    # no fix, so that part simply never happens). It is written for the lab's
    # namespaced drones (/x500_N/fmu/...): agent 'x500_1' gives target_system 1,
    # and the remaps point it at our un-namespaced PX4 topics.
    gps_sync_node = Node(
        package='time_synchro_gps_sys', executable='time_synchro_gps_sys_main', output='screen',
        arguments=['x500_1'],
        remappings=[('/x500_1/fmu/out/sensor_gps', '/fmu/out/sensor_gps'),
                    ('/x500_1/fmu/out/vehicle_command_ack', '/fmu/out/vehicle_command_ack'),
                    ('/x500_1/fmu/in/vehicle_command', '/fmu/in/vehicle_command')],
    )

    return LaunchDescription(args + [
        SetEnvironmentVariable(name='M2M_MODE', value='1'),
        agent,
        optitrack,
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
        mocap_px4_bridge_node,
        mocap_pad_bridge_node,
        gps_sync_node,
    ])

