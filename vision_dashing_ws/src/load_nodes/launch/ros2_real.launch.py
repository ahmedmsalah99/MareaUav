import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


# Real hardware at sea (M2M_MODE=2): the real drone (Raspberry Pi + Pixracer)
# landing on the real catamaran, positioned by GPS.
#   Pi camera (v4l2_camera) --> apriltag2_node --> guidance
#   MicroXRCEAgent <--serial--> PX4 (uXRCE-DDS)
#   Catamaran sensors (/ulisse/nav_filter/data, /adnav_node/imu_raw) come from
#   the boat's own stack over the network, not from this launch file.
#   time_synchro_gps_sys --> PX4 global origin + computer clock from GPS
#
# Build with M2M_MODE=2 exported (real ultrasonic / payload code).
# PX4: normal GPS flight parameters (not mocap_ekf2.params).
#
# Example:
#   ros2 launch load_nodes ros2_real.launch.py
# Pieces that run elsewhere or are not installed can be switched off:
#   start_camera:=false start_agent:=false
def generate_launch_description():

    args = [
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
        DeclareLaunchArgument('mission_list', default_value='list.txt',
                              description='Mission file in guidance_system/lists'),
    ]

    # ---------- Parameter files ----------
    apriltag_params = os.path.join(get_package_share_directory('apriltag2_node'),
                                   'cfg', 'apriltag_hw.yaml')
    rescue_params = os.path.join(get_package_share_directory('apriltag2_rescue_node'),
                                 'cfg', 'tag_real.yaml')

    # ---------- External pieces (optional) ----------
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

    # Lab node, unchanged: sets PX4's global origin (lab lat/lon) and, once PX4's
    # GPS reports UTC time, the computer's clock (needs root). It is written for
    # the lab's namespaced drones (/x500_N/fmu/...): agent 'x500_1' gives
    # target_system 1, and the remaps point it at our un-namespaced PX4 topics.
    gps_sync_node = Node(
        package='time_synchro_gps_sys', executable='time_synchro_gps_sys_main', output='screen',
        arguments=['x500_1'],
        remappings=[('/x500_1/fmu/out/sensor_gps', '/fmu/out/sensor_gps'),
                    ('/x500_1/fmu/out/vehicle_command_ack', '/fmu/out/vehicle_command_ack'),
                    ('/x500_1/fmu/in/vehicle_command', '/fmu/in/vehicle_command')],
    )

    # ---------- Our nodes ----------
    px4_bridge_node = Node(package='px4_bridge', executable='bridge_node', output='screen')

    ultrasonic_node = Node(package='ultrasonic_sensor',
                           executable='ultrasonic_sensor_node_Mocap2mav', output='screen')

    payload_node = Node(package='payload_releaser', executable='payload_releaser', output='screen')

    safety_node = Node(package='guidance_system', executable='safety_handler_node', output='screen')

    exec_node = Node(package='guidance_system', executable='executioner_node', output='screen',
                     parameters=[{'mission_list': LaunchConfiguration('mission_list')}])

    autom_node = Node(package='guidance_system', executable='autom_node', output='screen',
                      # +90 deg: the camera is turned 90 deg from the nose (rescue/SEARCH heading only).
                      parameters=[{'rescue_camera_heading_offset_rad': 1.57}])

    apriltag_node = Node(package='apriltag2_node', executable='apriltag2_node', output='screen',
                         parameters=[apriltag_params])

    rescue_node = Node(package='apriltag2_rescue_node', executable='apriltag2_rescue_node',
                       output='screen', parameters=[rescue_params])

    # Same estimator settings as the other launch files (the boat's data is ~10 Hz).
    spa_node = Node(package='spa_predictor', executable='spa_predictor_node', output='screen',
                    parameters=[{'sample_rate': 10.0, 'fft_window_size': 256, 'peak_threshold': 0.2}])

    heave_node = Node(package='heave_estimator', executable='heave_estimator_node', output='screen',
                      parameters=[{'sample_rate': 10.0, 'fft_window_size': 256}])

    return LaunchDescription(args + [
        SetEnvironmentVariable(name='M2M_MODE', value='2'),
        agent,
        camera,
        gps_sync_node,
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
    ])
