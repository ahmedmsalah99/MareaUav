import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    # ---------- Paths (portable) ----------
    apriltag_pkg = get_package_share_directory('apriltag2_node')
    rescue_pkg = get_package_share_directory('apriltag2_rescue_node')

    parameters_file_path = os.path.join(
        apriltag_pkg,
        'cfg', 'platform.yaml'
    )

    parameters_file_path2 = os.path.join(
        rescue_pkg,
        'cfg', 'tag.yaml'
    )

    # ---------- Included launch ----------
    bridge_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('load_nodes'),
                'launch',
                'bridge.launch.py'
            )
        )
    )

    # ---------- Nodes ----------
    # This is the hardware counterpart of ros2_sim.launch.py -- same node
    # graph (guidance/executive stack included, unlike the previous version
    # of this file), plus fsm_raspicam's camera node for the real RPi camera
    # pipeline, and without spawning anything Gazebo-specific.
    px4_bridge_node = Node(
        package='px4_bridge',
        executable='bridge_node',
        output='screen'
    )

    ultrasonic_node = Node(
        package='ultrasonic_sensor',
        executable='ultrasonic_sensor_node_Mocap2mav',
        output='screen'
    )

    payload_node = Node(
        package='payload_releaser',
        executable='payload_releaser',
        output='screen'
    )

    safety_node = Node(
        package='guidance_system',
        executable='safety_handler_node',
        output='screen'
    )

    exec_node = Node(
        package='guidance_system',
        executable='executioner_node',
        output='screen'
    )

    autom_node = Node(
        package='guidance_system',
        executable='autom_node',
        output='screen'
    )

    apriltag_node = Node(
        package='apriltag2_node',
        executable='apriltag2_node',
        output='screen',
        parameters=[parameters_file_path]
    )

    rescue_node = Node(
        package='apriltag2_rescue_node',
        executable='apriltag2_rescue_node',
        output='screen',
        parameters=[parameters_file_path2]
    )

    # Real RPi camera capture/publish pipeline -- has no equivalent in
    # ros2_sim.launch.py, where Gazebo's camera plugin fills this role
    # instead. Package lives outside vision_dashing_ws (top-level
    # fsm_raspicam/), so it isn't resolved through get_package_share_directory
    # here along with the others -- same executable name as the previous
    # version of this file.
    camera_node = Node(
        package='fsm_raspicam',
        executable='camera_main_node',
        output='screen'
    )

    # Same tuning rationale as ros2_sim.launch.py: real sources
    # (/ulisse/nav_filter/data, /adnav_node/imu_raw) publish at ~10 Hz, so
    # sample_rate/fft_window_size/peak_threshold are set to match -- these
    # were previously entirely absent from this launch file (spa_predictor
    # and heave_estimator weren't started at all).
    spa_node = Node(
        package='spa_predictor',
        executable='spa_predictor_node',
        output='screen',
        parameters=[{'sample_rate': 10.0, 'fft_window_size': 256, 'peak_threshold': 0.2}],
    )

    heave_node = Node(
        package='heave_estimator',
        executable='heave_estimator_node',
        output='screen',
        parameters=[{'sample_rate': 10.0, 'fft_window_size': 256}],
    )

    return LaunchDescription([
        SetEnvironmentVariable(
            name='M2M_SIM',
            value='0'
        ),
        bridge_launch,
        px4_bridge_node,
        ultrasonic_node,
        payload_node,
        apriltag_node,
        rescue_node,
        camera_node,
        safety_node,
        exec_node,
        autom_node,
        spa_node,
        heave_node,
    ])
