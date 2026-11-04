import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import SetEnvironmentVariable


# Mocap in sim (M2M_MODE=3): Gazebo flown as in the mocap lab.
#   - the landingplat plugin stops driving the pad and publishes fake OptiTrack
#     poses of every model on /mocap/<model>/pose (Gazebo must ALSO run with M2M_MODE=3:
#     export it in the terminal that starts `make px4_sitl gazebo-classic_...`)
#   - mocap_bridge feeds PX4 external vision and the pad sea-state topics
#   - do NOT play the boat bag: the pad's nav_filter/IMU come from mocap_bridge
#   - PX4 must fuse vision instead of GPS: see load_nodes/px4/mocap_ekf2.params
def generate_launch_description():

    # ---------- Paths (portable) ----------
    apriltag_pkg = get_package_share_directory('apriltag2_node')
    rescue_pkg = get_package_share_directory('apriltag2_rescue_node')

    parameters_file_path = os.path.join(
        apriltag_pkg,
         'cfg', 'apriltag_sim.yaml'   # per-mode: camera, mounting, pad layout
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
    px4_bridge_node = Node(
        package='px4_bridge',
        executable='bridge_node',
        output='screen'
    )

    ultrasonic_node = Node(
        package='ultrasonic_sensor',
        executable='ultrasonic_sensor_node_Mocap2mav',
        output='screen'
    	# ,prefix='xterm -e gdb -ex run --args'
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
        output='screen',
        parameters=[{'mission_list': 'list_mocap.txt'}],
    )

    autom_node = Node(
        package='guidance_system',
        executable='autom_node',
        output='screen',
        # +90 deg: the camera is turned 90 deg from the nose (rescue/SEARCH heading only).
        parameters=[{'rescue_camera_heading_offset_rad': 1.57}],
    )

    apriltag_node = Node(
    package='apriltag2_node',
    executable='apriltag2_node',
    output='screen',
    parameters=[parameters_file_path]
    # ,prefix='xterm -e gdb -ex run --args'
)

    rescue_node = Node(
        package='apriltag2_rescue_node',
        executable='apriltag2_rescue_node',
        output='screen',
        parameters=[parameters_file_path2]
    	# ,prefix='xterm -e gdb -ex run --args'
    )

    # Both nodes default to sample_rate=30.0/fft_window_size=1024, tuned for
    # the old simulated plugin IMU's much faster publish rate. Their real
    # sources (/ulisse/nav_filter/data, /adnav_node/imu_raw) publish at
    # ~10 Hz — left at the 30 Hz default, the FFT identifier's bin->frequency
    # mapping comes out ~3x wrong (bad estimates) and the 1024-sample warm-up
    # window takes ~100+ s instead of the ~34 s the default was tuned for
    # (heaveValid stays false until then, blocking COMP/LAND). fft_window_size
    # dropped to 256 to keep warm-up time roughly in line with the original
    # 30 Hz/1024 tuning at the new 10 Hz rate.
    # peak_threshold raised from the 0.05 default to 0.2: against real
    # roll/pitch data, identify() was consistently hitting the 6-mode cap —
    # per-mode logs showed 2 genuinely dominant modes per axis (well above
    # 40% of the strongest mode's amplitude, and cross-validated against
    # each other — e.g. roll and yaw both independently identifying the
    # same 5.1s-period mode) plus a cluster of noise-floor peaks at
    # 6-16% of the max, which 0.05 let straight through as "modes",
    # feeding the Kalman observer several spurious frequencies and
    # producing a noisy, wrong-amplitude spa/estimate.
    spa_node = Node(
        package='spa_predictor',
        executable='spa_predictor_node',
        output='screen',
        parameters=[{'sample_rate': 10.0, 'fft_window_size': 256, 'peak_threshold': 0.2,
                     'calm_angle_std_max_rad': 0.01}],   # calm-sea fallback (still platform)
    )

    heave_node = Node(
        package='heave_estimator',
        executable='heave_estimator_node',
        output='screen',
        parameters=[{'sample_rate': 10.0, 'fft_window_size': 256,
                     'calm_vel_std_max_mps': 0.02}],      # calm-sea fallback (still platform)
    )

    mocap_bridge_node = Node(
        package='mocap_bridge',
        executable='mocap_bridge_node',
        output='screen',
        parameters=[
            os.path.join(get_package_share_directory('mocap_bridge'), 'config', 'mocap_bridge.yaml'),
            # In sim the landingplat plugin publishes every Gazebo model on
            # /mocap/<model>/pose; pick the drone and the pad (in the lab these
            # are natnet_ros2's rigid-body topics from mocap_bridge.yaml).
            {'drone_pose_topic': '/mocap/iris/pose',
             'pad_pose_topic':   '/mocap/Landing_Site/pose'},
        ],
    )

    return LaunchDescription([
        SetEnvironmentVariable(
			name='M2M_MODE',
			value='3'
		),
        bridge_launch,
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
        mocap_bridge_node
    ])