import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import SetEnvironmentVariable


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
    # ,prefix='xterm -e gdb -ex run --args'
)

    rescue_node = Node(
        package='apriltag2_rescue_node',
        executable='apriltag2_rescue_node',
        output='screen',
        parameters=[parameters_file_path2]
    	# ,prefix='xterm -e gdb -ex run --args'
    )

    return LaunchDescription([
        SetEnvironmentVariable(
			name='M2M_SIM',
			value='1'
		),
        bridge_launch,
        px4_bridge_node,
        ultrasonic_node,
        payload_node,
        apriltag_node,
        rescue_node,
        safety_node,
        exec_node,
        autom_node
    ])