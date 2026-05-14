import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    # Note: radiation_type ULTRASOUND=0 INFRARED=1
    # See https://github.com/ros-tooling/topic_tools/blob/jazzy/topic_tools/topic_tools/relay_field.py

    # sonar_F_L_node = Node(
    #             package='topic_tools',
    #             executable='relay_field',
    #             name='sonar_relay',
    #             output='screen',
    #             respawn=True,
    #             respawn_delay=2.0,
    #             arguments=['/laser/sonar', '/ultrasonic/sonar', 'sensor_msgs/msg/Range', \
    #                         '{ \
    #                             header: { \
    #                                     stamp: {sec: m.header.stamp.sec, nanosec: m.header.stamp.nanosec}, \
    #                                     frame_id: m.header.frame_id \
    #                                   }, \
    #                             field_of_view: 0.1, \
    #                             max_range: m.range_max, \
    #                             min_range: m.range_min, \
    #                             radiation_type: 0, \
    #                             range: "min(m.ranges)", \
    #                           } \
    #                      ']
    #         )
    sonar_F_L_node = Node(
    package='topic_tools',
    executable='relay_field',
    name='sonar_relay',
    output='screen',
    respawn=True,
    respawn_delay=2.0,
    arguments=[
        '/laser/sonar', 
        '/ultrasonic/sonar', 
        'sensor_msgs/msg/Range',
        # Use a simple Python expression. 
        # Note: We remove the extra quotes around the math to let relay_field evaluate it directly.
        '{header: m.header, \
          field_of_view: 0.1, \
          max_range: m.range_max, \
          min_range: m.range_min, \
          radiation_type: 0, \
          range: "m.range_max if min(m.ranges) > m.range_max else min(m.ranges)"}'
    ]
)
     # Create the launch description and populate
    ld = LaunchDescription()

    # Add the actions to launch all of the navigation nodes
    ld.add_action(sonar_F_L_node)

    return ld