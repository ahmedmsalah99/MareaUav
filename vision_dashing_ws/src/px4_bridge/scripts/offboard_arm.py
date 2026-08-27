#!/usr/bin/env python3
"""Switch PX4 to Offboard mode and arm the drone.

Does NOT publish any OffboardControlMode/TrajectorySetpoint — it assumes
something else (e.g. px4_bridge's bridge_node) is already publishing that
stream. This script only sends the two VehicleCommands: mode switch to
Offboard, then arm.

Usage:
    python3 offboard_arm.py
"""

import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from px4_msgs.msg import VehicleCommand

ARM_DELAY_S = 0.5  # gap between the mode-switch command and the arm command


class OffboardArmer(Node):

    def __init__(self):
        super().__init__('offboard_armer')

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.vehicle_command_pub = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', qos)

    def _set_mode(self):
        # base_mode=1, custom_main_mode=6/PX4_CUSTOM_MAIN_MODE_OFFBOARD
        self._publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)

    def _arm(self):
        self._publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)  # 1 = arm

    def _publish_vehicle_command(self, command, param1=0.0, param2=0.0):
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = param1
        msg.param2 = param2
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = self._now_us()
        self.vehicle_command_pub.publish(msg)

    def _now_us(self):
        return int(self.get_clock().now().nanoseconds / 1000)


def main():
    rclpy.init()
    node = OffboardArmer()
    try:
        node._set_mode()
        node.get_logger().info('Requested OFFBOARD mode')
        time.sleep(ARM_DELAY_S)

        node._arm()
        node.get_logger().info('Requested ARM')
        time.sleep(0.1)  # give the publisher a moment to flush before shutdown
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
