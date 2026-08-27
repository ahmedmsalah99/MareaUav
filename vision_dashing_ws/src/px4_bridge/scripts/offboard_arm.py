#!/usr/bin/env python3
"""Switch PX4 to Offboard mode and arm the drone.

PX4 requires a steady stream of OffboardControlMode + setpoint messages
*before* it will accept the Offboard mode switch, so this node publishes
a hold-in-place setpoint at 20 Hz for a short warm-up period, then sends
the "set mode: offboard" and "arm" VehicleCommands.

Usage:
    python3 offboard_arm.py
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand

WARMUP_SETPOINTS = 20  # ~1s at 20Hz, well above PX4's 10-setpoint minimum


class OffboardArmer(Node):

    def __init__(self):
        super().__init__('offboard_armer')

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.offboard_control_mode_pub = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', qos)
        self.trajectory_setpoint_pub = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos)
        self.vehicle_command_pub = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', qos)

        self._setpoint_count = 0
        self._timer = self.create_timer(0.05, self._tick)  # 20 Hz

    def _publish_offboard_control_mode(self):
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = self._now_us()
        self.offboard_control_mode_pub.publish(msg)

    def _publish_hold_setpoint(self):
        msg = TrajectorySetpoint()
        msg.position = [0.0, 0.0, -2.0]  # hold 2m above the local-origin start point (NED)
        msg.yaw = 0.0
        msg.timestamp = self._now_us()
        self.trajectory_setpoint_pub.publish(msg)

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

    def _tick(self):
        self._publish_offboard_control_mode()
        self._publish_hold_setpoint()

        if self._setpoint_count == WARMUP_SETPOINTS:
            # Set mode: offboard (base_mode=1, custom_main_mode=6/PX4_CUSTOM_MAIN_MODE_OFFBOARD)
            self._publish_vehicle_command(
                VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
            self.get_logger().info('Requested OFFBOARD mode')

        if self._setpoint_count == WARMUP_SETPOINTS + 10:
            self._publish_vehicle_command(
                VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)  # 1 = arm
            self.get_logger().info('Requested ARM')

        if self._setpoint_count == WARMUP_SETPOINTS + 20:
            self.get_logger().info('Done — offboard hold setpoint will keep publishing.')

        self._setpoint_count += 1


def main():
    rclpy.init()
    node = OffboardArmer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
