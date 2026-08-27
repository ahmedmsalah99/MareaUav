#!/usr/bin/env python3
"""Switch PX4 to Offboard mode and arm, reporting exactly why if it refuses.

Sends VEHICLE_CMD_DO_SET_MODE (offboard) then VEHICLE_CMD_COMPONENT_ARM_DISARM,
and listens on /fmu/out/vehicle_command_ack + /fmu/out/vehicle_status so the
result is visible instead of having to guess.

Like the upstream px4_ros_com offboard example, this streams the
OffboardControlMode heartbeat and a hold setpoint itself by default: PX4 only
accepts offboard once BOTH are already flowing, and it drops back out if the
heartbeat rate falls below ~2 Hz. bridge_node publishes the heartbeat
unconditionally but only forwards trajectory_setpoint once guidance commands
something, so with no task active that precondition is unmet.

Usage:
    python3 offboard_arm.py                 # stream setpoints, go offboard, arm
    python3 offboard_arm.py --no-setpoints  # commands only (something else streams)
    python3 offboard_arm.py --no-arm        # only switch to offboard, don't arm
    python3 offboard_arm.py --sys-id 2      # target a non-default MAV_SYS_ID
"""

import argparse
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand

try:
    from px4_msgs.msg import VehicleCommandAck
except ImportError:  # depends on px4_msgs version
    VehicleCommandAck = None

# px4_msgs renamed this across versions; try the variants in order.
VehicleStatus = None
for _name in ('VehicleStatus', 'VehicleStatusV1'):
    try:
        VehicleStatus = getattr(__import__('px4_msgs.msg', fromlist=[_name]), _name)
        break
    except (ImportError, AttributeError):
        continue

ACK_RESULT = {
    0: 'ACCEPTED',
    1: 'TEMPORARILY_REJECTED (a precondition is not met yet -- usually no '
       'setpoint stream, or the EKF has no valid position)',
    2: 'DENIED',
    3: 'UNSUPPORTED',
    4: 'FAILED',
    5: 'IN_PROGRESS',
    6: 'CANCELLED',
}

NAV_STATE_OFFBOARD = 14
ARMING_STATE_ARMED = 2


class OffboardArmer(Node):

    def __init__(self, args):
        super().__init__('offboard_armer')
        self.args = args

        # PX4 publishes /fmu/out/* best-effort; subscribers must match.
        sub_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )
        # Publishers to /fmu/in/* must be RELIABLE (rclcpp/rclpy default), which
        # is what the upstream px4_ros_com offboard example uses --
        # create_publisher<T>(topic, 10). The XRCE-DDS agent subscribes to these
        # topics RELIABLE, and a BEST_EFFORT publisher is INCOMPATIBLE with a
        # RELIABLE subscriber: the endpoints never match, so every command is
        # silently dropped with no error anywhere. bridge_node gets this right
        # by passing a plain depth (default QoS) rather than a best-effort one.
        pub_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.vehicle_command_pub = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', pub_qos)

        if args.setpoints:
            self.offboard_mode_pub = self.create_publisher(
                OffboardControlMode, '/fmu/in/offboard_control_mode', pub_qos)
            self.setpoint_pub = self.create_publisher(
                TrajectorySetpoint, '/fmu/in/trajectory_setpoint', pub_qos)
            self.create_timer(0.05, self._stream_setpoints)  # 20 Hz

        if VehicleCommandAck is not None:
            self.create_subscription(
                VehicleCommandAck, '/fmu/out/vehicle_command_ack',
                self._on_ack, sub_qos)
        else:
            self.get_logger().warn(
                'px4_msgs has no VehicleCommandAck -- cannot report rejection reasons.')

        self._status_seen = False
        if VehicleStatus is not None:
            for topic in ('/fmu/out/vehicle_status', '/fmu/out/vehicle_status_v1'):
                self.create_subscription(
                    VehicleStatus, topic, self._on_status, sub_qos)
        else:
            self.get_logger().warn(
                'px4_msgs has no VehicleStatus -- cannot report nav/arming state.')

    # ---- outgoing ---------------------------------------------------------

    def _stream_setpoints(self):
        mode = OffboardControlMode()
        mode.position = True
        mode.velocity = False
        mode.acceleration = False
        mode.attitude = False
        mode.body_rate = False
        mode.timestamp = self._now_us()
        self.offboard_mode_pub.publish(mode)

        sp = TrajectorySetpoint()
        sp.position = [0.0, 0.0, -2.0]  # NED: 2 m above the local origin
        sp.yaw = 0.0
        sp.timestamp = self._now_us()
        self.setpoint_pub.publish(sp)

    def set_offboard(self):
        # base_mode = 1 (MAV_MODE_FLAG_CUSTOM_MODE_ENABLED),
        # custom_main_mode = 6 (PX4_CUSTOM_MAIN_MODE_OFFBOARD)
        self._publish_command(
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)

    def arm(self):
        self._publish_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)

    def _publish_command(self, command, param1=0.0, param2=0.0):
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = param1
        msg.param2 = param2
        msg.target_system = self.args.sys_id
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = self._now_us()
        self.vehicle_command_pub.publish(msg)

    def _now_us(self):
        return int(self.get_clock().now().nanoseconds / 1000)

    # ---- incoming ---------------------------------------------------------

    def _on_ack(self, msg):
        name = {
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE: 'DO_SET_MODE',
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM: 'ARM_DISARM',
        }.get(msg.command)
        if name is None:
            return  # someone else's command
        result = ACK_RESULT.get(msg.result, 'UNKNOWN({})'.format(msg.result))
        log = self.get_logger().info if msg.result == 0 else self.get_logger().error
        log('ACK {}: {}'.format(name, result))

    def _on_status(self, msg):
        offboard = msg.nav_state == NAV_STATE_OFFBOARD
        armed = msg.arming_state == ARMING_STATE_ARMED
        line = ('status: nav_state={} ({}) arming_state={} ({})'.format(
            msg.nav_state, 'OFFBOARD' if offboard else 'not offboard',
            msg.arming_state, 'ARMED' if armed else 'not armed'))
        for extra in ('pre_flight_checks_pass', 'failsafe'):
            if hasattr(msg, extra):
                line += ' {}={}'.format(extra, getattr(msg, extra))
        self._status_seen = True
        self.get_logger().info(line, throttle_duration_sec=1.0)


def spin_for(node, seconds):
    end = node.get_clock().now().nanoseconds + int(seconds * 1e9)
    while rclpy.ok() and node.get_clock().now().nanoseconds < end:
        rclpy.spin_once(node, timeout_sec=0.05)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--no-setpoints', dest='setpoints', action='store_false',
                        help='do not stream the heartbeat + hold setpoint '
                             '(only when something else, e.g. bridge_node driven '
                             'by an active guidance task, is already streaming both)')
    parser.add_argument('--no-arm', action='store_true',
                        help='switch to offboard but do not arm')
    parser.add_argument('--sys-id', type=int, default=1,
                        help='target MAV_SYS_ID (default 1)')
    args = parser.parse_args()

    rclpy.init()
    node = OffboardArmer(args)
    try:
        # Let discovery settle (and the setpoint stream warm up, if enabled)
        # before commanding -- a command published before the XRCE agent has
        # matched this publisher goes nowhere.
        node.get_logger().info('Waiting for discovery...')
        spin_for(node, 2.0)

        if not node._status_seen and VehicleStatus is not None:
            node.get_logger().warn(
                'No vehicle_status received -- PX4/XRCE agent may not be connected, '
                'in which case no command can reach the vehicle.')

        node.get_logger().info('Requesting OFFBOARD mode...')
        node.set_offboard()
        spin_for(node, 1.0)

        if not args.no_arm:
            node.get_logger().info('Requesting ARM...')
            node.arm()
            spin_for(node, 1.0)

        # Keep reporting status (and streaming setpoints, if enabled) so the
        # resulting mode is visible; Ctrl-C to stop.
        node.get_logger().info('Monitoring state -- Ctrl-C to exit.')
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
