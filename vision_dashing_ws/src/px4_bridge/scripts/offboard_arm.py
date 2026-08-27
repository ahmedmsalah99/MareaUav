#!/usr/bin/env python3
"""Switch PX4 to Offboard mode and arm, confirming both actually took effect.

Sends VEHICLE_CMD_DO_SET_MODE (offboard) then VEHICLE_CMD_COMPONENT_ARM_DISARM,
and does NOT exit until /fmu/out/vehicle_status reports nav_state == OFFBOARD
and arming_state == ARMED. A VehicleCommand can be dropped outright, or ACKed
and still not take effect a moment later, so each command is re-sent until
vehicle_status confirms it, and the script exits non-zero if it never does.
/fmu/out/vehicle_command_ack is reported alongside so a refusal says why.

PX4 only accepts offboard once a matching setpoint stream is already flowing
(the OffboardControlMode heartbeat AND the setpoint it advertises, e.g.
trajectory_setpoint for position) -- normally bridge_node provides both, but it
only forwards trajectory_setpoint once the guidance node actually commands
something. Use --setpoints if nothing else is streaming them.

Exit status: 0 once confirmed, 1 if offboard or arming could not be confirmed.

Usage:
    python3 offboard_arm.py                # commands only (bridge_node streaming)
    python3 offboard_arm.py --setpoints    # also stream heartbeat + hold setpoint
    python3 offboard_arm.py --no-arm       # only switch to offboard, don't arm
    python3 offboard_arm.py --sys-id 2     # target a non-default MAV_SYS_ID
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
        pub_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
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
        self.is_offboard = False
        self.is_armed = False
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
        self.is_offboard = msg.nav_state == NAV_STATE_OFFBOARD
        self.is_armed = msg.arming_state == ARMING_STATE_ARMED
        line = ('status: nav_state={} ({}) arming_state={} ({})'.format(
            msg.nav_state, 'OFFBOARD' if self.is_offboard else 'not offboard',
            msg.arming_state, 'ARMED' if self.is_armed else 'not armed'))
        for extra in ('pre_flight_checks_pass', 'failsafe'):
            if hasattr(msg, extra):
                line += ' {}={}'.format(extra, getattr(msg, extra))
        self._status_seen = True
        self.get_logger().info(line, throttle_duration_sec=1.0)


def spin_for(node, seconds):
    end = node.get_clock().now().nanoseconds + int(seconds * 1e9)
    while rclpy.ok() and node.get_clock().now().nanoseconds < end:
        rclpy.spin_once(node, timeout_sec=0.05)


def spin_until(node, predicate, seconds):
    """Spin until predicate() is true, or the timeout expires. Returns the result.

    Keeps spinning rather than sleeping so the setpoint stream (when enabled)
    stays alive and vehicle_status keeps arriving while we wait.
    """
    end = node.get_clock().now().nanoseconds + int(seconds * 1e9)
    while rclpy.ok() and node.get_clock().now().nanoseconds < end:
        if predicate():
            return True
        rclpy.spin_once(node, timeout_sec=0.05)
    return predicate()


def command_until(node, send, predicate, what, attempts, per_attempt_s):
    """Send a command and confirm it took effect, retrying on failure.

    A VehicleCommand can be accepted and still not take effect (a precondition
    clears a moment later), and it can be dropped outright, so confirm against
    vehicle_status rather than assuming the command worked.
    """
    for attempt in range(1, attempts + 1):
        node.get_logger().info(
            'Requesting {} (attempt {}/{})...'.format(what, attempt, attempts))
        send()
        if spin_until(node, predicate, per_attempt_s):
            node.get_logger().info('{} confirmed.'.format(what))
            return True
    node.get_logger().error(
        '{} NOT confirmed after {} attempts -- see the ACK and status lines above.'
        .format(what, attempts))
    return False


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--setpoints', action='store_true',
                        help='also stream the offboard heartbeat + a hold setpoint')
    parser.add_argument('--no-arm', action='store_true',
                        help='switch to offboard but do not arm')
    parser.add_argument('--sys-id', type=int, default=1,
                        help='target MAV_SYS_ID (default 1)')
    parser.add_argument('--attempts', type=int, default=5,
                        help='how many times to re-send a command that has not '
                             'taken effect (default 5)')
    parser.add_argument('--timeout', type=float, default=3.0,
                        help='seconds to wait for each command to be confirmed '
                             'in vehicle_status (default 3)')
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

        if VehicleStatus is None:
            # Without vehicle_status there is nothing to confirm against, so
            # fall back to fire-and-hope rather than silently claiming success.
            node.get_logger().warn(
                'Cannot confirm the result without VehicleStatus -- '
                'sending commands blind.')
            node.set_offboard()
            spin_for(node, 1.0)
            if not args.no_arm:
                node.arm()
                spin_for(node, 1.0)
            return 0

        if not command_until(node, node.set_offboard,
                             lambda: node.is_offboard, 'OFFBOARD mode',
                             args.attempts, args.timeout):
            return 1

        if not args.no_arm:
            if not command_until(node, node.arm,
                                 lambda: node.is_armed, 'ARM',
                                 args.attempts, args.timeout):
                return 1

        node.get_logger().info(
            'Done -- offboard {}; exiting.'.format(
                'enabled and armed' if not args.no_arm else 'enabled'))
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
