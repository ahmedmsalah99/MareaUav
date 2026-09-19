#!/usr/bin/env python3
"""Scale velocity/acceleration fields in a ROS 2 bag, leaving position and
orientation untouched.

Why this only touches rate-like fields:
  In sim, vision_dashing_ws/src/plugins/landingplat/src/plugin.cpp drives the
  Gazebo platform from /ulisse/simulated_system (or nav_filter/data replayed
  the same way): the message's absolute position
  (inertialframe_linear_position) is deliberately IGNORED and instead the
  platform's world position is dead-reckoned by integrating
  bodyframe_linear_velocity each tick (see simulatedSystemCallback() /
  OnUpdate() there). So scaling velocity is exactly consistent with how the
  sim already treats this data -- Gazebo recomputes position from whatever
  velocity you feed it.

  bodyframe_angular_position (roll/pitch/yaw) is the one exception: it is
  used directly as the platform's absolute orientation (slew-rate limited,
  not integrated from bodyframe_angular_velocity). bodyframe_angular_velocity
  itself only feeds the synthesized IMU gyro output (see the wx/wy/wz block
  in plugin.cpp), not the commanded rpy. That means scaling angular velocity
  changes the synthetic IMU's reported rotation rate WITHOUT changing the
  platform's actual simulated tilt motion (rpy is untouched) -- the two
  become inconsistent with each other. --scale-angular-velocity defaults to
  off for this reason; pass it explicitly once you've decided that tradeoff
  is fine for what you're testing.

Physical consistency is a strict rule, not a tunable: this script never
touches message timestamps, only field values. Acceleration is d(velocity)/dt
by definition -- with dt unchanged, scaling velocity by k means the ONLY
value of k that keeps acceleration consistent with it is k itself (NOT k**2;
that would only apply if timestamps were also being compressed/stretched,
which they are not). So acceleration fields always scale by the same
--factor as velocity fields -- there is no separate knob for it.

Usage:
    pip install rosbags
    ./scale_bag_rates.py INPUT_BAG OUTPUT_BAG --factor 1.5
    ./scale_bag_rates.py INPUT_BAG OUTPUT_BAG --factor 1.5 --list-fields   # dry run, no output bag written
"""

import argparse
import sys
from pathlib import Path

try:
    from rosbags.rosbag2 import Reader, StoragePlugin, Writer
    from rosbags.typesys import Stores, get_typestore, get_types_from_msg
except ImportError:
    sys.exit(
        "The 'rosbags' package is required.\n"
        "Install it with:  pip install rosbags"
    )

HUMBLE_METADATA_VERSION = 8

STORAGE_PLUGINS = {
    'sqlite3': StoragePlugin.SQLITE3,
    'mcap': StoragePlugin.MCAP,
}

# ---------------------------------------------------------------------------
# Field classification
# ---------------------------------------------------------------------------
#
# Two ways a field gets scaled:
#
# 1. EXPLICIT_RULES: exact (msgtype, dotted-field-path) entries for message
#    types this repo actually uses, so there is no ambiguity about what a
#    field means. Read these from the actual .msg definitions, not guessed.
#
# 2. Heuristic fallback (classify_by_name): for any other message type
#    (e.g. if you point this at a bag with topics not in EXPLICIT_RULES),
#    fields are scaled based on their name matching velocity/acceleration
#    tokens, and explicitly NOT scaled if their name matches a
#    position/orientation token first. This is a safety net, not the primary
#    mechanism -- always run --list-fields first on an unfamiliar bag and
#    check the output before trusting it.

# 'vel' and 'accel' entries both scale under --factor -- the same factor,
# always (see the module docstring: with timestamps unchanged, that's the
# only physically-consistent choice). 'angvel' also scales under --factor,
# but is additionally gated by --scale-angular-velocity.
EXPLICIT_RULES = {
    'ulisse_msgs/msg/SimulatedSystem': {
        'bodyframe_linear_velocity': 'vel',
        'bodyframe_angular_velocity': 'angvel',
        # NOT scaled: inertialframe_linear_position (unused by sim -- see
        # module docstring), bodyframe_angular_position (used directly as
        # absolute orientation), gyro_bias (a bias, not a rate to rescale),
        # inertialframe_water_current, n_p/n_s (thruster rpm -- not the
        # platform motion this script targets).
    },
    'ulisse_msgs/msg/NavFilterData': {
        'bodyframe_linear_velocity': 'vel',
        'bodyframe_angular_velocity': 'angvel',
    },
    'px4_msgs/msg/VehicleOdometry': {
        'velocity': 'vel',
        'angular_velocity': 'angvel',
        # NOT scaled: position, q (orientation quaternion).
    },
    'nav_msgs/msg/Odometry': {
        'twist.twist.linear': 'vel',
        'twist.twist.angular': 'angvel',
        # NOT scaled: pose.pose.position, pose.pose.orientation.
    },
    'geometry_msgs/msg/TwistStamped': {
        'twist.linear': 'vel',
        'twist.angular': 'angvel',
    },
    'geometry_msgs/msg/AccelStamped': {
        'accel.linear': 'accel',
        'accel.angular': 'accel',
    },
    'sensor_msgs/msg/Imu': {
        'linear_acceleration': 'accel',
        'angular_velocity': 'angvel',
        # NOT scaled: orientation.
    },
}

# Name tokens for the heuristic fallback, checked in order: a field whose
# dotted path contains a POSITION_TOKENS token is left alone even if it also
# contains a rate token (e.g. "angular_position" matches "position" first).
POSITION_TOKENS = (
    'position', 'pose', 'orientation', 'quaternion', 'lat', 'lon', 'alt',
    'latitude', 'longitude', 'altitude', 'gyro_bias', 'bias', 'covariance',
)
VEL_TOKENS = ('linear_velocity', 'lin_vel', 'twist')
ANGVEL_TOKENS = ('angular_velocity', 'ang_vel', 'angular_rate', 'gyro')
ACCEL_TOKENS = ('acceleration', 'accel')


def classify_by_name(path: str):
    lower = path.lower()
    if any(tok in lower for tok in POSITION_TOKENS):
        return None
    if any(tok in lower for tok in ANGVEL_TOKENS):
        return 'angvel'
    if any(tok in lower for tok in ACCEL_TOKENS):
        return 'accel'
    if any(tok in lower for tok in VEL_TOKENS):
        return 'vel'
    return None


def resolve_field(msg, dotted_path):
    """Walk a dotted attribute path, returning (parent_obj, leaf_attr_name)."""
    parts = dotted_path.split('.')
    obj = msg
    for part in parts[:-1]:
        obj = getattr(obj, part)
    return obj, parts[-1]


def scale_value(value, factor):
    """Scale a float, a numpy array (rosbags' representation for fixed-size
    .msg array fields -- element-wise * works directly), or a plain
    list/tuple sequence (variable-length .msg array fields)."""
    if type(value).__module__ == 'numpy':
        return value * factor
    if isinstance(value, (list, tuple)):
        return type(value)(v * factor for v in value)
    return value * factor


def iter_leaf_field_paths(msg, prefix=''):
    """Yield dotted paths to every leaf (non-message) field of a decoded
    rosbags message, for the heuristic fallback / --list-fields discovery."""
    for name in getattr(msg, '__dataclass_fields__', {}):
        value = getattr(msg, name)
        path = f'{prefix}{name}'
        if hasattr(value, '__dataclass_fields__'):
            yield from iter_leaf_field_paths(value, prefix=f'{path}.')
        else:
            yield path


def build_scaler(msgtype, args, seen_report):
    """Return a function(msg) -> msg that scales msg in place, and records
    which field paths it touched (for the --list-fields / summary report)."""
    explicit = EXPLICIT_RULES.get(msgtype)

    def scale(msg):
        touched = seen_report.setdefault(msgtype, set())
        if explicit is not None:
            for path, category in explicit.items():
                if category == 'angvel' and not args.scale_angular_velocity:
                    continue
                factor = args.factor  # vel and accel always share the same factor
                parent, leaf = resolve_field(msg, path)
                setattr(parent, leaf, scale_value(getattr(parent, leaf), factor))
                touched.add(f'{path} ({category})')
        else:
            for path in iter_leaf_field_paths(msg):
                category = classify_by_name(path)
                if category is None:
                    continue
                if category == 'angvel' and not args.scale_angular_velocity:
                    continue
                factor = args.factor  # vel and accel always share the same factor
                parent, leaf = resolve_field(msg, path)
                value = getattr(parent, leaf)
                if not isinstance(value, (int, float)) and not hasattr(value, '__len__'):
                    continue
                setattr(parent, leaf, scale_value(value, factor))
                touched.add(f'{path} ({category}, heuristic)')
        return msg

    return scale


def register_local_msg_types(typestore, repo_root: Path) -> int:
    """Register every <pkg>/msg/<Name>.msg found under repo_root as
    '<pkg>/msg/<Name>' -- covers custom types (ulisse_msgs, px4_msgs, ...)
    for bags old enough to carry no embedded message definitions of their
    own (pre rosbag2 metadata v8-ish). Returns how many were registered."""
    count = 0
    for msg_path in repo_root.glob('vision_dashing_ws/src/*/msg/*.msg'):
        pkg = msg_path.parent.parent.name
        name = msg_path.stem
        msgtype = f'{pkg}/msg/{name}'
        try:
            typestore.register(get_types_from_msg(msg_path.read_text(), msgtype))
            count += 1
        except Exception:
            pass  # malformed/unsupported .msg, or a type this bag never uses
    return count


def convert(input_path: Path, output_path: Path, args) -> None:
    typestore = get_typestore(Stores.ROS2_HUMBLE)
    seen_report = {}
    scaled_count = 0
    total_count = 0

    with Reader(input_path) as reader:
        # Prefer each connection's own embedded message definition (present
        # in bags recorded with a reasonably modern rosbag2 -- same trick as
        # convert_bag_jazzy_to_humble.py). Older bags carry no msgdef at all,
        # so fall back to scanning the repo's own .msg files for whatever
        # that leaves unregistered.
        for conn in reader.connections:
            if conn.msgdef and conn.msgdef.data:
                try:
                    typestore.register(
                        get_types_from_msg(conn.msgdef.data, conn.msgtype)
                    )
                except Exception:
                    pass  # already registered, or a built-in type
        register_local_msg_types(typestore, args.repo_root)

        writer_ctx = None if args.list_fields else Writer(
            output_path, version=HUMBLE_METADATA_VERSION,
            storage_plugin=STORAGE_PLUGINS[args.storage],
        )

        topic_filter = set(args.topics.split(',')) if args.topics else None

        def run(writer):
            nonlocal scaled_count, total_count
            conn_map = {}
            if writer is not None:
                for conn in reader.connections:
                    ext = conn.ext
                    conn_map[conn.id] = writer.add_connection(
                        conn.topic, conn.msgtype, msgdef=conn.msgdef.data,
                        rihs01=conn.digest, serialization_format=ext.serialization_format,
                        offered_qos_profiles=ext.offered_qos_profiles,
                    )

            for conn, timestamp, rawdata in reader.messages():
                total_count += 1
                do_scale = (topic_filter is None or conn.topic in topic_filter) and \
                           (conn.msgtype in EXPLICIT_RULES or args.heuristic_fallback)
                if do_scale:
                    scaler = build_scaler(conn.msgtype, args, seen_report)
                    msg = typestore.deserialize_cdr(rawdata, conn.msgtype)
                    msg = scaler(msg)
                    rawdata = typestore.serialize_cdr(msg, conn.msgtype)
                    scaled_count += 1
                if writer is not None:
                    writer.write(conn_map[conn.id], timestamp, rawdata)

        if writer_ctx is None:
            run(None)
        else:
            with writer_ctx as writer:
                run(writer)

    print(f'{"[dry run] " if args.list_fields else ""}'
          f'{scaled_count}/{total_count} messages scaled'
          + ('' if args.list_fields else f' -> {output_path}'))
    print('\nFields touched per message type:')
    if not seen_report:
        print('  (none -- check --topics filter, or pass --heuristic-fallback '
              'if these types are not in EXPLICIT_RULES)')
    for msgtype, fields in sorted(seen_report.items()):
        print(f'  {msgtype}:')
        for f in sorted(fields):
            print(f'    - {f}')


def main() -> None:
    parser = argparse.ArgumentParser(
        description=__doc__.split('\n\n')[0],
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument('input_bag', type=Path)
    parser.add_argument('output_bag', type=Path, nargs='?',
                         help='required unless --list-fields is given')
    parser.add_argument('--factor', type=float, required=True,
                         help='scale applied to velocity AND acceleration fields '
                              '(always the same value for both -- see module '
                              'docstring for why that is a strict rule, not a '
                              'default)')
    parser.add_argument('--scale-angular-velocity', action='store_true',
                         help='also scale angular velocity / gyro fields. '
                              'Off by default: in this codebase\'s sim plugin, '
                              'the platform\'s actual simulated tilt (rpy) is '
                              'NOT derived from angular velocity, so scaling '
                              'it only changes the synthetic IMU gyro output, '
                              'not the real motion -- read the module '
                              'docstring before turning this on.')
    parser.add_argument('--topics', type=str, default=None,
                         help='comma-separated topic names to restrict scaling to '
                              '(default: all topics whose type is recognized)')
    parser.add_argument('--heuristic-fallback', action='store_true',
                         help='also apply name-based heuristic scaling to message '
                              'types not in EXPLICIT_RULES. Off by default -- '
                              'verify with --list-fields before trusting it.')
    parser.add_argument('--storage', choices=STORAGE_PLUGINS.keys(), default='sqlite3')
    parser.add_argument('--list-fields', action='store_true',
                         help='dry run: report which fields would be scaled '
                              'per message type, write no output bag')
    parser.add_argument('--repo-root', type=Path,
                         default=Path(__file__).resolve().parent.parent,
                         help='repo root to scan for local .msg files, used '
                              'as a fallback for bags with no embedded '
                              'message definitions (default: this script\'s '
                              'repo)')
    args = parser.parse_args()

    if not args.input_bag.exists():
        sys.exit(f'Input bag not found: {args.input_bag}')
    if not args.list_fields:
        if args.output_bag is None:
            sys.exit('output_bag is required unless --list-fields is given')
        if args.output_bag.exists():
            sys.exit(f'Output path already exists, refusing to overwrite: {args.output_bag}')

    convert(args.input_bag, args.output_bag, args)


if __name__ == '__main__':
    main()
