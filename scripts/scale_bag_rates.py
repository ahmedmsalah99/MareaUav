#!/usr/bin/env python3
"""Scale per-axis velocity/rate fields in a recorded platform-motion ROS 2
bag (ulisse_msgs/msg/SimulatedSystem or ulisse_msgs/msg/NavFilterData),
leaving position and orientation untouched.

Scope, deliberately narrow: this script only understands the two message
types the sim/replay pipeline actually uses for platform motion --
ulisse_msgs/msg/SimulatedSystem (played back as /ulisse/simulated_system by
landingplat/plugin.cpp) and ulisse_msgs/msg/NavFilterData (the real sensor's
equivalent, /ulisse/nav_filter/data). Both carry the same six motion fields:

    bodyframe_linear_velocity[0]   x / surge velocity  [m/s]
    bodyframe_linear_velocity[1]   y / sway velocity   [m/s]
    bodyframe_linear_velocity[2]   heave velocity      [m/s]  (see
                                    heave_estimator_node.cpp:46,118, which
                                    reads this exact index as heave)
    bodyframe_angular_velocity[0]  roll rate           [rad/s]
    bodyframe_angular_velocity[1]  pitch rate          [rad/s]  (index order
    bodyframe_angular_velocity[2]  yaw rate            [rad/s]  matches
                                    RPY.msg's roll/pitch/yaw field order,
                                    which plugin.cpp relies on directly)

Each has its own --scale-* option below -- there is no single blanket
--factor, on purpose: scaling e.g. only heave to test a rougher sea state
without touching surge/sway/yaw motion is the actual use case this script
exists for.

Why only these six fields, and nothing else in the message:
  landingplat/plugin.cpp's OnUpdate() dead-reckons BOTH the platform's
  position (from bodyframe_linear_velocity) and its orientation (from
  bodyframe_angular_velocity) -- the message's own absolute position
  (inertialframe_linear_position) and absolute orientation
  (bodyframe_angular_position) are only ever used to seed the very first
  tick, never as a continuous target. So scaling these six velocity/rate
  fields is exactly consistent with how the sim already treats this data --
  Gazebo recomputes position and attitude from whatever you feed it here.
  gyro_bias, inertialframe_water_current, and n_p/n_s (thruster rpm) are
  left alone: none of them is a platform motion rate this script targets.

Strict rule, not a tunable: message timestamps are never touched, only field
values -- so there is no separate "acceleration" scale to offer. If this
script ever needs to scale an acceleration field, physical consistency
requires it use the exact same factor as the velocity it's the derivative
of (dt unchanged -> d(k*v)/dt = k*dv/dt), not an independent value.

Usage:
    pip install rosbags
    ./scale_bag_rates.py INPUT_BAG OUTPUT_BAG --scale-heave-vel 1.5
    ./scale_bag_rates.py INPUT_BAG OUTPUT_BAG --scale-roll-rate 2.0 --scale-pitch-rate 2.0
    ./scale_bag_rates.py INPUT_BAG OUTPUT_BAG --scale-heave-vel 1.5 --list-fields   # dry run, no output bag written
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

MSGTYPES = (
    'ulisse_msgs/msg/SimulatedSystem',
    'ulisse_msgs/msg/NavFilterData',
)

# CLI dest -> (message field name, index within that field's float64[3]).
AXES = {
    'scale_x_vel':    ('bodyframe_linear_velocity',  0),
    'scale_y_vel':    ('bodyframe_linear_velocity',  1),
    'scale_heave_vel':('bodyframe_linear_velocity',  2),
    'scale_roll_rate':('bodyframe_angular_velocity', 0),
    'scale_pitch_rate':('bodyframe_angular_velocity', 1),
    'scale_yaw_rate': ('bodyframe_angular_velocity', 2),
}


def register_local_msg_types(typestore, repo_root: Path) -> int:
    """Register every <pkg>/msg/<Name>.msg found under repo_root as
    '<pkg>/msg/<Name>' -- covers ulisse_msgs for bags old enough to carry no
    embedded message definitions of their own (pre rosbag2 metadata v8-ish).
    Returns how many were registered."""
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


def build_scale_plan(args):
    """Return {dest: (field, index, factor)} for every axis the user asked
    to scale (factor != 1.0), used both to apply the scaling and to report
    what was touched."""
    plan = {}
    for dest, (field, index) in AXES.items():
        factor = getattr(args, dest)
        if factor != 1.0:
            plan[dest] = (field, index, factor)
    return plan


def scale_message(msg, plan, touched: set) -> None:
    # rosbags hands back read-only numpy views into the deserialized buffer
    # for fixed-size float64[N] fields -- copy before mutating.
    touched_fields = {field for field, _, _ in plan.values()}
    copies = {field: getattr(msg, field).copy() for field in touched_fields}
    for dest, (field, index, factor) in plan.items():
        copies[field][index] = copies[field][index] * factor
        touched.add(f'{field}[{index}] ({dest.replace("scale_", "").replace("_", "-")}) x{factor}')
    for field, arr in copies.items():
        setattr(msg, field, arr)


def convert(input_path: Path, output_path: Path, args) -> None:
    typestore = get_typestore(Stores.ROS2_HUMBLE)
    plan = build_scale_plan(args)
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
            if conn.msgtype in MSGTYPES and conn.msgdef and conn.msgdef.data:
                try:
                    typestore.register(get_types_from_msg(conn.msgdef.data, conn.msgtype))
                except Exception:
                    pass  # already registered
        register_local_msg_types(typestore, args.repo_root)

        writer_ctx = None if args.list_fields else Writer(
            output_path, version=HUMBLE_METADATA_VERSION,
            storage_plugin=STORAGE_PLUGINS[args.storage],
        )

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
                if conn.msgtype in MSGTYPES:
                    entry = seen_report.setdefault((conn.topic, conn.msgtype), {'count': 0, 'fields': set()})
                    msg = typestore.deserialize_cdr(rawdata, conn.msgtype)
                    scale_message(msg, plan, entry['fields'])
                    rawdata = typestore.serialize_cdr(msg, conn.msgtype)
                    entry['count'] += 1
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
    print('\nTopics touched (every topic in the bag whose type is '
          f'{" or ".join(MSGTYPES)} -- whether or not any of its fields '
          'actually changed, since a factor of 1.0 for every axis still '
          'means the topic was inspected):')
    if not seen_report:
        print(f'  (none -- the bag has no topic of type {" / ".join(MSGTYPES)})')
    for (topic, msgtype), entry in sorted(seen_report.items()):
        print(f'  {topic}  [{msgtype}]  ({entry["count"]} messages)')
        if entry['fields']:
            for f in sorted(entry['fields']):
                print(f'    - {f}')
        else:
            print('    - (no fields changed -- every --scale-* option was left at its default 1.0)')


def main() -> None:
    parser = argparse.ArgumentParser(
        description=__doc__.split('\n\n')[0],
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument('input_bag', type=Path)
    parser.add_argument('output_bag', type=Path, nargs='?',
                         help='required unless --list-fields is given')

    parser.add_argument('--scale-x-vel', type=float, default=1.0,
                         help='factor for bodyframe_linear_velocity[0] (x/surge velocity, m/s). Default: 1.0 (unchanged)')
    parser.add_argument('--scale-y-vel', type=float, default=1.0,
                         help='factor for bodyframe_linear_velocity[1] (y/sway velocity, m/s). Default: 1.0 (unchanged)')
    parser.add_argument('--scale-heave-vel', type=float, default=1.0,
                         help='factor for bodyframe_linear_velocity[2] (heave velocity, m/s). Default: 1.0 (unchanged)')
    parser.add_argument('--scale-roll-rate', type=float, default=1.0,
                         help='factor for bodyframe_angular_velocity[0] (roll rate, rad/s). Default: 1.0 (unchanged)')
    parser.add_argument('--scale-pitch-rate', type=float, default=1.0,
                         help='factor for bodyframe_angular_velocity[1] (pitch rate, rad/s). Default: 1.0 (unchanged)')
    parser.add_argument('--scale-yaw-rate', type=float, default=1.0,
                         help='factor for bodyframe_angular_velocity[2] (yaw rate, rad/s). Default: 1.0 (unchanged)')

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
    if not build_scale_plan(args):
        sys.exit('No --scale-* option given a value != 1.0 -- nothing to do. '
                  'Pass e.g. --scale-heave-vel 1.5.')

    convert(args.input_bag, args.output_bag, args)


if __name__ == '__main__':
    main()
