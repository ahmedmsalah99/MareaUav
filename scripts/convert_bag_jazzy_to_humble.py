#!/usr/bin/env python3
"""Convert a ROS 2 rosbag2 bag recorded on Jazzy into one Humble can read.

Jazzy's default rosbag2 metadata format (version 9) and the sqlite3 schema
that goes with it are newer than what Humble's rosbag2 understands. Rather
than requiring both distros installed side by side, this uses the `rosbags`
library (pure Python, distro-agnostic — it parses message definitions and
metadata straight out of the bag itself) to re-serialize the bag with a
metadata/schema version Humble supports.

Messages are copied as raw (still-serialized) bytes, not decoded/re-encoded,
so custom message packages (e.g. px4_msgs) don't need to be installed or
registered anywhere — only their message *definitions*, which travel inside
the bag's own connection metadata, are needed.

Usage:
    pip install rosbags
    ./convert_bag_jazzy_to_humble.py INPUT_BAG OUTPUT_BAG [--storage {sqlite3,mcap}]

Requires the `rosbags` package (tested with 0.11.x). If Humble still fails to
open the result, try lowering the storage plugin to sqlite3 (the default) —
mcap support has to be installed separately on the Humble side
(`ros-humble-rosbag2-storage-mcap`).
"""

import argparse
import sys
from pathlib import Path

try:
    from rosbags.rosbag2 import Reader, StoragePlugin, Writer
except ImportError:
    sys.exit(
        "The 'rosbags' package is required.\n"
        "Install it with:  pip install rosbags"
    )

# Humble's rosbag2 understands metadata format version 8 fields (it does not
# know about version 9's additions from Jazzy), so re-writing at version 8 is
# the compatible target.
HUMBLE_METADATA_VERSION = 8

STORAGE_PLUGINS = {
    'sqlite3': StoragePlugin.SQLITE3,
    'mcap': StoragePlugin.MCAP,
}


def convert(input_path: Path, output_path: Path, storage: str) -> None:
    with Reader(input_path) as reader:
        with Writer(
            output_path,
            version=HUMBLE_METADATA_VERSION,
            storage_plugin=STORAGE_PLUGINS[storage],
        ) as writer:
            # Map each input connection to its freshly created output
            # connection (Writer assigns its own ids).
            conn_map = {}
            for conn in reader.connections:
                ext = conn.ext
                conn_map[conn.id] = writer.add_connection(
                    conn.topic,
                    conn.msgtype,
                    msgdef=conn.msgdef.data,
                    rihs01=conn.digest,
                    serialization_format=ext.serialization_format,
                    offered_qos_profiles=ext.offered_qos_profiles,
                )

            count = 0
            for conn, timestamp, rawdata in reader.messages():
                writer.write(conn_map[conn.id], timestamp, rawdata)
                count += 1
                if count % 5000 == 0:
                    print(f'  ... {count} messages written')

    print(f'Done: {count} messages, {len(conn_map)} topics -> {output_path}')


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    parser.add_argument('input_bag', type=Path, help='Path to the Jazzy-recorded bag directory')
    parser.add_argument('output_bag', type=Path, help='Path to write the Humble-compatible bag to (must not exist)')
    parser.add_argument(
        '--storage', choices=STORAGE_PLUGINS.keys(), default='sqlite3',
        help='Output storage plugin (default: sqlite3, always available on Humble)',
    )
    args = parser.parse_args()

    if not args.input_bag.exists():
        sys.exit(f'Input bag not found: {args.input_bag}')
    if args.output_bag.exists():
        sys.exit(f'Output path already exists, refusing to overwrite: {args.output_bag}')

    convert(args.input_bag, args.output_bag, args.storage)


if __name__ == '__main__':
    main()
