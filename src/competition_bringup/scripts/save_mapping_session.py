#!/usr/bin/env python3
"""Save a uniquely named preliminary SLAM map in the workspace."""
import json
from datetime import datetime, timezone
from pathlib import Path
import shutil
import subprocess
import sys
import time
from uuid import uuid4

from ament_index_python.packages import get_package_share_directory
import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy


def main():
    workspace = Path.cwd()
    if not (workspace / 'src/competition_bringup').is_dir():
        raise SystemExit('Run this command from the robotics_competition_ws root')
    rclpy.init()
    node = Node('preliminary_map_snapshot')
    received = []
    node.create_subscription(
        OccupancyGrid, '/map', received.append,
        QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                   durability=DurabilityPolicy.TRANSIENT_LOCAL))
    deadline = time.monotonic() + 10.0
    try:
        while not received and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.2)
    finally:
        node.destroy_node()
        rclpy.shutdown()
    if not received:
        raise SystemExit('No /map received; nothing was saved')

    map_msg = received[0]
    session_id = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ') + '_' + uuid4().hex[:8]
    directory = workspace / 'maps' / 'preliminary' / session_id
    directory.mkdir(parents=True, exist_ok=False)
    prefix = directory / 'map'
    result = subprocess.run(
        ['ros2', 'run', 'nav2_map_server', 'map_saver_cli',
         '-t', '/map', '-f', str(prefix)],
        capture_output=True, text=True, timeout=30, check=False)
    if result.returncode != 0 or not prefix.with_suffix('.pgm').exists() or not prefix.with_suffix('.yaml').exists():
        raise RuntimeError('Map saver failed; inspect ' + str(directory) + ': ' + result.stderr)
    source_config = Path(get_package_share_directory('competition_bringup')) / 'config/slam_mapping.yaml'
    shutil.copy2(source_config, directory / 'slam_mapping.yaml')
    cells = map_msg.data
    metadata = {
        'classification': 'preliminary_stationary',
        'autonomous_motion_enabled': False,
        'map_frame': map_msg.header.frame_id,
        'map_stamp': {'sec': map_msg.header.stamp.sec,
                      'nanosec': map_msg.header.stamp.nanosec},
        'resolution_m': map_msg.info.resolution,
        'width_cells': map_msg.info.width,
        'height_cells': map_msg.info.height,
        'origin': {'x': map_msg.info.origin.position.x,
                   'y': map_msg.info.origin.position.y},
        'unknown_cells': sum(v < 0 for v in cells),
        'free_cells': sum(0 <= v <= 20 for v in cells),
        'occupied_cells': sum(v > 50 for v in cells),
        'coverage_note': 'Known fraction of current grid extent, not validated arena coverage',
        'exploration_results': 'No autonomous motion or cube displacement test performed',
        'files': ['map.pgm', 'map.yaml', 'slam_mapping.yaml'],
    }
    (directory / 'session.json').write_text(json.dumps(metadata, indent=2) + '\n')
    print(directory)


if __name__ == '__main__':
    try:
        main()
    except Exception as error:
        print(error, file=sys.stderr)
        sys.exit(1)
