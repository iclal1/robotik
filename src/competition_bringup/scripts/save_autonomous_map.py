#!/usr/bin/env python3
"""Save a unique exploration map snapshot without touching earlier maps."""
import argparse
from datetime import datetime, timezone
import json
from pathlib import Path
import shutil
import subprocess
import time
from uuid import uuid4

import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String


WORKSPACE = Path(__file__).resolve().parents[3]


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--label', default='manual_snapshot')
    args = parser.parse_args()
    if not (WORKSPACE / 'src/competition_bringup').is_dir():
        raise RuntimeError('Workspace root could not be identified')
    rclpy.init()
    node = Node('autonomous_map_snapshot')
    received = {'map': None, 'status': None}
    node.create_subscription(OccupancyGrid, '/map',
        lambda msg: received.update(map=msg),
        QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                   durability=DurabilityPolicy.TRANSIENT_LOCAL))
    node.create_subscription(String, '/autonomous_exploration/status',
        lambda msg: received.update(status=json.loads(msg.data)), 1)
    deadline = time.monotonic()+10
    while received['map'] is None and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=.2)
    node.destroy_node()
    rclpy.shutdown()
    grid = received['map']
    if grid is None:
        raise RuntimeError('No /map received; no snapshot created')
    session = (datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ') +
               '_' + uuid4().hex[:8])
    directory = WORKSPACE / 'maps/autonomous_exploration' / session
    directory.mkdir(parents=True, exist_ok=False)
    prefix = directory / 'map'
    result = subprocess.run(
        ['ros2', 'run', 'nav2_map_server', 'map_saver_cli',
         '-t', '/map', '-f', str(prefix)],
        capture_output=True, text=True, timeout=30)
    if result.returncode or not prefix.with_suffix('.pgm').is_file() or not prefix.with_suffix('.yaml').is_file():
        raise RuntimeError('Map saver failed: ' + result.stderr)
    values = np.asarray(grid.data, np.int8).reshape(grid.info.height,
                                                    grid.info.width)
    np.save(directory / 'occupancy.npy', values)
    shutil.copy2(WORKSPACE / 'src/competition_bringup/config/slam_mapping.yaml',
                 directory / 'slam_mapping.yaml')
    summary = {'label': args.label, 'classification': 'autonomous_exploration_snapshot',
               'map_frame': grid.header.frame_id,
               'map_stamp': {'sec': grid.header.stamp.sec,
                             'nanosec': grid.header.stamp.nanosec},
               'resolution_m': grid.info.resolution,
               'width_cells': grid.info.width,
               'height_cells': grid.info.height,
               'origin': {'x': grid.info.origin.position.x,
                          'y': grid.info.origin.position.y},
               'known_cells': int(np.count_nonzero(values >= 0)),
               'unknown_cells': int(np.count_nonzero(values < 0)),
               'free_cells': int(np.count_nonzero((values >= 0) & (values <= 20))),
               'occupied_cells': int(np.count_nonzero(values > 50)),
               'exploration_status': received['status'],
               'coverage_note': 'Known fraction of map extent, not arena coverage',
               'files': ['map.pgm', 'map.yaml', 'occupancy.npy', 'slam_mapping.yaml']}
    (directory / 'session.json').write_text(json.dumps(summary, indent=2)+'\n')
    print(directory)


if __name__ == '__main__':
    main()
