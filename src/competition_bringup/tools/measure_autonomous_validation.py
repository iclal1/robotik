#!/usr/bin/env python3
"""Test-only measurement of the gated exploration launch and Gazebo poses."""
import argparse
import json
from pathlib import Path
import subprocess
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid, Odometry, Path as NavPath
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, QoSProfile, ReliabilityPolicy,
                       qos_profile_sensor_data)
from sensor_msgs.msg import Image, JointState, LaserScan
from std_msgs.msg import String

from validate_cube_perception import parse_pose_info


def gazebo_poses():
    output = subprocess.run(
        ['ign', 'topic', '-e', '-t', '/world/competition/pose/info', '-n', '1'],
        capture_output=True, text=True, timeout=5, check=True).stdout
    return parse_pose_info(output)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--seconds', type=float, default=6.)
    args = parser.parse_args()
    before = gazebo_poses()
    rclpy.init()
    node = Node('autonomous_validation_probe')
    samples = {name: [] for name in
               ('camera', 'scan', 'odom', 'joints', 'map', 'cmd', 'path', 'status')}
    node.create_subscription(Image, '/camera/image_raw',
                             lambda _: samples['camera'].append(time.monotonic()), 10)
    node.create_subscription(LaserScan, '/scan',
                             lambda _: samples['scan'].append(time.monotonic()),
                             qos_profile_sensor_data)
    node.create_subscription(Odometry, '/odom',
                             lambda _: samples['odom'].append(time.monotonic()), 10)
    node.create_subscription(JointState, '/joint_states',
                             lambda _: samples['joints'].append(time.monotonic()), 10)
    node.create_subscription(OccupancyGrid, '/map',
                             lambda m: samples['map'].append(m),
                             QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                        durability=DurabilityPolicy.TRANSIENT_LOCAL))
    node.create_subscription(Twist, '/cmd_vel',
                             lambda m: samples['cmd'].append((m.linear.x, m.angular.z)), 10)
    node.create_subscription(NavPath, '/autonomous_exploration/path',
                             lambda m: samples['path'].append(len(m.poses)),
                             QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                        durability=DurabilityPolicy.TRANSIENT_LOCAL))
    node.create_subscription(String, '/autonomous_exploration/status',
                             lambda m: samples['status'].append(json.loads(m.data)), 10)
    deadline = time.monotonic()+args.seconds
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=.1)
    node.destroy_node()
    rclpy.shutdown()
    after = gazebo_poses()

    def rate(values):
        return (len(values)-1)/(values[-1]-values[0]) if len(values) > 1 else None

    import math
    names = ('pusher_robot', 'red_cube', 'green_cube', 'blue_cube')
    movement = {name: math.dist(before[name][:2], after[name][:2]) for name in names}
    statuses = samples['status']
    map_msg = samples['map'][-1] if samples['map'] else None
    report = {
        'duration_s': args.seconds,
        'rates_hz': {name: rate(samples[name]) for name in
                     ('camera', 'scan', 'odom', 'joints')},
        'cmd_vel_messages': samples['cmd'],
        'path_lengths': samples['path'],
        'status_samples': len(statuses),
        'latest_status': statuses[-1] if statuses else None,
        'mean_planning_ms': (sum(s['planning_ms'] for s in statuses
                                 if s['planning_ms'] is not None)/len(statuses)
                              if statuses else None),
        'map': None if map_msg is None else {
            'width': map_msg.info.width, 'height': map_msg.info.height,
            'resolution_m': map_msg.info.resolution,
            'known_fraction': sum(x >= 0 for x in map_msg.data)/len(map_msg.data)},
        'gazebo_xy_displacement_m': movement,
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2)+'\n')
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
