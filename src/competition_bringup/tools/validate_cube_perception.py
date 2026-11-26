#!/usr/bin/env python3
"""Test-only comparison against Gazebo poses; never imported by cube_detector."""
import argparse
import json
import math
from pathlib import Path
import re
import subprocess
import sys
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


def parse_pose_info(text):
    poses = {}
    for section in re.split(r'(?:^|\n)pose \{', text)[1:]:
        name = re.search(r'\bname: "([^"]+)"', section)
        position = re.search(r'\bposition \{([^}]*)\}', section, re.S)
        orientation = re.search(r'\borientation \{([^}]*)\}', section, re.S)
        if not name or not position:
            continue

        def value(block, field, default=0.0):
            match = re.search(rf'\b{field}: ([+\-0-9.eE]+)', block or '')
            return float(match.group(1)) if match else default

        p = position.group(1)
        o = orientation.group(1) if orientation else ''
        poses[name.group(1)] = (
            value(p, 'x'), value(p, 'y'), value(p, 'z'),
            value(o, 'x'), value(o, 'y'), value(o, 'z'), value(o, 'w', 1.0))
    return poses


def compare(status, poses):
    robot = poses['pusher_robot']
    yaw = 2.0 * math.atan2(robot[5], robot[6])
    c, s = math.cos(yaw), math.sin(yaw)
    results = []
    for item in status['detections']:
        name = item['color'] + '_cube'
        if name not in poses:
            continue
        cube = poses[name]
        dx, dy = cube[0] - robot[0], cube[1] - robot[1]
        actual_x, actual_y = c*dx + s*dy, -s*dx + c*dy
        actual_range = math.hypot(actual_x, actual_y)
        actual_bearing = math.atan2(actual_y, actual_x)
        result = {'color': item['color'], 'valid': item['valid'],
                  'reason': item.get('reason'),
                  'actual_center_base_m': [actual_x, actual_y],
                  'actual_center_range_m': actual_range,
                  'actual_bearing_rad': actual_bearing,
                  'confidence': item['confidence']}
        if item['valid']:
            est_x, est_y = item['center_position_base_m']
            result.update({
                'estimated_center_base_m': [est_x, est_y],
                'estimated_center_range_m': item['center_range_m'],
                'estimated_bearing_rad': item['bearing_base_rad'],
                'position_error_m': math.hypot(est_x-actual_x, est_y-actual_y),
                'range_error_m': abs(item['center_range_m'] - actual_range),
                'bearing_error_deg': abs(math.degrees(math.atan2(
                    math.sin(item['bearing_base_rad']-actual_bearing),
                    math.cos(item['bearing_base_rad']-actual_bearing)))),
                'reported_position_uncertainty_m': item['position_uncertainty_m'],
            })
        elif 'tentative_center_range_m' in item:
            interval = item['tentative_center_range_bounds_m']
            result.update({
                'tentative_range_m': item['tentative_center_range_m'],
                'tentative_range_bounds_m': interval,
                'tentative_range_error_m': abs(
                    item['tentative_center_range_m'] - actual_range),
                'actual_inside_tentative_bounds':
                    interval[0] <= actual_range <= interval[1],
                'distance_valid': False,
            })
        results.append(result)
    return results


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--label', required=True)
    parser.add_argument('--output', required=True, type=Path)
    args = parser.parse_args()
    rclpy.init()
    node = Node('cube_validation_probe')
    statuses = []
    node.create_subscription(String, '/cube_detector/status',
                             lambda msg: statuses.append(json.loads(msg.data)), 10)
    deadline = time.monotonic() + 5.0
    try:
        while time.monotonic() < deadline and not any(
                s.get('detections') for s in statuses):
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        node.destroy_node()
        rclpy.shutdown()
    if not statuses:
        raise RuntimeError('No cube detector status received')
    status = statuses[-1]
    message = subprocess.run(
        ['ign', 'topic', '-e', '-t', '/world/competition/pose/info', '-n', '1'],
        capture_output=True, text=True, timeout=5, check=True).stdout
    poses = parse_pose_info(message)
    if not all(name in poses for name in
               ('pusher_robot', 'red_cube', 'green_cube', 'blue_cube')):
        raise RuntimeError('Missing Gazebo pose in test-only validation message')
    report = {
        'label': args.label,
        'detector_stamp': status['stamp'],
        'gazebo_robot_pose': poses['pusher_robot'],
        'gazebo_cube_poses': {name: poses[name] for name in
                              ('red_cube', 'green_cube', 'blue_cube')},
        'detections': compare(status, poses),
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open('a') as stream:
        stream.write(json.dumps(report) + '\n')
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    try:
        main()
    except Exception as error:
        print(error, file=sys.stderr)
        sys.exit(1)
