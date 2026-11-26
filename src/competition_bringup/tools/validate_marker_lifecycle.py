#!/usr/bin/env python3
"""Test-only short/long visual loss trial in one running Gazebo instance."""
import argparse
import json
import math
from pathlib import Path
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray

from capture_cube_angles import pose_info, set_pose


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', required=True, type=Path)
    args = parser.parse_args()
    original = pose_info()['pusher_robot']
    rclpy.init()
    node = Node('cube_marker_lifecycle_probe')
    events = []
    statuses = []
    clearances = []
    phase = ['initial']

    def on_markers(message):
        for marker in message.markers:
            events.append({'phase': phase[0], 'action': marker.action,
                           'id': marker.id, 'type': marker.type,
                           'text': marker.text,
                           'alpha': marker.color.a,
                           'stamp': marker.header.stamp.sec +
                                    marker.header.stamp.nanosec*1e-9})

    node.create_subscription(MarkerArray, '/cube_detector/markers', on_markers, 10)
    node.create_subscription(String, '/cube_detector/status',
                             lambda message: statuses.append(
                                 (phase[0], json.loads(message.data))), 10)
    node.create_subscription(String, '/cube_detector/clearance',
                             lambda message: clearances.append(
                                 (phase[0], json.loads(message.data))), 10)

    def collect(seconds, name):
        phase[0] = name
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.05)

    try:
        collect(.7, 'initial')
        set_pose(original[0], original[1], original[2],
                 0, 0, math.sin(-.5), math.cos(-.5))
        collect(.35, 'short_loss')
        set_pose(*original)
        collect(.7, 'recovery')
        set_pose(original[0], original[1], original[2],
                 0, 0, math.sin(-.5), math.cos(-.5))
        collect(1.65, 'long_loss')
        set_pose(*original)
        collect(.7, 'new_track')
    finally:
        set_pose(*original)
        node.destroy_node()
        rclpy.shutdown()

    summary = {}
    for name in ('initial', 'short_loss', 'recovery', 'long_loss', 'new_track'):
        subset = [e for e in events if e['phase'] == name]
        matching = [s for p, s in statuses if p == name]
        summary[name] = {
            'adds': sum(e['action'] == Marker.ADD for e in subset),
            'deletes': sum(e['action'] == Marker.DELETE for e in subset),
            'deleteall': sum(e['action'] == Marker.DELETEALL for e in subset),
            'ids': sorted({e['id'] for e in subset if e['action'] == Marker.ADD}),
            'labels': sorted({e['text'] for e in subset if e['text']})[:12],
            'confirmed_frames': sum(s['perception_state'] ==
                                    'confirmed_visible_obstacle' for s in matching),
            'unknown_frames': sum(s['perception_state'] ==
                                  'unknown_or_unobserved' for s in matching),
            'any_safe_to_move': any(c['safe_to_move'] for p, c in clearances
                                    if p == name),
        }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(summary, indent=2))
    print(json.dumps(summary, indent=2))


if __name__ == '__main__':
    main()
