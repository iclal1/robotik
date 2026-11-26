#!/usr/bin/env python3
"""Test-only fixed-pose Gazebo camera capture; always restores robot pose.

This changes only the running simulation entity pose. It sends no velocity
commands, does not edit the official world, and is never imported by the
production detector. Run with exactly one competition simulation active.
"""
import argparse
import json
import math
from pathlib import Path
import subprocess
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String

from validate_cube_perception import parse_pose_info


SERVICE = '/world/competition/set_pose'


def pose_info():
    result = subprocess.run(
        ['ign', 'topic', '-e', '-t', '/world/competition/pose/info', '-n', '1'],
        capture_output=True, text=True, timeout=5, check=True)
    return parse_pose_info(result.stdout)


def set_pose(x, y, z, qx, qy, qz, qw):
    request = (f'name: "pusher_robot" position {{x: {x} y: {y} z: {z}}} '
               f'orientation {{x: {qx} y: {qy} z: {qz} w: {qw}}}')
    result = subprocess.run(
        ['ign', 'service', '-s', SERVICE, '--reqtype', 'ignition.msgs.Pose',
         '--reptype', 'ignition.msgs.Boolean', '--timeout', '3000', '-r', request],
        capture_output=True, text=True, timeout=5, check=True)
    if 'data: true' not in result.stdout:
        raise RuntimeError(f'Gazebo set_pose failed: {result.stdout}')


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', required=True, type=Path)
    args = parser.parse_args()
    original = pose_info()['pusher_robot']
    args.output.mkdir(parents=True, exist_ok=True)
    rclpy.init()
    node = Node('cube_angle_capture')
    latest = {'image': None, 'status': None}

    def on_image(message):
        pixels = np.frombuffer(message.data, np.uint8)
        array = pixels.reshape(message.height, message.step)[:, :message.width*3]
        array = array.reshape(message.height, message.width, 3)
        latest['image'] = (cv2.cvtColor(array, cv2.COLOR_RGB2BGR)
                           if message.encoding == 'rgb8' else array.copy())

    node.create_subscription(Image, '/camera/image_raw', on_image, 10)
    node.create_subscription(String, '/cube_detector/status',
                             lambda message: latest.update(
                                 status=json.loads(message.data)), 10)
    cases = [(-.25, 'blue_right_edge'), (0., 'blue_frontal'),
             (.4, 'blue_green_oblique'), (.9, 'multiple_oblique'),
             (1.3, 'red_green_oblique')]
    rows = []
    try:
        for yaw, label in cases:
            set_pose(original[0], original[1], original[2],
                     0, 0, math.sin(yaw/2), math.cos(yaw/2))
            latest['image'] = latest['status'] = None
            deadline = time.monotonic() + 2.0
            while time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=.1)
            if latest['image'] is None or latest['status'] is None:
                raise RuntimeError(f'No camera/status for {label}')
            image_path = args.output / f'{label}.png'
            cv2.imwrite(str(image_path), latest['image'])
            rows.append({'label': label, 'yaw_rad': yaw,
                         'image': str(image_path),
                         'gazebo_poses': pose_info(),
                         'status': latest['status']})
            print(label, [d['color'] for d in latest['status']['detections']],
                  flush=True)
    finally:
        set_pose(*original)
        node.destroy_node()
        rclpy.shutdown()
        (args.output / 'manifest.json').write_text(json.dumps(rows, indent=2))


if __name__ == '__main__':
    main()
