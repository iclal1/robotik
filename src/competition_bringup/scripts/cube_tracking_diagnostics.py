#!/usr/bin/env python3
"""Read-only status for navigation cube tracks and camera evidence."""
import json

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from cube_tracks import track_state


class CubeTrackingDiagnostics(Node):
    def __init__(self):
        super().__init__('cube_tracking_diagnostics')
        self.safety = None
        self.camera = None
        self.create_subscription(String, '/nav2/safety_status',
                                 self.on_safety, 10)
        self.create_subscription(String, '/cube_detector/status',
                                 self.on_camera, 10)
        self.publisher = self.create_publisher(
            String, '/nav2/cube_tracking_diagnostics', 10)
        self.create_timer(.5, self.publish_status)

    def on_safety(self, msg):
        try:
            self.safety = json.loads(msg.data)
        except ValueError:
            self.safety = None

    def on_camera(self, msg):
        try:
            self.camera = json.loads(msg.data)
        except ValueError:
            self.camera = None

    def publish_status(self):
        if self.safety is None:
            return
        camera = self.camera or {}
        stamp = camera.get('stamp') or {}
        camera_time = stamp.get('sec', 0) + stamp.get('nanosec', 0)*1e-9
        current_time = self.get_clock().now().nanoseconds*1e-9
        camera_current = (camera_time > 0 and
                          -.1 <= current_time-camera_time <= .6)
        detections = {
            item.get('color'): item for item in camera.get('detections', [])
            if item.get('valid') and item.get('confirmed')
        } if camera_current else {}
        tracks = {}
        for color, item in self.safety.get('cube_obstacles', {}).items():
            detection = detections.get(color)
            age = item['age_s']
            state = track_state(age, item['confirmed'],
                                bool(detection and detection.get('partial_view')),
                                detection is None, item['inconsistent'])
            tracks[color] = {'state': state, 'age_s': age,
                             'uncertainty_m': item['position_uncertainty_m'],
                             'odom_xy': item['odom_xy'],
                             'fresh_camera_measurement': detection is not None,
                             'camera_partial_view': bool(
                                 detection and detection.get('partial_view')),
                             'track_id': None if detection is None else
                                 detection.get('track_id')}
        status = {'camera_current': camera_current,
                  'camera_perception_state': camera.get('perception_state'),
                  'tracks': tracks,
                  'motion_requested': self.safety['motion_requested']}
        self.publisher.publish(String(data=json.dumps(status, allow_nan=False)))


def main():
    rclpy.init()
    node = CubeTrackingDiagnostics()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
