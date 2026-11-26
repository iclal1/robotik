#!/usr/bin/env python3
"""Observe low obstacles and candidate frontiers without moving the robot."""
import json
import math
import signal
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, String
from tf2_ros import Buffer, TransformListener

from exploration_core import frontier_goal, input_failures


class ExplorationDiagnostics(Node):
    def __init__(self):
        super().__init__('exploration_diagnostics')
        self.last_seen = {}
        self.emergency_stop = False
        self.cubes = []
        self.cube_clearance = None
        self.grid = None
        self.tf_buffer = Buffer(cache_time=Duration(seconds=10))
        self.tf_listener = TransformListener(self.tf_buffer, self)
        reliable = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
        map_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(LaserScan, '/scan',
                                 lambda _: self.mark('scan'), qos_profile_sensor_data)
        self.create_subscription(String, '/cube_detector/status', self.on_cube_status,
                                 reliable)
        self.create_subscription(String, '/cube_detector/clearance', self.on_clearance,
                                 reliable)
        self.create_subscription(Odometry, '/odom',
                                 lambda _: self.mark('odom'), reliable)
        self.create_subscription(OccupancyGrid, '/map', self.on_map, map_qos)
        self.create_subscription(Bool, '/emergency_stop', self.on_stop,
                                 reliable)
        self.status = self.create_publisher(String, '/exploration/status', 1)
        self.goal = self.create_publisher(PoseStamped,
                                          '/exploration/candidate_goal', 1)
        self.create_timer(1.0, self.publish_diagnostics)

    def mark(self, name):
        self.last_seen[name] = time.monotonic()

    def on_cube_status(self, message):
        try:
            status = json.loads(message.data)
        except (TypeError, ValueError):
            return
        self.cubes = status.get('detections', [])
        if status.get('perception_state') != 'invalid_perception':
            self.mark('camera')

    def on_clearance(self, message):
        try:
            self.cube_clearance = json.loads(message.data)
        except (TypeError, ValueError):
            self.cube_clearance = None

    def on_map(self, message):
        self.grid = message
        self.mark('map')

    def on_stop(self, message):
        if message.data:
            self.emergency_stop = True  # Latched until this node is restarted.

    def publish_diagnostics(self):
        tf_age = None
        transform = None
        try:
            transform = self.tf_buffer.lookup_transform('map', 'base_footprint', Time())
            tf_age = (self.get_clock().now() - Time.from_msg(transform.header.stamp)).nanoseconds / 1e9
        except Exception:
            pass
        failures = input_failures(self.last_seen, time.monotonic(), tf_age,
                                  self.emergency_stop)
        candidate = None
        unknown_fraction = None
        if self.grid is not None:
            values = np.asarray(self.grid.data, dtype=np.int16)
            unknown_fraction = float(np.count_nonzero(values < 0) / len(values))
        if not failures and self.grid is not None and transform is not None:
            origin = self.grid.info.origin
            q = origin.orientation
            yaw = 2.0 * math.atan2(q.z, q.w)
            candidate = frontier_goal(
                self.grid.data, self.grid.info.width, self.grid.info.height,
                self.grid.info.resolution,
                (origin.position.x, origin.position.y, yaw),
                (transform.transform.translation.x, transform.transform.translation.y))
        if candidate is not None:
            goal = PoseStamped()
            goal.header.stamp = self.get_clock().now().to_msg()
            goal.header.frame_id = 'map'
            goal.pose.position.x = candidate['x']
            goal.pose.position.y = candidate['y']
            goal.pose.orientation.w = 1.0
            self.goal.publish(goal)
        report = {
            'mode': 'diagnostic_only', 'motion_enabled': False,
            'motion_blocker': 'cube_clearance_not_validated',
            'input_failures': failures, 'tf_age_s': tf_age,
            'unknown_fraction': unknown_fraction,
            'cube_candidates': self.cubes, 'cube_clearance': self.cube_clearance,
            'frontier_candidate': candidate,
        }
        self.status.publish(String(data=json.dumps(report, allow_nan=False)))


def main():
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = ExplorationDiagnostics()
    stopping = False

    def stop(_signal, _frame):
        nonlocal stopping
        stopping = True

    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)
    try:
        while rclpy.ok() and not stopping:
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
