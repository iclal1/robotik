#!/usr/bin/env python3
"""Relay Gazebo joint states to ROS at no more than 50 Hz."""
import signal
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import JointState


PERIOD_NS = 20_000_000


class JointStateRateLimiter(Node):
    def __init__(self):
        super().__init__('joint_state_rate_limiter')
        self.publisher = self.create_publisher(
            JointState, '/joint_states',
            QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE))
        self.subscription = self.create_subscription(
            JointState, '/joint_states_raw', self.forward,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.next_publish_ns = 0

    def forward(self, message):
        now_ns = time.monotonic_ns()
        if now_ns < self.next_publish_ns:
            return
        # Reuse the received message, including its simulation timestamp and
        # every joint field. The steady clock also works across simulation resets.
        self.publisher.publish(message)
        self.next_publish_ns = max(self.next_publish_ns + PERIOD_NS, now_ns)
        if self.next_publish_ns <= now_ns:
            self.next_publish_ns = now_ns + PERIOD_NS


def main():
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = JointStateRateLimiter()
    stopping = False

    def stop(_signal, _frame):
        nonlocal stopping
        stopping = True

    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)
    try:
        while rclpy.ok() and not stopping:
            rclpy.spin_once(node, timeout_sec=0.05)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
