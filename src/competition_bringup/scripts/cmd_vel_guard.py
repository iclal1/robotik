#!/usr/bin/env python3
"""The only production path from /cmd_vel to the Gazebo drive command."""
import math
import signal
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import Bool


class CmdVelGuard(Node):
    def __init__(self):
        super().__init__('cmd_vel_guard')
        self.declare_parameter('timeout_sec', 0.30)
        self.timeout_sec = float(self.get_parameter('timeout_sec').value)
        if not 0.05 <= self.timeout_sec <= 2.0:
            raise ValueError('timeout_sec must be between 0.05 and 2.0')
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
        self.publisher = self.create_publisher(Twist, '/model/pusher_robot/cmd_vel', qos)
        self.subscription = self.create_subscription(Twist, '/cmd_vel', self.on_command, qos)
        self.stop_subscription = self.create_subscription(
            Bool, '/emergency_stop', self.on_emergency_stop, qos)
        self.emergency_stop = False
        self.last_command = Twist()
        self.last_received = None
        self.timed_out = False
        self.timer = self.create_timer(1.0 / 30.0, self.send,
                                       clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.publisher.publish(Twist())

    def on_command(self, command):
        if self.emergency_stop:
            self.publisher.publish(Twist())
            return
        values = (command.linear.x, command.linear.y, command.linear.z,
                  command.angular.x, command.angular.y, command.angular.z)
        if not all(math.isfinite(value) for value in values) or any(
            abs(value) > 1e-9 for value in (command.linear.y, command.linear.z,
                                           command.angular.x, command.angular.y)
        ):
            self.last_command = Twist()
            self.last_received = None
            self.get_logger().warn('Rejected non-finite or non-planar cmd_vel')
            self.publisher.publish(Twist())
            return
        safe = Twist()
        safe.linear.x = max(-0.8, min(0.8, command.linear.x))
        safe.angular.z = max(-1.5, min(1.5, command.angular.z))
        self.last_command = safe
        self.last_received = time.monotonic()
        self.timed_out = False
        self.publisher.publish(safe)

    def on_emergency_stop(self, stop):
        if stop.data:
            self.emergency_stop = True  # Only a guard restart can clear the stop.
            self.last_command = Twist()
            self.last_received = None
            self.publisher.publish(Twist())

    def send(self):
        if self.emergency_stop or self.last_received is None:
            self.publisher.publish(Twist())
        elif time.monotonic() - self.last_received > self.timeout_sec:
            self.last_command = Twist()
            self.last_received = None
            self.timed_out = True
            self.publisher.publish(Twist())
        else:
            self.publisher.publish(self.last_command)


def main():
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = CmdVelGuard()
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
        if rclpy.ok():
            for _ in range(3):
                node.publisher.publish(Twist())
                time.sleep(0.02)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
