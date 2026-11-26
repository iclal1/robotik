#!/usr/bin/env python3
"""Convert DiffDrive axle odometry into chassis-center base_footprint odometry."""
import math
import signal

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from tf2_ros import TransformBroadcaster


AXLE_TO_BASE_X = 0.05  # Drive axle is 0.05 m behind the chassis center.


class OdometryCorrector(Node):
    def __init__(self):
        super().__init__('odometry_corrector')
        self.publisher = self.create_publisher(Odometry, '/odom', 10)
        self.broadcaster = TransformBroadcaster(self)
        self.create_subscription(Odometry, '/wheel_odom_raw', self.correct, 10)

    def correct(self, raw):
        q = raw.pose.pose.orientation
        norm = math.hypot(q.z, q.w)
        if norm < 1e-9:
            return
        yaw = 2.0*math.atan2(q.z, q.w)
        corrected = Odometry()
        corrected.header = raw.header
        corrected.header.frame_id = 'odom'
        corrected.child_frame_id = 'base_footprint'
        corrected.pose = raw.pose
        corrected.pose.pose.position.x += AXLE_TO_BASE_X*(math.cos(yaw)-1.0)
        corrected.pose.pose.position.y += AXLE_TO_BASE_X*math.sin(yaw)
        corrected.twist = raw.twist
        corrected.twist.twist.linear.y += AXLE_TO_BASE_X*raw.twist.twist.angular.z
        # Conservative planar starting uncertainties from Phase 2 engineering runs.
        # These are placeholders for later empirical calibration, not perfect confidence.
        pose_variance = [0.0]*36
        twist_variance = [0.0]*36
        for index in (14, 21, 28):
            pose_variance[index] = twist_variance[index] = 1000.0
        pose_variance[0] = pose_variance[7] = 0.03**2
        pose_variance[35] = 0.05**2
        twist_variance[0] = twist_variance[7] = 0.02**2
        twist_variance[35] = 0.03**2
        corrected.pose.covariance = pose_variance
        corrected.twist.covariance = twist_variance
        self.publisher.publish(corrected)

        transform = TransformStamped()
        transform.header = corrected.header
        transform.child_frame_id = 'base_footprint'
        transform.transform.translation.x = corrected.pose.pose.position.x
        transform.transform.translation.y = corrected.pose.pose.position.y
        transform.transform.translation.z = corrected.pose.pose.position.z
        transform.transform.rotation = corrected.pose.pose.orientation
        self.broadcaster.sendTransform(transform)


def main():
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = OdometryCorrector()
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
