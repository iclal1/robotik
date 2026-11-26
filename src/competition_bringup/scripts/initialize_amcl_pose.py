#!/usr/bin/env python3
"""Seed AMCL once at the known saved-map start with explicit uncertainty."""
import math
import time

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from lifecycle_msgs.srv import GetState
from nav2_msgs.srv import SetInitialPose
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


class Initializer(Node):
    def __init__(self):
        super().__init__('amcl_start_initializer')
        self.declare_parameter('initial_x', 0.0)
        self.declare_parameter('initial_y', 0.0)
        self.declare_parameter('initial_yaw', 0.0)
        # These bound start uncertainty; they are not claims of measured accuracy.
        self.declare_parameter('std_xy', 0.10)
        self.declare_parameter('std_yaw', 0.17)
        self.scan_stamp = None
        self.odom_stamp = None
        self.map_ready = False
        self.pose = None
        self.create_subscription(LaserScan, '/scan',
                                 lambda m: setattr(self, 'scan_stamp', m.header.stamp),
                                 qos_profile_sensor_data)
        self.create_subscription(Odometry, '/odom',
                                 lambda m: setattr(self, 'odom_stamp', m.header.stamp), 10)
        self.create_subscription(OccupancyGrid, '/map', self.map_callback,
                                 QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose',
                                 lambda m: setattr(self, 'pose', m), 10)
        self.state_client = self.create_client(GetState, '/amcl/get_state')
        self.pose_client = self.create_client(SetInitialPose, '/set_initial_pose')

    def map_callback(self, msg):
        self.map_ready = (msg.header.frame_id == 'map' and
                          msg.info.width > 0 and msg.info.height > 0 and
                          msg.info.resolution > 0 and
                          len(msg.data) == msg.info.width * msg.info.height)

    def ready(self):
        now = self.get_clock().now().nanoseconds / 1e9
        if now <= 0 or not self.map_ready:
            return False
        for stamp in (self.scan_stamp, self.odom_stamp):
            if stamp is None:
                return False
            age = now - (stamp.sec + stamp.nanosec / 1e9)
            if not -0.2 <= age <= 1.0:
                return False
        return True

    def wait(self, predicate, seconds):
        deadline = time.monotonic() + seconds
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if predicate():
                return True
        return False

    def run(self):
        if not self.wait(lambda: self.ready() and
                         self.state_client.service_is_ready() and
                         self.pose_client.service_is_ready(), 40.0):
            raise RuntimeError('AMCL, saved map, scan, odometry, or simulation clock not ready')
        state = self.state_client.call_async(GetState.Request())
        if not self.wait(state.done, 5.0) or state.result().current_state.label != 'active':
            raise RuntimeError('AMCL did not reach active lifecycle state')

        request = SetInitialPose.Request()
        pose = request.pose
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.pose.position.x = self.get_parameter('initial_x').value
        pose.pose.pose.position.y = self.get_parameter('initial_y').value
        yaw = self.get_parameter('initial_yaw').value
        pose.pose.pose.orientation.z = math.sin(yaw / 2)
        pose.pose.pose.orientation.w = math.cos(yaw / 2)
        xy_std = self.get_parameter('std_xy').value
        yaw_std = self.get_parameter('std_yaw').value
        if not all(math.isfinite(value) and value > 0 for value in (xy_std, yaw_std)):
            raise RuntimeError('AMCL start uncertainty must be finite and positive')
        xy_var = xy_std ** 2
        yaw_var = yaw_std ** 2
        pose.pose.covariance[0] = xy_var
        pose.pose.covariance[7] = xy_var
        pose.pose.covariance[35] = yaw_var
        self.pose = None
        response = self.pose_client.call_async(request)
        if not self.wait(response.done, 5.0):
            raise RuntimeError('AMCL did not acknowledge initial pose')
        if not self.wait(lambda: self.pose is not None and
                         all(math.isfinite(self.pose.pose.covariance[i]) and
                             self.pose.pose.covariance[i] > 1e-6
                             for i in (0, 7, 35)), 12.0):
            raise RuntimeError('AMCL did not publish a nondegenerate pose estimate')
        self.get_logger().info('AMCL initialized at saved-map start with finite covariance')


def main():
    rclpy.init()
    node = Initializer()
    try:
        node.run()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
