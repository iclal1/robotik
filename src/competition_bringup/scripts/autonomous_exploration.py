#!/usr/bin/env python3
"""Frontier planner with a separate, fail-closed navigation safety layer.

The node never reads Gazebo truth. It publishes /cmd_vel only when explicitly
enabled by the new launch's validation gate and every runtime check passes.
"""
import json
import math
import signal
import time

from builtin_interfaces.msg import Time as TimeMsg

import numpy as np
import rclpy
from geometry_msgs.msg import Point, PoseStamped, Twist
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, QoSProfile, ReliabilityPolicy,
                       qos_profile_sensor_data)
from rclpy.signals import SignalHandlerOptions
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, String
from tf2_ros import Buffer, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

from autonomous_core import (ROBOT_SWEEP_RADIUS_M, astar, control_command,
                             frontier_groups, make_grid, motion_segment_clear,
                             path_clear, rate_limit_command, select_frontier)
from motion_safety import (INITIAL_CUBE_ZONES, START_WORLD_XY, ZONE_UNCERTAINTY_M,
                           camera_candidate_blocks, first_unsafe_path_point,
                           stopping_distance,
                           swept_clearance)
from cube_tracks import CubeTracks


def yaw_of(quaternion):
    return math.atan2(2*(quaternion.w*quaternion.z + quaternion.x*quaternion.y),
                      1-2*(quaternion.y**2 + quaternion.z**2))


def transform_xy(x, y, transform):
    yaw = yaw_of(transform.transform.rotation)
    c, s = math.cos(yaw), math.sin(yaw)
    return (transform.transform.translation.x + c*x - s*y,
            transform.transform.translation.y + s*x + c*y)


class AutonomousExploration(Node):
    def __init__(self):
        super().__init__('autonomous_exploration')
        self.declare_parameter('enable_motion', False)
        self.declare_parameter('max_linear_mps', .10)
        self.declare_parameter('max_angular_radps', .25)
        self.declare_parameter('validation_mode', 'none')
        self.motion_requested = bool(self.get_parameter('enable_motion').value)
        self.max_linear = float(self.get_parameter('max_linear_mps').value)
        self.max_angular = float(self.get_parameter('max_angular_radps').value)
        self.validation_mode = str(self.get_parameter('validation_mode').value)
        if self.validation_mode not in ('none', 'rotation', 'translation', 'goal'):
            raise ValueError('Unknown validation_mode')
        if not 0 < self.max_linear <= .10 or not 0 < self.max_angular <= .25:
            raise ValueError('Exploration speed parameters exceed validated design range')
        self.map_msg = None
        self.scan = None
        self.odom = None
        self.cube_status = None
        self.cube_clearance = None
        self.tracker = CubeTracks()
        self.cubes = self.tracker.tracks
        self.seen_wall = {}
        self.last_stamps = {}
        self.emergency_latched = False
        self.time_reset_latched = False
        self.goal = None
        self.path = []
        self.blacklist = set()
        self.failed_goals = 0
        self.replans = 0
        self.last_plan = 0.
        self.last_plan_ms = None
        self.last_blocker = 'motion_opt_in_required'
        self.last_command = (0., 0.)
        self.last_control_wall = time.monotonic()
        self.visual_goal_active = False
        self.validation_start = None
        self.validation_done = False
        self.minimum_scan_clearance = None
        self.motion_ever_commanded = False
        self.safety_halt_latched = False
        self.last_path_unsafe_xy = None
        self.tf_buffer = Buffer(cache_time=Duration(seconds=10))
        self.tf_listener = TransformListener(self.tf_buffer, self)
        reliable = QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE)
        map_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(OccupancyGrid, '/map', self.on_map, map_qos)
        self.create_subscription(LaserScan, '/scan', self.on_scan,
                                 qos_profile_sensor_data)
        self.create_subscription(Odometry, '/odom', self.on_odom, reliable)
        self.create_subscription(String, '/cube_detector/status',
                                 self.on_cube_status, reliable)
        self.create_subscription(String, '/cube_detector/clearance',
                                 self.on_clearance, reliable)
        self.create_subscription(Bool, '/emergency_stop', self.on_stop, reliable)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 1)
        self.status_pub = self.create_publisher(String,
            '/autonomous_exploration/status', 1)
        visual_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.path_pub = self.create_publisher(Path,
            '/autonomous_exploration/path', visual_qos)
        self.goal_pub = self.create_publisher(PoseStamped,
            '/autonomous_exploration/goal', visual_qos)
        self.markers_pub = self.create_publisher(MarkerArray,
            '/autonomous_exploration/markers', visual_qos)
        self.create_timer(.1, self.control_tick)
        self.create_timer(1., self.plan_tick)

    def mark(self, name, stamp):
        value = stamp.sec + stamp.nanosec*1e-9
        if value <= 0:
            return
        previous = self.last_stamps.get(name)
        if previous is not None and value < previous - .10:
            self.time_reset_latched = True
        if previous is not None and value <= previous:
            return
        self.last_stamps[name] = value
        self.seen_wall[name] = time.monotonic()

    def on_map(self, msg):
        self.map_msg = msg
        self.mark('map', msg.header.stamp)

    def on_scan(self, msg):
        self.scan = msg
        self.mark('scan', msg.header.stamp)

    def on_odom(self, msg):
        self.odom = msg
        self.mark('odom', msg.header.stamp)

    def on_cube_status(self, msg):
        try:
            self.cube_status = json.loads(msg.data)
            stamp = self.cube_status.get('stamp')
            if stamp:
                source_stamp = TimeMsg(**stamp)
                self.mark('camera', source_stamp)
                transform = self.odom_transform_at(source_stamp)
                if transform is not None:
                    for item in self.cube_status.get('detections', []):
                        if not item.get('valid'):
                            continue
                        xy = item.get('center_position_base_m')
                        uncertainty = item.get('position_uncertainty_m')
                        if xy is None or uncertainty is None:
                            continue
                        self.tracker.observe(
                            item.get('color'), transform_xy(*xy, transform),
                            uncertainty, source_stamp, time.monotonic(),
                            item.get('track_id'), item.get('confirmed', False),
                            item.get('partial_view', False))
                self.tracker.note_visibility(
                    {item.get('color') for item in self.cube_status.get('detections', [])
                     if item.get('valid')})
        except (ValueError, TypeError):
            self.cube_status = None

    def on_clearance(self, msg):
        try:
            self.cube_clearance = json.loads(msg.data)
        except (ValueError, TypeError):
            self.cube_clearance = None

    def on_stop(self, msg):
        if msg.data:
            self.emergency_latched = True
            self.stop_robot()

    def odom_transform_at(self, stamp):
        try:
            return self.tf_buffer.lookup_transform(
                'odom', 'base_footprint', Time.from_msg(stamp))
        except Exception:
            try:
                transform = self.tf_buffer.lookup_transform(
                    'odom', 'base_footprint', Time())
                delta = abs((Time.from_msg(stamp) -
                             Time.from_msg(transform.header.stamp)).nanoseconds)/1e9
                return transform if delta <= .05 else None
            except Exception:
                return None

    def map_transform(self):
        try:
            transform = self.tf_buffer.lookup_transform('map', 'odom', Time())
            age = (self.get_clock().now() -
                   Time.from_msg(transform.header.stamp)).nanoseconds/1e9
            # SLAM Toolbox publishes map->odom up to transform_timeout (0.2 s)
            # into the future. Accept that documented lead without changing
            # the source stamp or the SLAM configuration.
            if not -.25 <= age <= .3:
                return None
            return transform
        except Exception:
            return None

    def robot_pose(self, map_odom):
        try:
            base = self.tf_buffer.lookup_transform('odom', 'base_footprint', Time())
            age = (self.get_clock().now() - Time.from_msg(base.header.stamp)).nanoseconds/1e9
            if not 0 <= age <= .2:
                return None
            x, y = transform_xy(base.transform.translation.x,
                                base.transform.translation.y, map_odom)
            yaw = yaw_of(map_odom.transform.rotation) + yaw_of(base.transform.rotation)
            return (x, y, math.atan2(math.sin(yaw), math.cos(yaw)))
        except Exception:
            return None

    def grid(self, map_odom):
        info = self.map_msg.info
        origin = info.origin
        # Gazebo world is offset from odom by the fixed competition spawn.
        # Convert layout exclusions to odom, then use the live map<-odom TF.
        obstacles = [(*transform_xy(x-START_WORLD_XY[0],
                                    y-START_WORLD_XY[1], map_odom),
                      ZONE_UNCERTAINTY_M, 0.)
                     for x, y in INITIAL_CUBE_ZONES]
        now = time.monotonic()
        for xy, uncertainty, age in self.tracker.obstacles(now):
            x, y = transform_xy(*xy, map_odom)
            obstacles.append((x, y, uncertainty, age))
        return make_grid(self.map_msg.data, info.width, info.height,
                         info.resolution,
                         (origin.position.x, origin.position.y,
                          yaw_of(origin.orientation)), obstacles)

    def failures(self):
        now = time.monotonic()
        limits = {'scan': .35, 'camera': .5, 'odom': .2, 'map': 8.}
        failures = [name+'_stale' for name, limit in limits.items()
                    if now-self.seen_wall.get(name, -math.inf) > limit]
        if self.emergency_latched:
            failures.append('emergency_stop_latched')
        if self.time_reset_latched:
            failures.append('simulation_time_reset_latched')
        if self.safety_halt_latched:
            failures.append('safety_halt_latched')
        if self.cube_status is None or self.cube_status.get('perception_state') == 'invalid_perception':
            failures.append('cube_perception_invalid')
        for color, cube in self.cubes.items():
            if cube['inconsistent']:
                failures.append(color+'_cube_position_jump')
        return failures

    def forward_scan_clear(self):
        if self.scan is None:
            return False
        values = []
        for index, distance in enumerate(self.scan.ranges):
            angle = self.scan.angle_min + index*self.scan.angle_increment
            if abs(math.atan2(math.sin(angle), math.cos(angle))) <= .4 and math.isfinite(distance):
                values.append(distance)
        return bool(values) and min(values) >= ROBOT_SWEEP_RADIUS_M + .35

    def plan_tick(self):
        begin = time.monotonic()
        try:
            map_odom = self.map_transform()
            pose = self.robot_pose(map_odom) if map_odom is not None else None
            grid = self.grid(map_odom) if pose is not None and self.map_msg is not None else None
            if grid is None or pose is None:
                if self.validation_mode == 'none':
                    if self.visual_goal_active:
                        self.clear_visual_plan()
                    self.goal = None
                    self.path = []
                elif self.motion_ever_commanded:
                    self.safety_halt_latched = True
                    self.stop_robot()
                self.last_blocker = 'map_or_tf_unavailable'
            elif self.validation_mode != 'none':
                # The bounded trial owns its goal. Frontier replanning must
                # never replace or clear it during the trial.
                if (not self.validation_done and self.goal is not None and
                        not path_clear(grid, [pose[:2], *self.path])):
                    self.last_path_unsafe_xy = first_unsafe_path_point(
                        grid, [pose[:2], *self.path])
                    self.last_blocker = 'bounded_path_became_unsafe'
                    if self.motion_ever_commanded:
                        self.safety_halt_latched = True
                    self.stop_robot()
            else:
                unsafe_current = self.goal is not None and not path_clear(
                    grid, [pose[:2], *self.path])
                if unsafe_current:
                    self.blacklist.add(self.goal['cell'])
                    self.failed_goals += 1
                    self.goal = None
                    self.path = []
                should_plan = (self.goal is None and
                               (unsafe_current or time.monotonic()-self.last_plan > 5.))
                if should_plan:
                    selected = select_frontier(grid, pose[:2], self.blacklist)
                else:
                    selected = None
                if should_plan:
                    self.last_plan = time.monotonic()
                    self.replans += 1
                    if selected is None:
                        if self.visual_goal_active:
                            self.clear_visual_plan()
                        self.goal = None
                        self.path = []
                        self.last_blocker = 'no_reachable_safe_frontier'
                    else:
                        self.goal = selected
                        self.path = selected['path']
                        self.publish_plan(grid, selected)
                        self.last_blocker = 'motion_opt_in_required' if not self.motion_requested else None
            self.last_plan_ms = (time.monotonic()-begin)*1000
            known = None if grid is None else float(np.count_nonzero(grid.values >= 0)/grid.values.size)
            ready = (self.motion_requested and not self.validation_done and
                     not self.failures() and
                     pose is not None and grid is not None and
                     (self.goal is not None or self.validation_mode != 'none'))
            report = {'mode': 'planning_only' if not self.motion_requested else 'motion_requested',
                      'motion_enabled': bool(ready),
                      'blockers': self.failures() + ([self.last_blocker] if self.last_blocker else []),
                      'known_grid_fraction': known, 'goal': None if self.goal is None else self.goal['goal'],
                      'goal_yaw_rad': None if self.goal is None else self.goal.get('goal_yaw'),
                      'path_cells': len(self.path), 'cube_tracks': sorted(self.cubes),
                      'planning_ms': self.last_plan_ms, 'replans': self.replans,
                      'failed_goals': self.failed_goals,
                      'last_command': self.last_command,
                      'validation_mode': self.validation_mode,
                      'validation_done': self.validation_done,
                      'minimum_scan_clearance_m': self.minimum_scan_clearance,
                      'safety_halt_latched': self.safety_halt_latched,
                      'camera_tracking_interruptions': self.tracker.interruptions,
                      'cube_track_reacquisitions': self.tracker.reacquisitions,
                      'cube_track_rejections': self.tracker.rejections,
                      'cube_track_ages_s': {color: round(self.tracker.age(color, time.monotonic()), 3)
                                           for color in self.cubes},
                      'cube_track_details': {color: {'odom_xy': track['odom_xy'],
                                                     'uncertainty_m': track['uncertainty'],
                                                     'confirmed': track['confirmed']}
                                             for color, track in self.cubes.items()},
                      'first_unsafe_path_xy': self.last_path_unsafe_xy}
            self.status_pub.publish(String(data=json.dumps(report, allow_nan=False)))
        except Exception as error:
            self.last_blocker = 'planner_exception:' + type(error).__name__
            if self.motion_ever_commanded:
                self.safety_halt_latched = True
            self.stop_robot()
            self.get_logger().error(str(error))

    def publish_plan(self, grid, selected):
        stamp = self.get_clock().now().to_msg()
        path = Path()
        path.header.frame_id = 'map'
        path.header.stamp = stamp
        for x, y in selected['path']:
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x, pose.pose.position.y = x, y
            pose.pose.orientation.w = 1.
            path.poses.append(pose)
        self.path_pub.publish(path)
        goal = PoseStamped()
        goal.header = path.header
        goal.pose.position.x, goal.pose.position.y = selected['goal']
        goal_yaw = selected.get('goal_yaw', 0.)
        goal.pose.orientation.z = math.sin(goal_yaw/2)
        goal.pose.orientation.w = math.cos(goal_yaw/2)
        self.goal_pub.publish(goal)
        marker = Marker()
        marker.header = path.header
        marker.ns = 'frontier_candidates'
        marker.id = 0
        marker.type = Marker.POINTS
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.
        marker.scale.x = marker.scale.y = .04
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = .9, .4, .1, .8
        for group in frontier_groups(grid):
            for col, row in group:
                x, y = grid.cell_to_world(col, row)
                marker.points.append(Point(x=x, y=y, z=.02))
        goal_marker = Marker()
        goal_marker.header = path.header
        goal_marker.ns = 'exploration_goal'
        goal_marker.id = 1
        goal_marker.type = Marker.SPHERE
        goal_marker.action = Marker.ADD
        goal_marker.pose.position.x, goal_marker.pose.position.y = selected['goal']
        goal_marker.pose.position.z = .08
        goal_marker.pose.orientation.w = 1.
        goal_marker.scale.x = goal_marker.scale.y = goal_marker.scale.z = .16
        goal_marker.color.r, goal_marker.color.g = 1., .55
        goal_marker.color.a = .9
        self.markers_pub.publish(MarkerArray(markers=[marker, goal_marker]))
        self.visual_goal_active = True

    def clear_visual_plan(self):
        stamp = self.get_clock().now().to_msg()
        empty = Path()
        empty.header.frame_id = 'map'
        empty.header.stamp = stamp
        self.path_pub.publish(empty)
        removed = []
        for namespace, marker_id in (('frontier_candidates', 0),
                                     ('exploration_goal', 1)):
            marker = Marker()
            marker.header = empty.header
            marker.ns = namespace
            marker.id = marker_id
            marker.action = Marker.DELETE
            removed.append(marker)
        self.markers_pub.publish(MarkerArray(markers=removed))
        self.visual_goal_active = False

    def stop_robot(self):
        self.last_command = (0., 0.)
        if self.motion_requested:
            self.cmd_pub.publish(Twist())

    def control_tick(self):
        if not self.motion_requested:
            return
        if self.validation_done:
            self.stop_robot()
            return
        try:
            failures = self.failures()
            map_odom = self.map_transform()
            pose = self.robot_pose(map_odom) if map_odom is not None else None
            if failures or pose is None or (self.goal is None and self.validation_mode == 'none'):
                if self.motion_ever_commanded and (failures or pose is None):
                    self.safety_halt_latched = True
                if not self.safety_halt_latched or not self.last_blocker:
                    self.last_blocker = (failures[0] if failures else
                                         'local_clearance_or_goal_invalid')
                self.stop_robot()
                return
            grid = self.grid(map_odom)
            if self.validation_mode == 'none' and not path_clear(grid, [pose[:2], *self.path]):
                self.last_blocker = 'planned_path_became_unsafe'
                self.stop_robot()
                self.goal = None
                return
            if self.validation_mode != 'none':
                if self.validation_start is None:
                    self.validation_start = pose
                if self.validation_mode == 'rotation':
                    delta = math.atan2(math.sin(pose[2]-self.validation_start[2]),
                                       math.cos(pose[2]-self.validation_start[2]))
                    complete = delta >= .18
                    linear, angular = 0., min(self.max_angular, max(0., (.20-delta)*1.5))
                elif self.validation_mode == 'translation':
                    delta = math.dist(pose[:2], self.validation_start[:2])
                    complete = delta >= .10
                    linear, angular = min(self.max_linear, max(0., (.12-delta)*.8)), 0.
                else:
                    if not getattr(self, 'validation_goal_planned', False):
                        goal_xy = (pose[0], pose[1] + .25)
                        goal_cell = grid.world_to_cell(*goal_xy)
                        cells = astar(grid, grid.world_to_cell(*pose[:2]), goal_cell)
                        if cells is None:
                            self.last_blocker = 'local_goal_not_reachable'
                            self.stop_robot()
                            return
                        self.path = [grid.cell_to_world(*cell) for cell in cells]
                        goal_yaw = math.atan2(goal_xy[1]-pose[1], goal_xy[0]-pose[0])
                        self.goal = {'cell': goal_cell, 'goal': goal_xy,
                                     'goal_yaw': goal_yaw,
                                     'path': self.path, 'frontier_cells': 0}
                        self.publish_plan(grid, self.goal)
                        self.validation_goal_planned = True
                    distance = math.dist(pose[:2], self.goal['goal'])
                    heading_error = math.atan2(
                        math.sin(self.goal['goal_yaw']-pose[2]),
                        math.cos(self.goal['goal_yaw']-pose[2]))
                    complete = distance <= .08 and abs(heading_error) <= .15
                    angular = max(-self.max_angular,
                                  min(self.max_angular, 1.5*heading_error))
                    linear = (0. if abs(heading_error) > .40 or distance <= .08 else
                              min(self.max_linear, .8*distance))
                if self.validation_done:
                    self.stop_robot()
                    return
            else:
                linear, angular, complete = control_command(
                    pose, self.path, self.max_linear, self.max_angular)
            if complete:
                if self.validation_mode != 'none':
                    self.validation_done = True
                    self.last_blocker = 'validation_complete'
                    self.stop_robot()
                    return
                self.blacklist.add(self.goal['cell'])
                self.goal = None
                self.path = []
                self.clear_visual_plan()
                self.stop_robot()
                return
            # The drive decelerates after a zero command. Validate the larger
            # of requested and currently commanded speeds through that tail.
            check_linear = (linear if abs(linear) >= abs(self.last_command[0])
                            else self.last_command[0])
            check_angular = (angular if abs(angular) >= abs(self.last_command[1])
                             else self.last_command[1])
            reach = (ROBOT_SWEEP_RADIUS_M + .10 + abs(check_linear)*.5 +
                     stopping_distance(check_linear, 1.))
            if camera_candidate_blocks(self.cube_status, reach,
                                       self.cubes, time.monotonic()):
                self.last_blocker = 'visible_cube_nearfield_unvalidated'
                if self.motion_ever_commanded:
                    self.safety_halt_latched = True
                self.stop_robot()
                return
            clear, clearance, reason = swept_clearance(
                grid, self.scan, pose, check_linear, check_angular)
            if not clear:
                self.last_blocker = reason
                if self.motion_ever_commanded:
                    self.safety_halt_latched = True
                self.stop_robot()
                return
            self.minimum_scan_clearance = (clearance if self.minimum_scan_clearance is None
                                           else min(clearance, self.minimum_scan_clearance))
            now = time.monotonic()
            linear, angular = rate_limit_command(
                self.last_command, (linear, angular), now-self.last_control_wall)
            self.last_control_wall = now
            command = Twist()
            command.linear.x, command.angular.z = linear, angular
            self.cmd_pub.publish(command)
            self.last_command = (linear, angular)
            if abs(linear) > 1e-5 or abs(angular) > 1e-5:
                self.motion_ever_commanded = True
            self.last_blocker = None
        except Exception as error:
            self.last_blocker = 'controller_exception:' + type(error).__name__
            self.safety_halt_latched = True
            self.stop_robot()
            self.get_logger().error(str(error))


def main():
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = AutonomousExploration()
    stopping = False

    def stop(_signal, _frame):
        nonlocal stopping
        stopping = True

    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)
    try:
        while rclpy.ok() and not stopping:
            rclpy.spin_once(node, timeout_sec=.05)
    finally:
        node.stop_robot()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
