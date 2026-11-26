#!/usr/bin/env python3
"""Gate Nav2 commands and publish camera-cube exclusions for Nav2 costmaps.

No Gazebo truth is read here. Initial cube zones are labeled fixed-layout
assumptions for pre-pushing navigation; sensor-derived tracks are separate.
"""
import json
import math
import signal
import time

import numpy as np
import rclpy
from builtin_interfaces.msg import Time as TimeMsg
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from nav2_msgs.srv import ClearEntireCostmap
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, QoSProfile, ReliabilityPolicy,
                       qos_profile_sensor_data)
from rclpy.signals import SignalHandlerOptions
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Bool, Header, String
from tf2_ros import Buffer, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

from autonomous_core import (CUBE_RADIUS_M, ROBOT_SWEEP_RADIUS_M, make_grid,
                             path_clear, rate_limit_command)
from cube_tracks import CubeTracks
from motion_safety import (INITIAL_CUBE_ZONES, START_WORLD_XY,
                           ZONE_UNCERTAINTY_M, camera_candidate_blocks,
                           stopping_distance, swept_clearance)


def yaw_of(q):
    return math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))


def transform_xy(x, y, transform):
    yaw = yaw_of(transform.transform.rotation)
    c, s = math.cos(yaw), math.sin(yaw)
    return (transform.transform.translation.x+c*x-s*y,
            transform.transform.translation.y+s*x+c*y)


def disk_points(x, y, radius, step=.05):
    """Dense occupied disk; inflation and the footprint add Nav2 clearance."""
    reach = math.ceil(radius/step)
    for ix in range(-reach, reach+1):
        for iy in range(-reach, reach+1):
            dx, dy = ix*step, iy*step
            if dx*dx+dy*dy <= radius*radius:
                yield (x+dx, y+dy, .09)


def segment_distance(point, start, end):
    dx, dy = end[0]-start[0], end[1]-start[1]
    length2 = dx*dx+dy*dy
    t = max(0., min(1., ((point[0]-start[0])*dx +
                             (point[1]-start[1])*dy)/length2)) if length2 else 0.
    return math.dist(point, (start[0]+t*dx, start[1]+t*dy))


def inverse_transform_xy(x, y, transform):
    yaw = yaw_of(transform.transform.rotation)
    dx = x-transform.transform.translation.x
    dy = y-transform.transform.translation.y
    return (math.cos(yaw)*dx+math.sin(yaw)*dy,
            -math.sin(yaw)*dx+math.cos(yaw)*dy)


class Nav2SafetySupervisor(Node):
    def __init__(self):
        super().__init__('nav2_safety_supervisor')
        self.declare_parameter('enable_motion', False)
        self.declare_parameter('max_linear_mps', .10)
        self.declare_parameter('max_angular_radps', .25)
        self.declare_parameter('passive_cube_drift_mps', .005)
        self.enable_motion = bool(self.get_parameter('enable_motion').value)
        self.max_linear = float(self.get_parameter('max_linear_mps').value)
        self.max_angular = float(self.get_parameter('max_angular_radps').value)
        drift = float(self.get_parameter('passive_cube_drift_mps').value)
        if not 0 < self.max_linear <= .10 or not 0 < self.max_angular <= .25:
            raise ValueError('Nav2 validation speeds must stay at Phase 4C limits')
        if not .005 <= drift <= .02:
            raise ValueError('Passive cube drift bound must be in [.005, .02] m/s')
        self.tf_buffer = Buffer(cache_time=Duration(seconds=10))
        self.tf_listener = TransformListener(self.tf_buffer, self)
        # In the unchanged single-robot pre-push arena, cubes are passive and
        # the guarded path prevents contact. Their uncertainty still grows.
        # Pushing mode must use a faster contact-aware model.
        self.tracker = CubeTracks(growth_mps=drift)
        self.map_msg = self.scan = self.odom = self.camera_status = None
        self.amcl_pose = None
        self.plan = None
        self.nav_command = Twist()
        self.nav_command_wall = None
        self.last_command = (0., 0.)
        self.last_tick = time.monotonic()
        self.seen = {}
        self.stamps = {}
        self.emergency_latched = False
        self.safety_halt_latched = False
        self.time_reset_latched = False
        self.motion_ever_commanded = False
        self.reason = 'motion_opt_in_required' if not self.enable_motion else 'awaiting_inputs'
        self.min_scan_margin = None
        self.trajectory = []
        self.reobservation = None
        self.reobservation_state = 'idle'
        self.reobservation_color = None
        self.costmap_refresh_futures = None
        self.costmap_refresh_until = 0.
        reliable = QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE)
        map_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(OccupancyGrid, '/map', self.on_map, map_qos)
        self.create_subscription(LaserScan, '/scan', self.on_scan,
                                 qos_profile_sensor_data)
        self.create_subscription(Odometry, '/odom', self.on_odom, reliable)
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose',
                                 self.on_amcl, reliable)
        self.create_subscription(String, '/cube_detector/status',
                                 self.on_camera, reliable)
        self.create_subscription(Path, '/plan', self.on_plan, reliable)
        self.create_subscription(Twist, '/nav2_cmd_vel', self.on_nav_command,
                                 reliable)
        self.create_subscription(Bool, '/emergency_stop', self.on_emergency,
                                 reliable)
        self.create_subscription(String, '/nav2/reobservation_request',
                                 self.on_reobservation_request, reliable)
        self.global_clear = self.create_client(
            ClearEntireCostmap, '/global_costmap/clear_entirely_global_costmap')
        self.local_clear = self.create_client(
            ClearEntireCostmap, '/local_costmap/clear_entirely_local_costmap')
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 1)
        self.cloud_pub = self.create_publisher(
            point_cloud2.PointCloud2, '/nav2/cube_obstacles', 2)
        self.status_pub = self.create_publisher(String, '/nav2/safety_status', 2)
        self.trajectory_pub = self.create_publisher(Path, '/nav2/trajectory', 1)
        self.marker_pub = self.create_publisher(MarkerArray,
                                                '/nav2/safety_markers', 1)
        self.create_timer(.1, self.control_tick)
        self.create_timer(.2, self.publish_cube_cloud)
        self.create_timer(1., self.report_tick)
        self.cmd_pub.publish(Twist())

    def mark(self, name, stamp):
        value = stamp.sec + stamp.nanosec*1e-9
        if value <= 0:
            return
        old = self.stamps.get(name)
        if old is not None and value < old-.1:
            self.time_reset_latched = True
        if old is not None and value <= old:
            return
        self.stamps[name] = value
        self.seen[name] = time.monotonic()

    def on_map(self, msg):
        self.map_msg = msg
        self.mark('map', msg.header.stamp)

    def on_scan(self, msg):
        self.scan = msg
        self.mark('scan', msg.header.stamp)

    def on_odom(self, msg):
        self.odom = msg
        self.mark('odom', msg.header.stamp)

    def on_amcl(self, msg):
        self.amcl_pose = msg
        self.mark('amcl', msg.header.stamp)

    def on_plan(self, msg):
        if msg.header.frame_id == 'map' and msg.poses:
            self.plan = [(p.pose.position.x, p.pose.position.y)
                         for p in msg.poses]

    def on_nav_command(self, msg):
        self.nav_command = msg
        self.nav_command_wall = time.monotonic()

    def on_emergency(self, msg):
        if msg.data:
            self.emergency_latched = True
            self.stop('emergency_stop_latched')

    def on_reobservation_request(self, msg):
        """Arm one camera-facing rotation only after the whole sweep is clear."""
        try:
            request = json.loads(msg.data)
            goal = request['blocked_goal_map_xy']
            if (len(goal) != 2 or not all(math.isfinite(v) for v in goal)):
                raise ValueError('Invalid goal')
        except (KeyError, TypeError, ValueError):
            self.reobservation_state = 'invalid_request'
            return
        now = time.monotonic()
        if (not self.enable_motion or self.reobservation is not None or
                self.costmap_refresh_futures is not None or
                now < self.costmap_refresh_until or self.input_failures() or
                any(abs(v) > 1e-5 for v in self.last_command) or
                (self.nav_command_wall is not None and
                 now-self.nav_command_wall < .5 and
                 (abs(self.nav_command.linear.x) > 1e-5 or
                  abs(self.nav_command.angular.z) > 1e-5))):
            self.reobservation_state = 'preconditions_failed'
            return
        map_odom = self.map_transform()
        pose = self.robot_pose(map_odom) if map_odom is not None else None
        if pose is None:
            self.reobservation_state = 'tf_invalid'
            return
        goal_odom = inverse_transform_xy(*goal, map_odom)
        base_odom = self.odom_transform_at(self.odom.header.stamp)
        if base_odom is None:
            self.reobservation_state = 'odom_tf_invalid'
            return
        start_odom = (base_odom.transform.translation.x,
                      base_odom.transform.translation.y)
        candidates = []
        for color, track in self.tracker.tracks.items():
            age = self.tracker.age(color, now)
            if track['inconsistent'] or not track['confirmed'] or age <= 1.2:
                continue
            radius = (ROBOT_SWEEP_RADIUS_M + .10 + CUBE_RADIUS_M +
                      track['uncertainty'] + self.tracker.growth_mps*age)
            distance = segment_distance(track['odom_xy'],start_odom,goal_odom)
            # Start before the exclusion reaches the route. A later request
            # may be impossible because the uncertain disk reaches the robot.
            if distance <= radius + .20:
                candidates.append((distance-radius,color,track))
        if not candidates:
            self.reobservation_state = 'no_stale_track_intersects_route'
            return
        _,color,track = min(candidates)
        target = math.atan2(track['odom_xy'][1]-start_odom[1],
                            track['odom_xy'][0]-start_odom[0])
        base_yaw = yaw_of(base_odom.transform.rotation)
        delta = math.atan2(math.sin(target-base_yaw),math.cos(target-base_yaw))
        if not .10 <= abs(delta) <= 1.8:
            self.reobservation_state = 'observation_angle_unavailable'
            return
        direction = math.copysign(1.,delta)
        grid = self.grid(map_odom)
        nearfield = ROBOT_SWEEP_RADIUS_M + .10 + stopping_distance(0.,1.)
        if camera_candidate_blocks(self.camera_status,nearfield,
                                   self.tracker.tracks,now):
            self.reobservation_state = 'camera_nearfield_unvalidated'
            return
        clear,_,reason = swept_clearance(
            grid,self.scan,pose,0.,direction*.15,horizon=abs(delta)/.15)
        if not clear:
            self.reobservation_state = 'rotation_unsafe_'+reason
            return
        self.reobservation = {'color':color,'direction':direction,
                              'start_yaw':base_yaw,'max_angle':abs(delta)+.08,
                              'started':now,'track_seen':track['seen']}
        self.reobservation_color = color
        self.reobservation_state = 'rotating_to_observe'
        self.last_tick = now

    def start_costmap_refresh(self):
        # Clear only while stopped; the static map, scan, and current cube cloud
        # repopulate the costmaps before any later Nav2 command is admitted.
        self.stop('costmap_refresh_hold')
        if not self.global_clear.service_is_ready() or not self.local_clear.service_is_ready():
            self.reobservation_state = 'costmap_clear_service_unavailable'
            self.costmap_refresh_until = math.inf
            return
        request = ClearEntireCostmap.Request()
        self.costmap_refresh_futures = (
            self.global_clear.call_async(request),
            self.local_clear.call_async(ClearEntireCostmap.Request()))
        self.reobservation_state = 'refreshing_costmaps'

    def refresh_tick(self):
        if self.costmap_refresh_futures is None:
            return False
        self.stop('costmap_refresh_hold')
        if not all(f.done() for f in self.costmap_refresh_futures):
            return True
        if any(f.exception() is not None for f in self.costmap_refresh_futures):
            self.reobservation_state = 'costmap_refresh_failed'
            self.costmap_refresh_until = math.inf
        else:
            self.reobservation_state = 'reobservation_complete'
            self.costmap_refresh_until = time.monotonic()+2.
        self.costmap_refresh_futures = None
        return True

    def odom_transform_at(self, stamp):
        try:
            return self.tf_buffer.lookup_transform('odom', 'base_footprint',
                                                   Time.from_msg(stamp))
        except Exception:
            return None

    def on_camera(self, msg):
        try:
            status = json.loads(msg.data)
        except (ValueError, TypeError):
            self.camera_status = None
            return
        self.camera_status = status
        stamp = status.get('stamp')
        if not stamp:
            return
        source = TimeMsg(**stamp)
        self.mark('camera', source)
        transform = self.odom_transform_at(source)
        if transform is not None:
            for item in status.get('detections', []):
                if not item.get('valid'):
                    continue
                xy = item.get('center_position_base_m')
                uncertainty = item.get('position_uncertainty_m')
                if xy is None or uncertainty is None:
                    continue
                self.tracker.observe(
                    item.get('color'), transform_xy(*xy, transform),
                    uncertainty, source, time.monotonic(),
                    item.get('track_id'), item.get('confirmed', False),
                    item.get('partial_view', False))
        self.tracker.note_visibility(
            {item.get('color') for item in status.get('detections', [])
             if item.get('valid')})

    def map_transform(self):
        try:
            transform = self.tf_buffer.lookup_transform('map', 'odom', Time())
            age = (self.get_clock().now()-Time.from_msg(
                transform.header.stamp)).nanoseconds/1e9
            return transform if -.5 <= age <= .6 else None
        except Exception:
            return None

    def robot_pose(self, map_odom):
        try:
            base = self.tf_buffer.lookup_transform('odom', 'base_footprint',
                                                   Time())
            age = (self.get_clock().now()-Time.from_msg(
                base.header.stamp)).nanoseconds/1e9
            if not 0 <= age <= .2:
                return None
            xy = transform_xy(base.transform.translation.x,
                              base.transform.translation.y, map_odom)
            yaw = yaw_of(map_odom.transform.rotation)+yaw_of(
                base.transform.rotation)
            return (xy[0], xy[1], math.atan2(math.sin(yaw), math.cos(yaw)))
        except Exception:
            return None

    def layout_odom(self):
        return [(x-START_WORLD_XY[0], y-START_WORLD_XY[1])
                for x,y in INITIAL_CUBE_ZONES]

    def grid(self, map_odom):
        info = self.map_msg.info
        origin = info.origin
        obstacles = [(*transform_xy(x,y,map_odom),ZONE_UNCERTAINTY_M,0.)
                     for x,y in self.layout_odom()]
        for xy, uncertainty, age in self.tracker.obstacles(time.monotonic()):
            obstacles.append((*transform_xy(*xy,map_odom),uncertainty,age))
        return make_grid(self.map_msg.data,info.width,info.height,
                         info.resolution,
                         (origin.position.x,origin.position.y,
                          yaw_of(origin.orientation)),obstacles)

    def input_failures(self):
        now = time.monotonic()
        # AMCL publishes a pose only when its motion update threshold is met.
        # Its map->odom transform is refreshed from scans and is checked below.
        limits = {'scan':.35,'camera':.5,'odom':.2}
        failures = [name+'_stale' for name,limit in limits.items()
                    if now-self.seen.get(name,-math.inf)>limit]
        if self.map_msg is None:
            failures.append('map_unavailable')
        if self.amcl_pose is None:
            failures.append('amcl_pose_unavailable')
        if self.emergency_latched:
            failures.append('emergency_stop_latched')
        if self.safety_halt_latched:
            failures.append('safety_halt_latched')
        if self.time_reset_latched:
            failures.append('simulation_time_reset_latched')
        if (self.camera_status is None or
                self.camera_status.get('perception_state') == 'invalid_perception'):
            failures.append('cube_perception_invalid')
        if self.amcl_pose is not None:
            covariance = self.amcl_pose.pose.covariance
            if (not all(math.isfinite(covariance[i]) for i in (0,7,35)) or
                    max(covariance[0],covariance[7]) > .25 or
                    covariance[35] > .25):
                failures.append('localization_uncertain')
        for color,track in self.tracker.tracks.items():
            if track['inconsistent']:
                failures.append(color+'_cube_position_jump')
        return failures

    def stop(self, reason):
        self.last_command = (0.,0.)
        self.last_tick = time.monotonic()
        self.reason = reason
        self.cmd_pub.publish(Twist())

    def control_tick(self):
        now = time.monotonic()
        if not self.enable_motion:
            self.stop('motion_opt_in_required')
            return
        if self.safety_halt_latched:
            self.cmd_pub.publish(Twist())
            return
        if self.refresh_tick():
            return
        if now < self.costmap_refresh_until:
            self.stop('costmap_refresh_hold')
            return
        failures = self.input_failures()
        map_odom = self.map_transform()
        pose = self.robot_pose(map_odom) if map_odom is not None else None
        if map_odom is None:
            failures.append('map_to_odom_tf_invalid')
        if pose is None:
            failures.append('robot_tf_invalid')
        moving = any(abs(v)>1e-5 for v in self.last_command)
        if failures:
            if moving:
                self.safety_halt_latched = True
            self.stop(failures[0])
            return
        if self.reobservation is not None:
            request = self.reobservation
            track = self.tracker.tracks.get(request['color'])
            if (track is not None and track['confirmed'] and
                    not track['inconsistent'] and
                    track['seen'] > request['track_seen']+.05 and
                    self.tracker.fresh(request['color'],now)):
                self.reobservation = None
                self.start_costmap_refresh()
                return
            if now-request['started'] > 18.:
                self.reobservation = None
                self.reobservation_state = 'reobservation_timed_out'
                self.stop('reobservation_timed_out')
                return
            base = self.odom_transform_at(self.odom.header.stamp)
            if base is None:
                self.safety_halt_latched = True
                self.stop('reobservation_odom_tf_invalid')
                return
            turned = abs(math.atan2(
                math.sin(yaw_of(base.transform.rotation)-request['start_yaw']),
                math.cos(yaw_of(base.transform.rotation)-request['start_yaw'])))
            if turned >= request['max_angle']:
                self.reobservation = None
                self.reobservation_state = 'cube_not_reacquired'
                self.stop('cube_not_reacquired')
                return
            angular = request['direction']*.15
            grid = self.grid(map_odom)
            clear,margin,reason = swept_clearance(grid,self.scan,pose,0.,angular)
            if not clear or camera_candidate_blocks(
                    self.camera_status,ROBOT_SWEEP_RADIUS_M+.10,
                    self.tracker.tracks,now):
                self.safety_halt_latched = any(abs(v)>1e-5
                                               for v in self.last_command)
                self.reobservation = None
                self.reobservation_state = 'rotation_stopped_unsafe'
                self.stop(reason or 'camera_nearfield_unvalidated')
                return
            self.min_scan_margin = (margin if self.min_scan_margin is None else
                                    min(self.min_scan_margin,margin))
            limited = rate_limit_command(self.last_command,(0.,angular),
                                         now-self.last_tick)
            self.last_tick = now
            cmd = Twist()
            cmd.angular.z = limited[1]
            self.cmd_pub.publish(cmd)
            self.last_command = limited
            self.reason = 'reobserving_'+request['color']
            return
        if self.nav_command_wall is None or now-self.nav_command_wall>.3:
            self.stop('nav2_command_idle')
            return
        requested = (self.nav_command.linear.x,self.nav_command.angular.z)
        if abs(requested[0])<1e-5 and abs(requested[1])<1e-5:
            self.stop('nav2_command_zero')
            return
        if self.plan is None:
            self.stop('nav2_plan_unavailable')
            return
        if (not all(math.isfinite(v) for v in requested) or
                abs(requested[0]) > self.max_linear+1e-3 or
                abs(requested[1]) > self.max_angular+1e-3 or
                any(abs(v)>1e-8 for v in (self.nav_command.linear.y,
                                          self.nav_command.linear.z,
                                          self.nav_command.angular.x,
                                          self.nav_command.angular.y))):
            self.safety_halt_latched = True
            self.stop('nav2_command_out_of_range')
            return
        grid = self.grid(map_odom)
        if not path_clear(grid,[pose[:2],*self.plan]):
            self.safety_halt_latched = moving
            self.stop('nav2_path_unsafe')
            return
        check_linear = (requested[0] if abs(requested[0])>=abs(self.last_command[0])
                        else self.last_command[0])
        check_angular = (requested[1] if abs(requested[1])>=abs(self.last_command[1])
                         else self.last_command[1])
        reach = (ROBOT_SWEEP_RADIUS_M+.10+abs(check_linear)*.5+
                 stopping_distance(check_linear,1.))
        if camera_candidate_blocks(self.camera_status,reach,
                                   self.tracker.tracks,now):
            self.safety_halt_latched = moving
            self.stop('visible_cube_nearfield_unvalidated')
            return
        clear, margin, reason = swept_clearance(
            grid,self.scan,pose,check_linear,check_angular)
        if not clear:
            self.safety_halt_latched = moving
            self.stop(reason)
            return
        self.min_scan_margin = (margin if self.min_scan_margin is None else
                                min(self.min_scan_margin,margin))
        limited = rate_limit_command(self.last_command,requested,now-self.last_tick)
        self.last_tick = now
        cmd = Twist()
        cmd.linear.x,cmd.angular.z = limited
        self.cmd_pub.publish(cmd)
        self.last_command = limited
        if abs(limited[0])>1e-5 or abs(limited[1])>1e-5:
            self.motion_ever_commanded = True
        self.reason = 'moving' if self.motion_ever_commanded else 'ready'
        if not self.trajectory or math.dist(pose[:2],
            (self.trajectory[-1].pose.position.x,
             self.trajectory[-1].pose.position.y))>.02:
            waypoint = PoseStamped()
            waypoint.header.frame_id = 'map'
            waypoint.header.stamp = self.get_clock().now().to_msg()
            waypoint.pose.position.x,waypoint.pose.position.y = pose[:2]
            waypoint.pose.orientation.z = math.sin(pose[2]/2)
            waypoint.pose.orientation.w = math.cos(pose[2]/2)
            self.trajectory.append(waypoint)
            path = Path()
            path.header = waypoint.header
            path.poses = self.trajectory[-1000:]
            self.trajectory_pub.publish(path)

    def publish_cube_cloud(self):
        points = []
        for x,y in self.layout_odom():
            points.extend(disk_points(x,y,CUBE_RADIUS_M+ZONE_UNCERTAINTY_M))
        for xy,uncertainty,_ in self.tracker.obstacles(time.monotonic()):
            points.extend(disk_points(*xy,CUBE_RADIUS_M+uncertainty))
        header = Header()
        header.frame_id = 'odom'
        header.stamp = self.get_clock().now().to_msg()
        self.cloud_pub.publish(point_cloud2.create_cloud_xyz32(header,points))

    def report_tick(self):
        report = {'motion_requested':self.enable_motion,
                  'safety_halt_latched':self.safety_halt_latched,
                  'emergency_latched':self.emergency_latched,
                  'reason':self.reason,
                  'last_command':self.last_command,
                  'minimum_scan_margin_m':self.min_scan_margin,
                  'cube_tracks':sorted(self.tracker.tracks),
                  'camera_tracking_interruptions':self.tracker.interruptions,
                  'cube_track_rejections':self.tracker.rejections,
                  'cube_obstacles':{
                      color:{'odom_xy':track['odom_xy'],
                             'age_s':self.tracker.age(color,time.monotonic()),
                             'position_uncertainty_m':track['uncertainty']+
                                 self.tracker.growth_mps*self.tracker.age(color,time.monotonic()),
                             'confirmed':track['confirmed'],
                             'inconsistent':track['inconsistent']}
                      for color,track in self.tracker.tracks.items()},
                  'reobservation_state':self.reobservation_state,
                  'reobservation_color':self.reobservation_color,
                  'plan_points':0 if self.plan is None else len(self.plan)}
        self.status_pub.publish(String(data=json.dumps(report,allow_nan=False)))
        markers = []
        for index,(x,y) in enumerate(self.layout_odom()):
            marker = Marker()
            marker.header.frame_id = 'odom'
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = 'layout_cube_exclusions'
            marker.id = index
            marker.type = Marker.CYLINDER
            marker.action = Marker.ADD
            marker.pose.position.x,marker.pose.position.y = x,y
            marker.pose.orientation.w = 1.
            marker.scale.x = marker.scale.y = 2*(CUBE_RADIUS_M+ZONE_UNCERTAINTY_M)
            marker.scale.z = .01
            marker.color.r,marker.color.g,marker.color.b,marker.color.a = 1.,.45,.1,.25
            markers.append(marker)
        label = Marker()
        label.header.frame_id = 'base_footprint'
        label.header.stamp = self.get_clock().now().to_msg()
        label.ns = 'nav2_safety_state'
        label.id = 0
        label.type = Marker.TEXT_VIEW_FACING
        label.action = Marker.ADD
        label.pose.position.z = .55
        label.pose.orientation.w = 1.
        label.scale.z = .12
        label.color.r = 1. if self.reason != 'moving' else .2
        label.color.g = .35 if self.reason != 'moving' else 1.
        label.color.b = .2
        label.color.a = 1.
        label.text = 'Nav2 safety: '+self.reason
        markers.append(label)
        self.marker_pub.publish(MarkerArray(markers=markers))


def main():
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = Nav2SafetySupervisor()
    stopping = False
    def stop(_signal,_frame):
        nonlocal stopping
        stopping = True
    signal.signal(signal.SIGINT,stop)
    signal.signal(signal.SIGTERM,stop)
    try:
        while rclpy.ok() and not stopping:
            rclpy.spin_once(node,timeout_sec=.05)
    finally:
        node.stop('shutdown')
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
