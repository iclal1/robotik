#!/usr/bin/env python3
"""Independent Gazebo evaluation; never publishes control or perception data."""
import argparse
import json
import math
import time
from pathlib import Path

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, Path as NavPath
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String
from tf2_ros import Buffer, TransformListener
from rclpy.time import Time

from measure_autonomous_validation import gazebo_poses


def yaw(pose):
    q = pose.orientation if hasattr(pose, 'orientation') else pose.rotation
    return math.atan2(2*(q.w*q.z + q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))


def gz_yaw(pose):
    return 2*math.atan2(pose[5], pose[6])


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--seconds', type=float, default=35.)
    args = parser.parse_args()
    rclpy.init()
    node = Node('bounded_goal_evaluator')
    node.tf_buffer = Buffer()
    node.tf_listener = TransformListener(node.tf_buffer, node)
    odom = []
    statuses = []
    camera = []
    commands = []
    paths = []
    emergency_stops = []
    node.create_subscription(Odometry, '/odom',
        lambda m: odom.append((m.pose.pose.position.x, m.pose.pose.position.y,
                               yaw(m.pose.pose))), 10)
    node.create_subscription(String, '/autonomous_exploration/status',
        lambda m: statuses.append(json.loads(m.data)), 10)
    node.create_subscription(String, '/cube_detector/status',
        lambda m: camera.append(json.loads(m.data)), 10)
    node.create_subscription(Twist, '/cmd_vel',
        lambda m: commands.append((m.linear.x, m.angular.z)), 10)
    node.create_subscription(Bool, '/emergency_stop',
        lambda m: emergency_stops.append(m.data), 10)
    node.create_subscription(NavPath, '/autonomous_exploration/path',
        lambda m: paths.append([(p.pose.position.x, p.pose.position.y)
                                for p in m.poses]),
        QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                   durability=DurabilityPolicy.TRANSIENT_LOCAL))
    before = None
    for _ in range(40):
        rclpy.spin_once(node, timeout_sec=.05)
        try:
            sample = gazebo_poses()
            if 'pusher_robot' in sample:
                before = sample
                break
        except Exception:
            pass
    if before is None:
        raise RuntimeError('Robot did not spawn for evaluation')
    deadline = time.monotonic() + args.seconds
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=.05)
        if statuses and statuses[-1].get('validation_done'):
            break
        if statuses and statuses[-1].get('safety_halt_latched'):
            break
    for _ in range(20):
        rclpy.spin_once(node, timeout_sec=.05)
    map_pose = None
    try:
        transform = node.tf_buffer.lookup_transform('map', 'base_footprint', Time())
        map_pose = [transform.transform.translation.x,
                    transform.transform.translation.y,
                    yaw(transform.transform)]
    except Exception:
        pass
    node.destroy_node()
    rclpy.shutdown()
    after = gazebo_poses()
    robot_before, robot_after = before['pusher_robot'], after['pusher_robot']
    planned = max(paths, key=len, default=[])
    goal = next((s['goal'] for s in reversed(statuses) if s.get('goal')), None)
    position_error = (math.dist(map_pose[:2], goal)
                      if map_pose is not None and goal is not None else None)
    goal_yaw = next((s['goal_yaw_rad'] for s in reversed(statuses)
                     if s.get('goal_yaw_rad') is not None), None)
    heading_error = (abs(math.atan2(math.sin(goal_yaw-map_pose[2]),
                                    math.cos(goal_yaw-map_pose[2])))
                     if goal_yaw is not None and map_pose is not None else None)
    robot_radius = math.hypot(.2375, .12)
    cube_radius = math.sqrt(2)*.09
    world_trace = [robot_before[:2], robot_after[:2]] + [
        (robot_before[0] + sample[0] - odom[0][0],
         robot_before[1] + sample[1] - odom[0][1])
        for sample in odom] if odom else [robot_before[:2], robot_after[:2]]
    def nominal_clearance(xy):
        x, y = xy
        wall = min(x+2.95, 2.95-x, y+1.95, 1.95-y)-robot_radius
        cubes = [math.dist(xy, before[color][:2])-robot_radius-cube_radius
                 for color in ('red_cube','green_cube','blue_cube')]
        return min(wall, *cubes)
    min_nominal_clearance = min(nominal_clearance(xy) for xy in world_trace)
    report = {
        'goal_completed': bool(statuses and statuses[-1].get('validation_done')),
        'last_status': statuses[-1] if statuses else None,
        'first_halt_status': next((s for s in statuses
                                   if s.get('safety_halt_latched')), None),
        'status_samples': len(statuses),
        'robot_start_world_xy': robot_before[:2],
        'robot_end_world_xy': robot_after[:2],
        'robot_displacement_m': math.dist(robot_before[:2], robot_after[:2]),
        'gazebo_net_rotation_rad': abs(math.atan2(
            math.sin(gz_yaw(robot_after)-gz_yaw(robot_before)),
            math.cos(gz_yaw(robot_after)-gz_yaw(robot_before)))),
        'odom_observed_total_rotation_rad': (sum(abs(math.atan2(math.sin(b[2]-a[2]),
                                                               math.cos(b[2]-a[2])))
                                                 for a,b in zip(odom, odom[1:]))),
        'robot_final_yaw_rad': gz_yaw(robot_after),
        'odom_executed_path_m': sum(math.dist(a[:2], b[:2])
                                    for a,b in zip(odom, odom[1:])),
        'planned_path_m': sum(math.dist(a,b) for a,b in zip(planned,planned[1:])),
        'planned_path_points': len(planned),
        'goal_map_xy': goal,
        'final_map_pose': map_pose,
        'position_error_m': position_error,
        'heading_error_rad': heading_error,
        'minimum_nominal_geometry_clearance_m': min_nominal_clearance,
        'nominal_geometric_overlap': min_nominal_clearance <= 0,
        'cube_displacement_m': {color: math.dist(before[color][:2], after[color][:2])
                                for color in ('red_cube','green_cube','blue_cube')},
        'command_samples': len(commands),
        'max_command_linear_mps': max((abs(v) for v,w in commands), default=0.),
        'max_command_angular_radps': max((abs(w) for v,w in commands), default=0.),
        'emergency_stop_events': sum(emergency_stops),
        'camera_invalid_frames': sum(s.get('perception_state') == 'invalid_perception'
                                     for s in camera),
        'camera_unranged_detections': sum(not d.get('valid',False)
                                          for s in camera for d in s.get('detections',[])),
        'camera_unranged_events': [
            {'stamp': s.get('stamp'), 'color': d.get('color'),
             'track_id': d.get('track_id'), 'reason': d.get('reason'),
             'bounds': d.get('tentative_center_range_bounds_m'),
             'bbox': d.get('bbox')}
            for s in camera for d in s.get('detections',[])
            if not d.get('valid',False)][-30:],
        'mean_camera_processing_ms': (sum(s.get('processing_latency_ms') or 0
                                          for s in camera)/len(camera) if camera else None),
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2)+'\n')
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
