#!/usr/bin/env python3
"""Independent Nav2 action and Gazebo measurement; never publishes velocity."""
import argparse
import json
import math
import time
from pathlib import Path

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry, Path as NavPath
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import Bool, String
from tf2_ros import Buffer, TransformListener
from rclpy.time import Time

from measure_autonomous_validation import gazebo_poses


def yaw(q):
    return math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))


def pose_yaw(values):
    return 2*math.atan2(values[5], values[6])


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--seconds', type=float, default=45.)
    parser.add_argument('--goal-x', type=float, default=0.)
    parser.add_argument('--goal-y', type=float, default=.25)
    parser.add_argument('--goal-yaw', type=float, default=math.pi/2)
    args = parser.parse_args()
    rclpy.init()
    node = Node('nav2_goal_evaluator')
    node.tf_buffer = Buffer()
    node.tf_listener = TransformListener(node.tf_buffer, node)
    action = ActionClient(node, NavigateToPose, '/navigate_to_pose')
    odom, amcl, statuses, camera, commands, paths, estops = ([] for _ in range(7))
    node.create_subscription(Odometry, '/odom',
        lambda m: odom.append((m.pose.pose.position.x,m.pose.pose.position.y,
                               yaw(m.pose.pose.orientation))),10)
    node.create_subscription(PoseWithCovarianceStamped, '/amcl_pose',
        lambda m: amcl.append((m.pose.pose.position.x,m.pose.pose.position.y,
                               yaw(m.pose.pose.orientation))),10)
    node.create_subscription(String, '/nav2/safety_status',
        lambda m: statuses.append(json.loads(m.data)),10)
    node.create_subscription(String, '/cube_detector/status',
        lambda m: camera.append(json.loads(m.data)),10)
    node.create_subscription(Twist, '/cmd_vel',
        lambda m: commands.append((m.linear.x,m.angular.z)),10)
    node.create_subscription(NavPath, '/plan',
        lambda m: paths.append([(p.pose.position.x,p.pose.position.y)
                                for p in m.poses]),10)
    node.create_subscription(Bool, '/emergency_stop',
        lambda m: estops.append(m.data),10)
    before = None
    start_wait = time.monotonic()
    while time.monotonic()-start_wait < 12:
        rclpy.spin_once(node,timeout_sec=.05)
        try:
            before = gazebo_poses()
            if 'pusher_robot' in before and statuses and amcl and odom:
                break
        except Exception:
            pass
    if before is None or not action.wait_for_server(timeout_sec=5):
        raise RuntimeError('Gazebo pose or NavigateToPose server unavailable')
    goal = NavigateToPose.Goal()
    goal.pose.header.frame_id = 'map'
    goal.pose.header.stamp = node.get_clock().now().to_msg()
    goal.pose.pose.position.x = args.goal_x
    goal.pose.pose.position.y = args.goal_y
    goal.pose.pose.orientation.z = math.sin(args.goal_yaw/2)
    goal.pose.pose.orientation.w = math.cos(args.goal_yaw/2)
    sent = action.send_goal_async(goal)
    deadline = time.monotonic()+args.seconds
    while not sent.done() and time.monotonic()<deadline:
        rclpy.spin_once(node,timeout_sec=.05)
    handle = sent.result() if sent.done() else None
    result = handle.get_result_async() if handle and handle.accepted else None
    while result is not None and not result.done() and time.monotonic()<deadline:
        rclpy.spin_once(node,timeout_sec=.05)
        if statuses and statuses[-1].get('safety_halt_latched'):
            break
    if result is not None and not result.done():
        cancel = handle.cancel_goal_async()
        while not cancel.done() and time.monotonic()<deadline+3:
            rclpy.spin_once(node,timeout_sec=.05)
    for _ in range(10):
        rclpy.spin_once(node,timeout_sec=.05)
    final_tf = None
    try:
        t = node.tf_buffer.lookup_transform('map','base_footprint',Time())
        final_tf = [t.transform.translation.x,t.transform.translation.y,
                    yaw(t.transform.rotation)]
    except Exception:
        pass
    after = gazebo_poses()
    robot_before,robot_after = before['pusher_robot'],after['pusher_robot']
    planned = max(paths,key=len,default=[])
    status = result.result().status if result and result.done() else None
    final_pose = final_tf or (list(amcl[-1]) if amcl else None)
    pos_error = math.dist(final_pose[:2],(args.goal_x,args.goal_y)) if final_pose else None
    heading_error = abs(math.atan2(math.sin(final_pose[2]-args.goal_yaw),
                                   math.cos(final_pose[2]-args.goal_yaw))) if final_pose else None
    robot_radius = math.hypot(.2375,.12)
    cube_radius = math.sqrt(2)*.09
    world_trace = [robot_before[:2],robot_after[:2]]
    if odom:
        world_trace += [(robot_before[0]+p[0]-odom[0][0],
                         robot_before[1]+p[1]-odom[0][1]) for p in odom]
    def clearance(xy):
        x,y=xy
        wall=min(x+2.95,2.95-x,y+1.95,1.95-y)-robot_radius
        cubes=[math.dist(xy,before[c][:2])-robot_radius-cube_radius
               for c in ('red_cube','green_cube','blue_cube')]
        return min(wall,*cubes)
    report={
        'goal_accepted': bool(handle and handle.accepted),
        'nav2_status_code':status,
        'navigate_to_pose_succeeded':status==GoalStatus.STATUS_SUCCEEDED,
        'robot_displacement_m':math.dist(robot_before[:2],robot_after[:2]),
        'robot_start_world_xy':robot_before[:2],
        'robot_end_world_xy':robot_after[:2],
        'robot_rotation_rad':abs(math.atan2(math.sin(pose_yaw(robot_after)-pose_yaw(robot_before)),
                                              math.cos(pose_yaw(robot_after)-pose_yaw(robot_before)))),
        'odom_executed_path_m':sum(math.dist(a[:2],b[:2]) for a,b in zip(odom,odom[1:])),
        'planned_path_m':sum(math.dist(a,b) for a,b in zip(planned,planned[1:])),
        'planned_path_points':len(planned),
        'goal_map_xy':[args.goal_x,args.goal_y],
        'final_map_pose':final_pose,
        'position_error_m':pos_error,
        'heading_error_rad':heading_error,
        'minimum_nominal_geometry_clearance_m':min(clearance(p) for p in world_trace),
        'cube_displacement_m':{c:math.dist(before[c][:2],after[c][:2])
                               for c in ('red_cube','green_cube','blue_cube')},
        'emergency_stop_events':sum(estops),
        'safety_halt_latched':any(s.get('safety_halt_latched') for s in statuses),
        'final_safety_status':statuses[-1] if statuses else None,
        'max_command_linear_mps':max((abs(v) for v,w in commands),default=0.),
        'max_command_angular_radps':max((abs(w) for v,w in commands),default=0.),
        'camera_invalid_frames':sum(s.get('perception_state')=='invalid_perception' for s in camera),
        'camera_unranged_events':[{'color':d.get('color'),'reason':d.get('reason'),
                                    'bounds':d.get('tentative_center_range_bounds_m'),
                                    'track_id':d.get('track_id'),'bbox':d.get('bbox')}
                                   for s in camera for d in s.get('detections',[])
                                   if not d.get('valid',False)][-30:],
        'camera_tracking_interruptions':max((s.get('camera_tracking_interruptions',0) for s in statuses),default=0),
        'mean_camera_processing_ms':sum(s.get('processing_latency_ms') or 0 for s in camera)/len(camera) if camera else None,
        'duration_s':args.seconds-(deadline-time.monotonic()),
    }
    args.output.parent.mkdir(parents=True,exist_ok=True)
    args.output.write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(report,indent=2))
    node.destroy_node()
    rclpy.shutdown()


if __name__=='__main__':
    main()
