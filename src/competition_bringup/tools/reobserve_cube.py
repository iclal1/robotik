#!/usr/bin/env python3
"""Request a safety-checked cube observation before retrying a Nav2 route."""
import argparse
import json
import math
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import ComputePathToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import String


def compute_path(node, client, x, y, timeout=6.):
    goal = ComputePathToPose.Goal()
    goal.goal = PoseStamped()
    goal.goal.header.frame_id = 'map'
    goal.goal.header.stamp = node.get_clock().now().to_msg()
    goal.goal.pose.position.x = x
    goal.goal.pose.position.y = y
    goal.goal.pose.orientation.w = 1.
    sent = client.send_goal_async(goal)
    deadline = time.monotonic()+timeout
    while not sent.done() and time.monotonic()<deadline:
        rclpy.spin_once(node,timeout_sec=.05)
    if not sent.done() or not sent.result().accepted:
        return False
    result = sent.result().get_result_async()
    while not result.done() and time.monotonic()<deadline:
        rclpy.spin_once(node,timeout_sec=.05)
    return bool(result.done() and result.result().result.path.poses)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--goal-x',type=float,required=True)
    parser.add_argument('--goal-y',type=float,required=True)
    parser.add_argument('--proactive',action='store_true',
                        help='reobserve before uncertainty fully blocks the route')
    parser.add_argument('--seconds',type=float,default=24.)
    args = parser.parse_args()
    if not all(math.isfinite(v) for v in (args.goal_x,args.goal_y)):
        raise SystemExit('Finite goal coordinates are required')
    rclpy.init()
    node = Node('nav2_cube_reobservation_requester')
    status = []
    node.create_subscription(String,'/nav2/safety_status',
                             lambda m:status.append(json.loads(m.data)),10)
    request = node.create_publisher(String,'/nav2/reobservation_request',10)
    planner = ActionClient(node,ComputePathToPose,'/compute_path_to_pose')
    deadline = time.monotonic()+8.
    while (not status or not planner.server_is_ready()) and time.monotonic()<deadline:
        rclpy.spin_once(node,timeout_sec=.1)
    if not status or not planner.server_is_ready():
        raise SystemExit('Nav2 planner or safety supervisor unavailable')
    if not status[-1]['motion_requested']:
        raise SystemExit('Motion is disabled; launch with explicit enable_motion:=true')
    path_available = compute_path(node,planner,args.goal_x,args.goal_y)
    if path_available and not args.proactive:
        print(json.dumps({'state':'path_already_available','motion_performed':False}))
        return
    request.publish(String(data=json.dumps({
        'blocked_goal_map_xy':[args.goal_x,args.goal_y]})))
    start = time.monotonic()
    seen_rotation = False
    state = 'request_not_accepted'
    while time.monotonic()-start<args.seconds:
        rclpy.spin_once(node,timeout_sec=.1)
        if not status:
            continue
        state = status[-1].get('reobservation_state','unknown')
        if state in ('rotating_to_observe','refreshing_costmaps'):
            seen_rotation = True
        if seen_rotation and state == 'reobservation_complete':
            break
        if state in ('rotation_stopped_unsafe','cube_not_reacquired',
                     'reobservation_timed_out','costmap_refresh_failed',
                     'costmap_clear_service_unavailable'):
            break
        if status[-1].get('safety_halt_latched'):
            break
        if not seen_rotation and time.monotonic()-start>3. and state not in (
                'idle','rotating_to_observe'):
            break
    if state == 'reobservation_complete':
        # The supervisor holds motion for two seconds while costmaps refill.
        until = time.monotonic()+3.
        while time.monotonic()<until:
            rclpy.spin_once(node,timeout_sec=.1)
        path_after = compute_path(node,planner,args.goal_x,args.goal_y)
    else:
        path_after = False
    print(json.dumps({'state':state,'observation_rotation_started':seen_rotation,
                      'path_before':path_available,'path_after':path_after,
                      'selected_cube':status[-1].get('reobservation_color')},indent=2))
    node.destroy_node()
    rclpy.shutdown()


if __name__=='__main__':
    main()
