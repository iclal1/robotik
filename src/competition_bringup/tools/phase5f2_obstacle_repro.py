"""Reproduce the historical large cube exclusion in a motion-disabled session."""
import json
import math
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
from nav2_msgs.action import ComputePathToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.parameter import Parameter
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header, String

rclpy.init()
n = Node('phase5f2_obstacle_repro')
n.set_parameters([Parameter('use_sim_time', Parameter.Type.BOOL, True)])
pub = n.create_publisher(PointCloud2, '/nav2/cube_obstacles', 10)
q = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
               durability=DurabilityPolicy.TRANSIENT_LOCAL)
state = {}


def callback(m):
    state['grid'] = {'origin': [m.info.origin.position.x, m.info.origin.position.y],
                     'resolution': m.info.resolution, 'width': m.info.width,
                     'height': m.info.height, 'data': list(m.data)}


n.create_subscription(OccupancyGrid, '/global_costmap/costmap', callback, q)
n.create_subscription(String, '/nav2/safety_status',
                      lambda m: state.__setitem__('safety', json.loads(m.data)), 10)
client = ActionClient(n, ComputePathToPose, '/compute_path_to_pose')
start = (.03814725491106844, .34363031046635345)
goal = (.03814725491106844, .8936303104663534)


def cell(p):
    g = state['grid']
    x = int((p[0] - g['origin'][0]) // g['resolution'])
    y = int((p[1] - g['origin'][1]) // g['resolution'])
    return g['data'][y * g['width'] + x]


def pose(p):
    v = PoseStamped()
    v.header.frame_id = 'map'
    v.header.stamp = n.get_clock().now().to_msg()
    v.pose.position.x, v.pose.position.y = p
    v.pose.orientation.w = 1.
    return v


def spin_until(pred, seconds):
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        rclpy.spin_once(n, timeout_sec=.05)
        if pred(): return True
    return False


if not spin_until(lambda: 'grid' in state and 'safety' in state
                  and client.server_is_ready(), 20):
    raise RuntimeError('not ready')
if state['safety']['motion_requested'] or state['safety']['last_command'] != [0.0, 0.0]:
    raise RuntimeError('reproduction requires enable_motion:=false and stopped robot')
before = [cell(start), cell(goal)]
# Historical odom-frame green track (from Test B) with its 2.580 m uncertainty.
# Use the same radial point spacing as the production cube cloud.
center = (.7567932299839926, 1.3742237936163129)
radius = .1272792206 + 2.580063164807708
points = []
for ix in range(-math.ceil(radius/.05), math.ceil(radius/.05)+1):
    for iy in range(-math.ceil(radius/.05), math.ceil(radius/.05)+1):
        x,y = ix*.05, iy*.05
        if x*x+y*y <= radius*radius:
            points.append((center[0]+x,center[1]+y,0.1))
for _ in range(7):
    header = Header()
    header.frame_id = 'odom'
    header.stamp = n.get_clock().now().to_msg()
    pub.publish(point_cloud2.create_cloud_xyz32(header, points))
    spin_until(lambda: False, .3)
after = [cell(start), cell(goal)]
request = ComputePathToPose.Goal()
request.start, request.goal, request.use_start = pose(start), pose(goal), True
sent = client.send_goal_async(request)
if not spin_until(sent.done, 8): raise RuntimeError('send timeout')
result = sent.result().get_result_async()
if not spin_until(result.done, 12): raise RuntimeError('result timeout')
out = {'before_start_goal_cost': before, 'after_start_goal_cost': after,
       'radius_m': radius, 'point_count': len(points),
       'status': result.result().status,
       'path_points': len(result.result().result.path.poses)}
open('reports/phase5f2_obstacle_repro.json','w').write(json.dumps(out,indent=2))
print(out)
n.destroy_node()
rclpy.shutdown()
