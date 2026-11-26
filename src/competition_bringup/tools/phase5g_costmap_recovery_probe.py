"""No-motion proof of obsolete cube marks and guarded costmap refresh."""
import json
import math
import time
from pathlib import Path

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import ComputePathToPose
from rclpy.action import ActionClient
from nav2_msgs.srv import ClearEntireCostmap
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header, String

rclpy.init()
n = Node('phase5g_costmap_recovery_probe')
n.set_parameters([Parameter('use_sim_time', Parameter.Type.BOOL, True)])
state = {}
q = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
               durability=DurabilityPolicy.TRANSIENT_LOCAL)
for key, topic in [('global', '/global_costmap/costmap'),
                   ('local', '/local_costmap/costmap')]:
    n.create_subscription(OccupancyGrid, topic,
                          lambda m, k=key: state.__setitem__(k, m), q)
n.create_subscription(String, '/nav2/safety_status',
                      lambda m: state.__setitem__('safety', json.loads(m.data)), 10)
pub = n.create_publisher(PointCloud2, '/nav2/cube_obstacles', 10)
planner = ActionClient(n, ComputePathToPose, '/compute_path_to_pose')
global_clear = n.create_client(ClearEntireCostmap,
                               '/global_costmap/clear_entirely_global_costmap')
local_clear = n.create_client(ClearEntireCostmap,
                              '/local_costmap/clear_entirely_local_costmap')


def spin(seconds):
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        rclpy.spin_once(n, timeout_sec=.05)


def cell(key, xy):
    m = state[key]
    x = math.floor((xy[0] - m.info.origin.position.x) / m.info.resolution)
    y = math.floor((xy[1] - m.info.origin.position.y) / m.info.resolution)
    if not (0 <= x < m.info.width and 0 <= y < m.info.height):
        return None
    return int(m.data[y*m.info.width+x])


def publish_disk(radius, seconds=2.0):
    # Exact old green location from 5F, in odom. Test only under motion opt-out.
    cx, cy = .75679323, 1.37422379
    reach = math.ceil(radius/.05)
    points = [(cx+ix*.05, cy+iy*.05, .09)
              for ix in range(-reach, reach+1)
              for iy in range(-reach, reach+1)
              if (ix*.05)**2+(iy*.05)**2 <= radius**2]
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        header = Header()
        header.frame_id = 'odom'
        header.stamp = n.get_clock().now().to_msg()
        pub.publish(point_cloud2.create_cloud_xyz32(header, points))
        spin(.2)


def plan(start, goal):
    if not planner.wait_for_server(timeout_sec=3):
        raise RuntimeError('planner unavailable')
    request = ComputePathToPose.Goal()
    request.use_start = True
    for attr, xy in [('start', start), ('goal', goal)]:
        p = PoseStamped()
        p.header.frame_id = 'map'
        p.header.stamp = n.get_clock().now().to_msg()
        p.pose.position.x, p.pose.position.y = xy
        p.pose.orientation.w = 1.
        setattr(request, attr, p)
    sent = planner.send_goal_async(request)
    end = time.monotonic()+6
    while not sent.done() and time.monotonic()<end:
        rclpy.spin_once(n, timeout_sec=.05)
    if not sent.done() or not sent.result().accepted:
        raise RuntimeError('planning request rejected')
    result = sent.result().get_result_async()
    end = time.monotonic()+8
    while not result.done() and time.monotonic()<end:
        rclpy.spin_once(n, timeout_sec=.05)
    if not result.done():
        raise RuntimeError('planning timeout')
    return {'status': result.result().status,
            'points': len(result.result().result.path.poses)}


spin(8)
if not all(k in state for k in ('global', 'local', 'safety')):
    raise RuntimeError('costmaps or safety status unavailable')
if state['safety']['motion_requested'] or state['safety']['last_command'] != [0., 0.]:
    raise RuntimeError('requires enable_motion:=false and zero command')
if not global_clear.wait_for_service(timeout_sec=3) or not local_clear.wait_for_service(timeout_sec=3):
    raise RuntimeError('clear service unavailable')
start = (.03814725, .34363031)
goal = (.03814725, .89363031)
before = [cell('global', p) for p in (start, goal)]
local_before = [cell('local', p) for p in (start, goal)]
before_plan = plan(start, goal)
publish_disk(.12727922+2.58006316)
large = [cell('global', p) for p in (start, goal)]
local_large = [cell('local', p) for p in (start, goal)]
large_plan = plan(start, goal)
publish_disk(.12727922+.42)
small_without_refresh = [cell('global', p) for p in (start, goal)]
local_small = [cell('local', p) for p in (start, goal)]
small_plan = plan(start, goal)
futures = [global_clear.call_async(ClearEntireCostmap.Request()),
           local_clear.call_async(ClearEntireCostmap.Request())]
end = time.monotonic()+5
while not all(f.done() for f in futures) and time.monotonic()<end:
    rclpy.spin_once(n, timeout_sec=.05)
if not all(f.done() and f.exception() is None for f in futures):
    raise RuntimeError('costmap refresh failed')
spin(2.5)
after = [cell('global', p) for p in (start, goal)]
local_after = [cell('local', p) for p in (start, goal)]
out = {'before': before, 'large_disk': large,
       'small_disk_without_refresh': small_without_refresh,
       'after_stopped_refresh': after,
       'local_cells':{'before':local_before,'large_disk':local_large,
                      'small_disk_without_refresh':local_small,
                      'after_stopped_refresh':local_after},
       'planning': {'before': before_plan, 'large_disk': large_plan,
                    'small_disk_without_refresh': small_plan,
                    'after_refresh': plan(start, goal)},
       'green_layout_center_cost': cell('global', (.8, 1.4)),
       'safety': state['safety']}
Path('reports/phase5g_costmap_recovery_probe.json').write_text(json.dumps(out, indent=2))
print({k:v for k,v in out.items() if k!='safety'})
n.destroy_node()
rclpy.shutdown()
