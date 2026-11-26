"""Planning-only Nav2 probe: save costmaps and exact planner outcomes."""
import json
import math
import time
from pathlib import Path

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.parameter import Parameter
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid
from nav2_msgs.action import ComputePathToPose
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener


def stamped(node, xy):
    p = PoseStamped()
    p.header.frame_id = 'map'
    p.header.stamp = node.get_clock().now().to_msg()
    p.pose.position.x, p.pose.position.y = xy
    p.pose.orientation.w = 1.0
    return p


def grid(m):
    return {'width': m.info.width, 'height': m.info.height,
            'resolution': m.info.resolution,
            'origin': [m.info.origin.position.x, m.info.origin.position.y],
            'data': list(m.data)}


def cell(g, p):
    x = math.floor((p[0] - g['origin'][0]) / g['resolution'])
    y = math.floor((p[1] - g['origin'][1]) / g['resolution'])
    if not (0 <= x < g['width'] and 0 <= y < g['height']):
        return {'index': [x, y], 'value': None}
    return {'index': [x, y], 'value': g['data'][y * g['width'] + x]}


rclpy.init()
n = Node('phase5f2_planning_probe')
n.set_parameters([Parameter('use_sim_time', Parameter.Type.BOOL, True)])
b = Buffer()
listener = TransformListener(b, n)
latest = {}
q = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
               durability=DurabilityPolicy.TRANSIENT_LOCAL)
for topic, key in [('/map', 'map'), ('/global_costmap/costmap', 'global'),
                   ('/local_costmap/costmap', 'local')]:
    n.create_subscription(OccupancyGrid, topic,
                          lambda m, k=key: latest.__setitem__(k, grid(m)), q)
n.create_subscription(PoseWithCovarianceStamped, '/amcl_pose',
                      lambda m: latest.__setitem__('amcl',
                          [m.pose.pose.position.x, m.pose.pose.position.y,
                           m.pose.covariance[0], m.pose.covariance[7],
                           m.pose.covariance[35]]), q)
n.create_subscription(String, '/nav2/safety_status',
                      lambda m: latest.__setitem__('safety', json.loads(m.data)), 10)
client = ActionClient(n, ComputePathToPose, '/compute_path_to_pose')


def until(predicate, seconds=15):
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        rclpy.spin_once(n, timeout_sec=.05)
        if predicate():
            return True
    return False


if not until(lambda: all(k in latest for k in
                         ('map', 'global', 'local', 'amcl', 'safety'))
             and client.server_is_ready()
             and b.can_transform('map', 'base_footprint', rclpy.time.Time()), 30):
    raise RuntimeError('missing inputs: ' + str(latest.keys()))

starts = {'current': latest['amcl'][:2], 'failed_5f': [.03814725491106844, .34363031046635345]}
goals = {'failed_5f': [.03814725491106844, .8936303104663534],
         'open_north': [.0, 1.0], 'wall_a_other_side': [2.7, 1.4],
         'wall_b_other_side': [3.8, 1.4]}
report = {'starts': starts, 'goals': goals, 'amcl': latest['amcl'],
          'safety': latest['safety'], 'grids': {k: latest[k] for k in ('map', 'global', 'local')},
          'trials': []}
try:
    tf = b.lookup_transform('map', 'base_footprint', rclpy.time.Time())
    report['tf'] = {'stamp': [tf.header.stamp.sec, tf.header.stamp.nanosec],
                    'translation': [tf.transform.translation.x, tf.transform.translation.y]}
except Exception as exc:
    report['tf_error'] = str(exc)
for start_name, goal_name in [('current', 'failed_5f'), ('failed_5f', 'failed_5f'),
                              ('current', 'open_north'), ('current', 'wall_a_other_side'),
                              ('current', 'wall_b_other_side')]:
    start, goal = starts[start_name], goals[goal_name]
    trial = {'start': start_name, 'goal': goal_name,
             'cells': {k: {'start': cell(latest[k], start),
                           'goal': cell(latest[k], goal)} for k in ('map', 'global', 'local')}}
    req = ComputePathToPose.Goal()
    req.start = stamped(n, start)
    req.goal = stamped(n, goal)
    req.use_start = True
    sent = client.send_goal_async(req)
    if not until(sent.done, 8):
        trial['error'] = 'send_timeout'
    elif not sent.result().accepted:
        trial['error'] = 'rejected'
    else:
        result = sent.result().get_result_async()
        if not until(result.done, 12):
            trial['error'] = 'result_timeout'
        else:
            response = result.result()
            trial['status'] = response.status
            trial['error_code'] = getattr(response.result, 'error_code', None)
            trial['path'] = [[p.pose.position.x, p.pose.position.y]
                             for p in response.result.path.poses]
    report['trials'].append(trial)
    print(start_name, goal_name, trial.get('status'), trial.get('error_code'), len(trial.get('path', [])), flush=True)
Path('reports/phase5f2_planning_probe.json').write_text(json.dumps(report))
n.destroy_node()
rclpy.shutdown()
