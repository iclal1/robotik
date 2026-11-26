"""Bounded, motion-disabled cube-aware planning recovery diagnostic."""
import argparse
import json
import math
import time
from pathlib import Path

import rclpy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav2_msgs.action import ComputePathToPose
from nav2_msgs.srv import ClearEntireCostmap
from nav_msgs.msg import OccupancyGrid
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener

parser = argparse.ArgumentParser()
parser.add_argument('--goal-x', type=float, required=True)
parser.add_argument('--goal-y', type=float, required=True)
parser.add_argument('--wait-seconds', type=float, default=5.)
parser.add_argument('--output', type=Path, required=True)
args = parser.parse_args()
if not 0 <= args.wait_seconds <= 10:
    parser.error('wait-seconds must be between 0 and 10')

rclpy.init()
n = Node('phase5g_plan_only_recovery')
n.set_parameters([Parameter('use_sim_time', Parameter.Type.BOOL, True)])
b = Buffer()
listener = TransformListener(b, n)
latest = {}
q = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
               durability=DurabilityPolicy.TRANSIENT_LOCAL)
for topic, key in [('/map', 'map'), ('/global_costmap/costmap', 'global'),
                   ('/local_costmap/costmap', 'local')]:
    n.create_subscription(OccupancyGrid, topic,
                          lambda m, k=key: latest.__setitem__(k, m), q)
n.create_subscription(PoseWithCovarianceStamped, '/amcl_pose',
                      lambda m: latest.__setitem__('amcl', m), q)
for topic, key in [('/nav2/safety_status', 'safety'),
                   ('/cube_detector/status', 'camera')]:
    n.create_subscription(String, topic,
                          lambda m, k=key: latest.__setitem__(k, json.loads(m.data)), 10)
planner = ActionClient(n, ComputePathToPose, '/compute_path_to_pose')
global_clear = n.create_client(ClearEntireCostmap,
                               '/global_costmap/clear_entirely_global_costmap')
local_clear = n.create_client(ClearEntireCostmap,
                              '/local_costmap/clear_entirely_local_costmap')


def spin_until(predicate, seconds):
    end = time.monotonic()+seconds
    while time.monotonic()<end:
        rclpy.spin_once(n, timeout_sec=.05)
        if predicate(): return True
    return False


def pose(xy):
    p = PoseStamped()
    p.header.frame_id = 'map'
    p.header.stamp = n.get_clock().now().to_msg()
    p.pose.position.x, p.pose.position.y = xy
    p.pose.orientation.w = 1.
    return p


def plan(start, goal):
    request = ComputePathToPose.Goal()
    request.start, request.goal, request.use_start = pose(start), pose(goal), True
    sent = planner.send_goal_async(request)
    if not spin_until(sent.done, 5) or not sent.result().accepted:
        return {'status': 'request_failed', 'path': []}
    result = sent.result().get_result_async()
    if not spin_until(result.done, 8):
        return {'status': 'timeout', 'path': []}
    response = result.result()
    return {'status': response.status,
            'path': [[p.pose.position.x, p.pose.position.y]
                     for p in response.result.path.poses]}


def distance_to_segment(point, a, z):
    dx,dy = z[0]-a[0],z[1]-a[1]
    t = 0. if dx*dx+dy*dy == 0 else max(0.,min(1.,
        ((point[0]-a[0])*dx+(point[1]-a[1])*dy)/(dx*dx+dy*dy)))
    return math.dist(point,(a[0]+t*dx,a[1]+t*dy))


def blocking_tracks(start, goal):
    tf = b.lookup_transform('map', 'odom', rclpy.time.Time())
    yaw = math.atan2(2*(tf.transform.rotation.w*tf.transform.rotation.z),
                     1-2*tf.transform.rotation.z**2)
    c,s=math.cos(yaw),math.sin(yaw)
    answer=[]
    for color, track in latest['safety']['cube_obstacles'].items():
        x,y=track['odom_xy']
        center=(tf.transform.translation.x+c*x-s*y,
                tf.transform.translation.y+s*x+c*y)
        radius=.1272792206+track['position_uncertainty_m']+.37
        distance=distance_to_segment(center,start,goal)
        if distance<=radius:
            answer.append({'color':color,'age_s':track['age_s'],
                           'uncertainty_m':track['position_uncertainty_m'],
                           'center_map':center,'route_distance_m':distance,
                           'exclusion_radius_with_robot_m':radius})
    return answer


def full_disk_check(path):
    if not path: return {'safe':False,'reason':'empty_path'}
    for grid_name in ('map','global'):
        grid=latest[grid_name]
        for a,z in zip(path,path[1:]):
            count=max(1,math.ceil(math.dist(a,z)/.025))
            for i in range(count+1):
                t=i/count
                center=(a[0]*(1-t)+z[0]*t,a[1]*(1-t)+z[1]*t)
                for radius in (0.,.05,.10,.15,.20,.25,.30,.35,.37):
                    count_angles=max(1, math.ceil(2*math.pi*radius/.025))
                    for angle in range(count_angles):
                        x=center[0]+radius*math.cos(2*math.pi*angle/count_angles)
                        y=center[1]+radius*math.sin(2*math.pi*angle/count_angles)
                        col=math.floor((x-grid.info.origin.position.x)/grid.info.resolution)
                        row=math.floor((y-grid.info.origin.position.y)/grid.info.resolution)
                        if not(0<=col<grid.info.width and 0<=row<grid.info.height):
                            return {'safe':False,'reason':grid_name+'_outside_map','xy':center}
                        cost=grid.data[row*grid.info.width+col]
                        # Cost 99 is Nav2's footprint-inscribed inflation;
                        # testing the 0.37 m disk against it would count the
                        # robot footprint twice. The disk must avoid lethal
                        # 100 and unknown cells themselves.
                        if (cost!=0 if grid_name=='map' else cost<0 or cost>=100):
                            return {'safe':False,'reason':grid_name+'_obstacle',
                                    'xy':center,'obstacle_sample_xy':[x,y],
                                    'cost':int(cost)}
    return {'safe':True}


def stopped_costmap_refresh():
    safety = latest['safety']
    if safety['motion_requested'] or safety['last_command'] != [0.,0.]:
        return False
    if not global_clear.wait_for_service(timeout_sec=2) or not local_clear.wait_for_service(timeout_sec=2):
        return False
    futures = [global_clear.call_async(ClearEntireCostmap.Request()),
               local_clear.call_async(ClearEntireCostmap.Request())]
    if not spin_until(lambda: all(f.done() for f in futures), 5):
        return False
    if any(f.exception() is not None for f in futures):
        return False
    # Wait for both live obstacle layers to republish after the clear.
    old_global = latest['global'].header.stamp
    old_local = latest['local'].header.stamp
    def refreshed():
        return (latest['global'].header.stamp != old_global and
                latest['local'].header.stamp != old_local)
    return spin_until(refreshed, 3)


if not spin_until(lambda: all(k in latest for k in
                             ('map','global','local','amcl','safety','camera'))
                  and planner.server_is_ready()
                  and b.can_transform('map','odom',rclpy.time.Time()), 30):
    raise RuntimeError('required planner, map, TF or sensor status unavailable')
if latest['safety']['motion_requested'] or latest['safety']['last_command'] != [0.,0.]:
    raise RuntimeError('requires enable_motion:=false and zero command')
start=[latest['amcl'].pose.pose.position.x,latest['amcl'].pose.pose.position.y]
goal=[args.goal_x,args.goal_y]
first=plan(start,goal)
report={'start':start,'goal':goal,'first':{'status':first['status'],
        'points':len(first['path'])},'blocking_tracks':blocking_tracks(start,goal)}
if first['status']==4:
    report['path_check']=full_disk_check(first['path'])
    report['decision']='planning_only_clear' if report['path_check']['safe'] else 'refuse_unsafe_path'
elif report['blocking_tracks']:
    initial_stamp=latest['camera'].get('stamp')
    initial_uncertainty={x['color']:x['uncertainty_m'] for x in report['blocking_tracks']}
    colors=set(initial_uncertainty)
    def new_measurement():
        if latest['camera'].get('stamp')==initial_stamp: return False
        return any(d.get('color') in colors and d.get('valid') and d.get('confirmed')
                   for d in latest['camera'].get('detections',[]))
    report['new_camera_measurement']=spin_until(new_measurement,args.wait_seconds)
    if report['new_camera_measurement']:
        reduced=spin_until(lambda:any(
            latest['safety']['cube_obstacles'].get(c,{}).get('position_uncertainty_m',math.inf)
            < initial_uncertainty[c] for c in colors),2.)
        report['uncertainty_reduced_by_measurement']=reduced
        report['blocking_tracks_after']=blocking_tracks(start,goal)
        retry=plan(start,goal)
        report['retry']={'status':retry['status'],'points':len(retry['path'])}
        if retry['status']!=4 and reduced:
            report['costmap_refresh_succeeded']=stopped_costmap_refresh()
            if report['costmap_refresh_succeeded']:
                retry=plan(start,goal)
                report['retry_after_refresh']={
                    'status':retry['status'],'points':len(retry['path'])}
        if retry['status']==4:
            report['path_check']=full_disk_check(retry['path'])
            report['decision']=('planning_only_clear' if report['path_check']['safe']
                                else 'refuse_unsafe_path')
        else:
            report['decision']='refuse_uncertain_cube_route'
    else:
        report['decision']='refuse_no_new_cube_measurement'
else:
    report['decision']='refuse_non_cube_planning_failure'
args.output.write_text(json.dumps(report,indent=2))
print(report['decision'])
n.destroy_node()
rclpy.shutdown()
