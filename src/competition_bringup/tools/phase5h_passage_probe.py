"""Four direction planning-only passage trials with polygon sweep evidence."""
import json
import math
import time
from pathlib import Path

import rclpy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav2_msgs.action import ComputePathToPose
from nav_msgs.msg import OccupancyGrid
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener

from footprint_clearance import rotated_polygon, square, validate_sweep


def grid(m):
    return {'width':m.info.width,'height':m.info.height,
            'resolution':m.info.resolution,
            'origin':[m.info.origin.position.x,m.info.origin.position.y],
            'data':list(m.data)}


def yaw(q):
    return math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))


rclpy.init()
n=Node('phase5h_passage_probe')
n.set_parameters([Parameter('use_sim_time',Parameter.Type.BOOL,True)])
b=Buffer();listener=TransformListener(b,n)
latest={}
q=QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,
             durability=DurabilityPolicy.TRANSIENT_LOCAL)
for key,topic in [('map','/map'),('global','/global_costmap/costmap'),
                  ('local','/local_costmap/costmap')]:
    n.create_subscription(OccupancyGrid,topic,
        lambda m,k=key:latest.__setitem__(k,grid(m)),q)
n.create_subscription(PoseWithCovarianceStamped,'/amcl_pose',
    lambda m:latest.__setitem__('amcl',{
        'xy':[m.pose.pose.position.x,m.pose.pose.position.y],
        'variance':[m.pose.covariance[i] for i in (0,7,35)]}),q)
n.create_subscription(String,'/nav2/safety_status',
    lambda m:latest.__setitem__('safety',json.loads(m.data)),10)
planner=ActionClient(n,ComputePathToPose,'/compute_path_to_pose')


def until(pred,seconds):
    end=time.monotonic()+seconds
    while time.monotonic()<end:
        rclpy.spin_once(n,timeout_sec=.05)
        if pred():return True
    return False


def pose(x,y,heading):
    p=PoseStamped();p.header.frame_id='map'
    p.header.stamp=n.get_clock().now().to_msg()
    p.pose.position.x=x;p.pose.position.y=y
    p.pose.orientation.z=math.sin(heading/2)
    p.pose.orientation.w=math.cos(heading/2)
    return p


def request(start,goal):
    g=ComputePathToPose.Goal();g.start=pose(*start);g.goal=pose(*goal);g.use_start=True
    sent=planner.send_goal_async(g)
    if not until(sent.done,5) or not sent.result().accepted:
        return {'status':'request_failed','poses':[]}
    result=sent.result().get_result_async()
    if not until(result.done,8):return {'status':'timeout','poses':[]}
    output=result.result();path=output.result.path
    return {'status':output.status,'poses':[
        (p.pose.position.x,p.pose.position.y,yaw(p.pose.orientation))
        for p in path.poses]}


if not until(lambda:all(k in latest for k in ('map','global','local','amcl','safety'))
             and planner.server_is_ready()
             and b.can_transform('map','odom',rclpy.time.Time()),30):
    raise RuntimeError('planner/costmap/TF unavailable')
if latest['safety']['motion_requested'] or latest['safety']['last_command']!=[0.,0.]:
    raise RuntimeError('enable_motion:=false and zero command required')
sigma=max(math.sqrt(max(latest['amcl']['variance'][:2])),math.sqrt(.006603216))
yaw_bound=2*math.sqrt(latest['amcl']['variance'][2])
live_margin=.10+2*sigma
map_margin=live_margin+.05
tf=b.lookup_transform('map','odom',rclpy.time.Time())
c,s=math.cos(yaw(tf.transform.rotation)),math.sin(yaw(tf.transform.rotation))
def map_xy(x,y):return (tf.transform.translation.x+c*x-s*y,
                        tf.transform.translation.y+s*x+c*y)
cube_centers={name:map_xy(.8,y) for name,y in
              [('blue',.2),('green',1.4),('red',2.6)]}
cubes=[(name,rotated_polygon(square(-.09,-.09,.18),
       (center[0],center[1],yaw(tf.transform.rotation))))
       for name,center in cube_centers.items()]

# Exact mapped opening centers from the unchanged 0.05 m map: A 0.882–1.932,
# B 0.432–1.582. Staging poses stay outside the wall thickness.
scenarios={
    'current_to_a_approach':((latest['amcl']['xy'][0],latest['amcl']['xy'][1],0.),
                             (1.65,1.407,0.)),
    'a_west_to_central':((1.65,1.407,0.),(2.55,1.407,0.)),
    'a_central_to_west':((2.55,1.407,math.pi),(1.65,1.407,math.pi)),
    'b_central_to_east':((2.75,1.007,0.),(3.85,1.007,0.)),
    'b_east_to_central':((3.85,1.007,math.pi),(2.75,1.007,math.pi)),
    'central_connector':((2.55,1.407,0.),(2.75,1.007,0.)),
    'central_connector_return':((2.75,1.007,math.pi),(2.55,1.407,math.pi)),
    'connector_east_half_1':((2.55,1.407,0.),
                             (2.65,1.207,-1.1071487178)),
    'connector_east_half_2':((2.65,1.207,-1.1071487178),
                             (2.75,1.007,0.)),
    'connector_west_half_1':((2.75,1.007,math.pi),
                             (2.65,1.207,2.0344439358)),
    'connector_west_half_2':((2.65,1.207,2.0344439358),
                             (2.55,1.407,math.pi)),
}
report={'amcl':latest['amcl'],'safety':latest['safety'],
        'margin':{'one_axis_sigma_m':sigma,'engineering_m':.10,
                  'map_cell_allowance_m':.05,'map_m':map_margin,
                  'live_and_cube_m':live_margin,
                  'two_sigma_yaw_rad':yaw_bound},
        'map_to_odom':{'xy':[tf.transform.translation.x,tf.transform.translation.y],
                       'yaw':yaw(tf.transform.rotation)},
        'cube_centers_map':cube_centers,
        'grid_sizes':{k:[latest[k]['width'],latest[k]['height']] for k in
                      ('map','global','local')},'scenarios':{}}
report['grid_origins']={k:latest[k]['origin'] for k in
                        ('map','global','local')}
for name,(start,goal) in scenarios.items():
    result=request(start,goal)
    poses=result['poses']
    record={'start':start,'goal':goal,'status':result['status'],
            'path_poses':poses,
            'path_length_m':sum(math.dist(a[:2],z[:2]) for a,z in zip(poses,poses[1:])),
            'max_segment_m':max((math.dist(a[:2],z[:2]) for a,z in
                                 zip(poses,poses[1:])),default=0.),
            'heading_range_rad':[min((p[2] for p in poses),default=None),
                                 max((p[2] for p in poses),default=None)]}
    if poses:
        record['polygon_check']=validate_sweep(
            poses,{'map':latest['map'],'global':latest['global']},cubes,
            map_margin=map_margin,live_margin=live_margin,cube_margin=live_margin)
        # Compare the Navfn path with a straight, aligned geometric corridor.
        record['aligned_corridor_check']=validate_sweep(
            [start,goal],{'map':latest['map'],'global':latest['global']},cubes,
            map_margin=map_margin,live_margin=live_margin,cube_margin=live_margin)
        # Navfn in Humble returns 0 rad along a westbound path, then puts
        # pi only on the final goal. Validate a separately staged facing
        # direction, including the necessary initial in-place rotation.
        heading=math.atan2(goal[1]-start[1],goal[0]-start[0])
        directional=[(start[0],start[1],start[2]),
                     (start[0],start[1],heading)]
        directional.extend((x,y,heading) for x,y,_ in poses)
        directional.extend(((goal[0],goal[1],heading),
                            (goal[0],goal[1],goal[2])))
        record['directional_candidate_check']=validate_sweep(
            directional,{'map':latest['map'],'global':latest['global']},cubes,
            map_margin=map_margin,live_margin=live_margin,cube_margin=live_margin)
        if name.startswith(('a_','b_')):
            record['robust_directional_check']=validate_sweep(
                directional,{'map':latest['map'],'global':latest['global']},cubes,
                map_margin=map_margin,live_margin=live_margin,
                cube_margin=live_margin,yaw_uncertainty=yaw_bound)
        if name.startswith('central_connector'):
            record['straight_connector_check']=validate_sweep(
                [start,(start[0],start[1],heading),
                 (goal[0],goal[1],heading),goal],
                {'map':latest['map'],'global':latest['global']},cubes,
                map_margin=map_margin,live_margin=live_margin,cube_margin=live_margin)
    report['scenarios'][name]=record
    print(name,result['status'],len(poses),
          record.get('polygon_check',{}).get('safe'),
          record.get('aligned_corridor_check',{}).get('safe'),flush=True)

report['candidate_routes']={}
for name,parts,sequence in (
        ('west_to_east',('a_west_to_central','central_connector',
                         'b_central_to_east'),
         [(1.65,1.407,0.),(2.55,1.407,0.),
          (2.55,1.407,-1.1071487178),(2.75,1.007,-1.1071487178),
          (2.75,1.007,0.),(3.85,1.007,0.)]),
        ('east_to_west',('b_east_to_central','central_connector_return',
                         'a_central_to_west'),
         [(3.85,1.007,math.pi),(2.75,1.007,math.pi),
          (2.75,1.007,2.0344439358),(2.55,1.407,2.0344439358),
          (2.55,1.407,math.pi),(1.65,1.407,math.pi)])):
    report['candidate_routes'][name]={
        'parts':parts,'method':'straight drive segments with in-place staging rotations',
        'poses':sequence,
        'polygon_check':validate_sweep(
            sequence,{'map':latest['map'],'global':latest['global']},cubes,
            map_margin=map_margin,live_margin=live_margin,cube_margin=live_margin),
        'robust_check':validate_sweep(
            sequence,{'map':latest['map'],'global':latest['global']},cubes,
            map_margin=map_margin,live_margin=live_margin,
            cube_margin=live_margin,yaw_uncertainty=yaw_bound)}
Path('reports/phase5h_passage_probe.json').write_text(json.dumps(report,indent=2))
n.destroy_node();rclpy.shutdown()
