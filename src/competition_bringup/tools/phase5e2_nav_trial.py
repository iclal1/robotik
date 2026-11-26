import argparse, json, math, time
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from std_srvs.srv import Empty
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from nav_msgs.msg import Odometry, OccupancyGrid, Path as NavPath
from sensor_msgs.msg import LaserScan, Image
from std_msgs.msg import String, Bool
from rosgraph_msgs.msg import Clock
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer, TransformListener
from nav2_msgs.action import ComputePathToPose, NavigateToPose
from nav2_msgs.msg import ParticleCloud
from action_msgs.msg import GoalStatus
from nav_trial_monitor import AsyncGazeboSampler, sensor_ages

def yaw(q): return math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
def pose(p): return [p.position.x,p.position.y,yaw(p.orientation)]
def stamp(m): return m.sec+m.nanosec*1e-9
def angle(a,b): return abs(math.atan2(math.sin(a-b),math.cos(a-b)))

p=argparse.ArgumentParser()
p.add_argument('--goal-x',type=float)
p.add_argument('--goal-y',type=float)
p.add_argument('--goal-yaw',type=float)
p.add_argument('--delta-x',type=float)
p.add_argument('--delta-y',type=float)
p.add_argument('--delta-yaw',type=float)
p.add_argument('--max-forward',type=float,default=.25)
p.add_argument('--max-lateral',type=float,default=.12)
p.add_argument('--max-yaw',type=float,default=.35)
p.add_argument('--seconds',type=float,default=18)
p.add_argument('--output',type=Path,required=True)
a=p.parse_args()
if (a.goal_x is None) == (a.delta_x is None) or (a.goal_y is None) == (a.delta_y is None) or (a.goal_yaw is None) == (a.delta_yaw is None):
    p.error('provide exactly one absolute or relative value for each goal coordinate')
rclpy.init(); n=Node('phase5d_bounded_trial')
b=Buffer(); l=TransformListener(b,n)
gzs=AsyncGazeboSampler()
latest={}; samples={k:[] for k in ('amcl','odom','clock','scan','camera','detector','safety','cmd','drive','plan','tf','gazebo','feedback','global','local','particles','tf_health','sensor_health')}
def record(key, value):
    latest[key]=value
    samples[key].append(value)
    if len(samples[key])>3000: samples[key]=samples[key][-3000:]
def amcl(m): record('amcl',{'t':stamp(m.header.stamp),'pose':pose(m.pose.pose),'cov':[m.pose.covariance[i] for i in (0,7,35)],'wall':time.monotonic()})
def odom(m): record('odom',{'t':stamp(m.header.stamp),'pose':pose(m.pose.pose),'v':m.twist.twist.linear.x,'w':m.twist.twist.angular.z,'wall':time.monotonic()})
n.create_subscription(PoseWithCovarianceStamped,'/amcl_pose',amcl,QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL))
n.create_subscription(Odometry,'/odom',odom,10)
n.create_subscription(Clock,'/clock',lambda m:(record('clock',stamp(m.clock)),latest.update({'clock_wall':time.monotonic()})),10)
n.create_subscription(LaserScan,'/scan',lambda m:record('scan',{'t':stamp(m.header.stamp),'wall':time.monotonic(),'n':len(m.ranges)}),qos_profile_sensor_data)
n.create_subscription(Image,'/camera/image_raw',lambda m:record('camera',{'t':stamp(m.header.stamp),'wall':time.monotonic()}),qos_profile_sensor_data)
n.create_subscription(String,'/cube_detector/status',lambda m:record('detector',json.loads(m.data)),10)
n.create_subscription(String,'/nav2/safety_status',lambda m:record('safety',dict(json.loads(m.data),_received_wall=time.monotonic())),10)
n.create_subscription(Twist,'/cmd_vel',lambda m:record('cmd',[time.monotonic(),m.linear.x,m.angular.z]),10)
n.create_subscription(Twist,'/model/pusher_robot/cmd_vel',lambda m:record('drive',[time.monotonic(),m.linear.x,m.angular.z]),10)
n.create_subscription(NavPath,'/plan',lambda m:record('plan',[[v.pose.position.x,v.pose.position.y] for v in m.poses]),10)
n.create_subscription(TFMessage,'/tf',lambda m:record('tf',[[t.header.frame_id,t.child_frame_id,stamp(t.header.stamp)] for t in m.transforms]),10)
n.create_subscription(ParticleCloud,'/particle_cloud',lambda m:record('particles',{'t':stamp(m.header.stamp),'n':len(m.particles)}),qos_profile_sensor_data)
mapdata={}
def onmap(m):
    mapdata.update({'w':m.info.width,'h':m.info.height,'res':m.info.resolution,'origin':(m.info.origin.position.x,m.info.origin.position.y),'data':list(m.data)})
n.create_subscription(OccupancyGrid,'/map',onmap,QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL))
for key,topic in (('global','/global_costmap/costmap'),('local','/local_costmap/costmap')):
    n.create_subscription(OccupancyGrid,topic,lambda m,k=key:record(k,{'t':stamp(m.header.stamp),'w':m.info.width,'h':m.info.height}),10)
nomotion=n.create_client(Empty,'/request_nomotion_update')
planner=ActionClient(n,ComputePathToPose,'/compute_path_to_pose')
nav=ActionClient(n,NavigateToPose,'/navigate_to_pose')
stop_pub=n.create_publisher(Bool,'/emergency_stop',10)
def spin_until(cond,deadline):
    while time.monotonic()<deadline:
        rclpy.spin_once(n,timeout_sec=.05)
        sample=gzs.tick()
        if sample is not None:
            record('gazebo',{'t':sample['finished_wall'],'sim':latest.get('clock'),
                             'query_duration_wall_s':sample['duration_wall_s'],
                             'poses':{k:sample['poses'][k] for k in ('pusher_robot','red_cube','green_cube','blue_cube')}})
        if cond(): return True
    return False
def tfcheck():
    now=latest.get('clock',0)
    out={}
    for target,source in (('map','odom'),('odom','base_footprint'),('map','base_footprint')):
        try:
            t=b.lookup_transform(target,source,Time())
            q=t.transform.rotation;v=t.transform.translation
            out[target+'_'+source]={'t':stamp(t.header.stamp),'age':now-stamp(t.header.stamp),'pose':[v.x,v.y,yaw(q)]}
        except Exception as e: out[target+'_'+source]={'error':str(e)}
    return out
def clear_disk_path(start,end):
    # Conservative 0.37 m rotating sweep; every sampled map cell must be free.
    for i in range(31):
        t=i/30
        x=start[0]*(1-t)+end[0]*t;y=start[1]*(1-t)+end[1]*t
        for j in range(72):
            ang=2*math.pi*j/72
            xx=x+.37*math.cos(ang);yy=y+.37*math.sin(ang)
            col=math.floor((xx-mapdata['origin'][0])/mapdata['res']);row=math.floor((yy-mapdata['origin'][1])/mapdata['res'])
            if not(0<=col<mapdata['w'] and 0<=row<mapdata['h']): return False
            if mapdata['data'][row*mapdata['w']+col]!=0: return False
    return True
def snapshot():
    gz=latest.get('gazebo')
    return {'clock':latest.get('clock'),'amcl':latest.get('amcl'),'odom':latest.get('odom'),'safety':latest.get('safety'),'detector':latest.get('detector'),'tf':tfcheck(),'gazebo':None if gz is None else gz['poses']}
def emergency():
    for _ in range(12):
        stop_pub.publish(Bool(data=True));rclpy.spin_once(n,timeout_sec=.1)

report={'goal':None,'timeout_s':a.seconds,'tolerances_predeclared':{'position_m':0.10,'orientation_rad':0.20,'estimator_displacement_m':0.08,'cube_displacement_m':0.005,'max_linear_mps':0.105,'max_angular_radps':0.26,'envelope_forward_m':a.max_forward,'envelope_lateral_m':a.max_lateral,'envelope_yaw_rad':a.max_yaw,'tf_age_s':0.7,'sensor_receipt_and_stamp_s':{'odom':.3,'scan':.4,'camera':.6}}}
handle=None
try:
    deadline=time.monotonic()+15
    needed=('clock','amcl','odom','scan','camera','detector','safety','global','local')
    if not spin_until(lambda:all(k in latest for k in needed) and bool(mapdata) and 'gazebo' in latest,deadline): raise RuntimeError('preflight_inputs_missing')
    if not nomotion.wait_for_service(timeout_sec=2): raise RuntimeError('nomotion_service_missing')
    old=latest['amcl']['t']; f=nomotion.call_async(Empty.Request())
    if not spin_until(lambda:f.done() and latest['amcl']['t']>old,time.monotonic()+5): raise RuntimeError('amcl_refresh_failed')
    # AMCL intentionally stamps map->odom 0.3 s ahead. A newly created TF
    # buffer needs odom history to reach that stamp before a combined lookup.
    def tf_ready():
        state=tfcheck()
        return all('error' not in v and abs(v['age'])<.7 for v in state.values())
    if not spin_until(tf_ready,time.monotonic()+2): raise RuntimeError('tf_initialization_timeout')
    start=snapshot();report['start']=start
    a.goal_x = a.goal_x if a.goal_x is not None else start['amcl']['pose'][0]+a.delta_x
    a.goal_y = a.goal_y if a.goal_y is not None else start['amcl']['pose'][1]+a.delta_y
    a.goal_yaw = a.goal_yaw if a.goal_yaw is not None else start['amcl']['pose'][2]+a.delta_yaw
    report['goal']=[a.goal_x,a.goal_y,a.goal_yaw]
    now=time.monotonic(); sim=latest['clock']
    ages,failures=sensor_ages(latest,now,sim,{'odom':.3,'scan':.4,'camera':.6})
    report['preflight_sensor_ages']=ages
    if failures: raise RuntimeError('preflight_'+','.join(failures))
    if sim-latest['amcl']['t']>.6: raise RuntimeError('stale_amcl_pose')
    if any(x<0 or x>.25 for x in latest['amcl']['cov']): raise RuntimeError('bad_covariance')
    s=latest['safety']
    if not s['motion_requested'] or s['emergency_latched'] or s['safety_halt_latched'] or any(abs(x)>1e-4 for x in s['last_command']): raise RuntimeError('safety_not_ready')
    if latest['detector']['perception_state']=='invalid_perception': raise RuntimeError('camera_invalid')
    if any(abs(x[1])>1e-4 or abs(x[2])>1e-4 for x in samples['drive'][-10:]): raise RuntimeError('drive_nonzero')
    tf=tfcheck()
    if any('error' in v or abs(v['age'])>.7 for v in tf.values()): raise RuntimeError('tf_invalid')
    if not clear_disk_path(latest['amcl']['pose'],(a.goal_x,a.goal_y)): raise RuntimeError('swept_map_disk_unsafe')
    g0=start['gazebo']['pusher_robot']
    if time.monotonic()-latest['gazebo']['t']>.7: raise RuntimeError('gazebo_pose_stale')
    cube0={k:start['gazebo'][k] for k in ('red_cube','green_cube','blue_cube')}
    for i in range(31):
        t=i/30;wx=g0[0]+(a.goal_x-start['amcl']['pose'][0])*t;wy=g0[1]+(a.goal_y-start['amcl']['pose'][1])*t
        if min(wx+2.95,2.95-wx,wy+1.95,1.95-wy)-.37<.12: raise RuntimeError('wall_clearance_low')
        for name,c in cube0.items():
            if math.dist((wx,wy),c[:2])-(.37+math.sqrt(2)*.09+.12)<.02: raise RuntimeError('cube_clearance_low_'+name)
    if not planner.wait_for_server(timeout_sec=3) or not nav.wait_for_server(timeout_sec=3): raise RuntimeError('nav_action_missing')
    def mkpose():
        from geometry_msgs.msg import PoseStamped
        q=PoseStamped();q.header.frame_id='map';q.header.stamp=n.get_clock().now().to_msg();q.pose.position.x=a.goal_x;q.pose.position.y=a.goal_y;q.pose.orientation.z=math.sin(a.goal_yaw/2);q.pose.orientation.w=math.cos(a.goal_yaw/2);return q
    pg=ComputePathToPose.Goal();pg.goal=mkpose();pg.use_start=False
    sent=planner.send_goal_async(pg)
    if not spin_until(lambda:sent.done(),time.monotonic()+5) or not sent.result().accepted: raise RuntimeError('plan_rejected')
    pf=sent.result().get_result_async()
    if not spin_until(lambda:pf.done(),time.monotonic()+5) or pf.result().status!=GoalStatus.STATUS_SUCCEEDED: raise RuntimeError('plan_failed')
    path=[[v.pose.position.x,v.pose.position.y] for v in pf.result().result.path.poses]
    report['planned_path']=path
    for u,v in zip(path,path[1:]):
        if not clear_disk_path(u,v): raise RuntimeError('planned_sweep_unsafe')
    if not path: raise RuntimeError('empty_path')
    goal=NavigateToPose.Goal();goal.pose=mkpose()
    sent=nav.send_goal_async(goal,feedback_callback=lambda m:record('feedback',{'t':time.monotonic(),'distance_remaining':m.feedback.distance_remaining,'navigation_time_s':stamp(m.feedback.navigation_time)}))
    if not spin_until(lambda:sent.done(),time.monotonic()+5): raise RuntimeError('goal_send_timeout')
    handle=sent.result();report['accepted']=handle.accepted
    if not handle.accepted: raise RuntimeError('navigation_goal_rejected')
    result=handle.get_result_async();started=time.monotonic();reason=None
    while not result.done() and time.monotonic()-started<a.seconds:
        rclpy.spin_once(n,timeout_sec=.05)
        sample=gzs.tick()
        if sample is not None:
            gz=sample['poses'];record('gazebo',{'t':sample['finished_wall'],'sim':latest.get('clock'),
                'query_duration_wall_s':sample['duration_wall_s'],
                'poses':{k:gz[k] for k in ('pusher_robot','red_cube','green_cube','blue_cube')}})
        now=time.monotonic();s=latest.get('safety',{})
        if s.get('emergency_latched') or s.get('safety_halt_latched'): reason='safety_latch';break
        if now-latest.get('clock_wall',0)>.3:reason='clock_receipt_stale';break
        ages,failures=sensor_ages(latest,now,latest['clock'],{'odom':.3,'scan':.4,'camera':.6})
        record('sensor_health',{'sim':latest['clock'],'wall':now,'ages':ages,'failures':failures})
        if failures: reason=','.join(failures);break
        if now-s.get('_received_wall',0)>1.5:reason='safety_status_stale';break
        if latest['detector']['perception_state']=='invalid_perception': reason='camera_invalid';break
        if any(x<0 or x>.25 for x in latest['amcl']['cov']): reason='covariance_invalid';break
        tf=tfcheck()
        record('tf_health',{'sim':latest['clock'],'ages':{k:v.get('age') for k,v in tf.items()},'valid':all('error' not in v for v in tf.values())})
        if any('error' in v or abs(v['age'])>.7 for v in tf.values()): reason='tf_invalid';break
        od=latest['odom']['pose'];o0=start['odom']['pose']
        if math.dist(od[:2],o0[:2])>.04 and latest['amcl']['t']<=start['amcl']['t']:
            reason='amcl_no_dynamic_update';break
        if abs(od[0]-o0[0])>a.max_forward or abs(od[1]-o0[1])>a.max_lateral or angle(od[2],o0[2])>a.max_yaw: reason='odom_envelope';break
        cmd=latest.get('drive',[0,0,0])
        if abs(cmd[1])>.105 or abs(cmd[2])>.26: reason='speed_limit';break
        gz_sample=latest.get('gazebo')
        if gz_sample is None or now-gz_sample['t']>2.5:reason='gazebo_measurement_stale';break
        gz=gz_sample['poses'];rb=gz['pusher_robot']
        if abs(rb[0]-g0[0])>a.max_forward or abs(rb[1]-g0[1])>a.max_lateral: reason='gazebo_envelope';break
        if any(math.dist(gz[k][:2],cube0[k][:2])>.005 for k in cube0): reason='cube_moved';break
    if not result.done() and reason is None:reason='timeout'
    report['interruption']=reason
    if reason:
        cf=handle.cancel_goal_async();spin_until(lambda:cf.done(),time.monotonic()+2);emergency()
    spin_until(lambda:result.done(),time.monotonic()+3)
    report['status']=result.result().status if result.done() else None
    report['duration_s']=time.monotonic()-started
    spin_until(lambda:False,time.monotonic()+1.0)
    report['end']=snapshot()
    report['stopped']=abs(latest['odom']['v'])<.002 and abs(latest['odom']['w'])<.01 and abs(latest['drive'][1])<1e-4 and abs(latest['drive'][2])<1e-4
    if not report['stopped']:
        report['interruption']='stop_not_verified'
        emergency()
    final_cube_drift=max(math.dist(report['end']['gazebo'][k][:2],cube0[k][:2]) for k in cube0)
    report['max_final_cube_displacement_m']=final_cube_drift
    if final_cube_drift>.005:
        report['interruption']='cube_moved_after_goal'
        emergency()
except Exception as e:
    report['error']=str(e)
    if handle is not None and handle.accepted:
        try:
            cf=handle.cancel_goal_async();spin_until(lambda:cf.done(),time.monotonic()+2)
        finally:
            emergency()
finally:
    report['samples']=samples
    a.output.parent.mkdir(parents=True,exist_ok=True)
    a.output.write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps({k:v for k,v in report.items() if k!='samples'},indent=2))
    gzs.close();n.destroy_node();rclpy.shutdown()
