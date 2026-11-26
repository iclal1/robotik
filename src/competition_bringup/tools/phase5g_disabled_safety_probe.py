"""Verify stationary AMCL, camera and emergency latch under motion opt-out."""
import json
import time
from pathlib import Path

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String
from tf2_ros import Buffer, TransformListener

rclpy.init()
n=Node('phase5g_disabled_safety_probe')
n.set_parameters([Parameter('use_sim_time',Parameter.Type.BOOL,True)])
b=Buffer()
listener=TransformListener(b,n)
state={}
q=QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,
             durability=DurabilityPolicy.TRANSIENT_LOCAL)
n.create_subscription(PoseWithCovarianceStamped,'/amcl_pose',
    lambda m:state.__setitem__('amcl',{'stamp':[m.header.stamp.sec,m.header.stamp.nanosec],
        'xy':[m.pose.pose.position.x,m.pose.pose.position.y],
        'variance':[m.pose.covariance[i] for i in (0,7,35)]}),q)
for topic,key in [('/map','map'),('/global_costmap/costmap','global'),
                  ('/local_costmap/costmap','local')]:
    n.create_subscription(OccupancyGrid,topic,
        lambda m,k=key:state.__setitem__(k,[m.info.width,m.info.height]),q)
for topic,key in [('/nav2/safety_status','safety'),
                  ('/cube_detector/status','camera')]:
    n.create_subscription(String,topic,
        lambda m,k=key:state.__setitem__(k,json.loads(m.data)),10)
n.create_subscription(Twist,'/model/pusher_robot/cmd_vel',
    lambda m:state.__setitem__('drive',[m.linear.x,m.angular.z]),10)
stop=n.create_publisher(Bool,'/emergency_stop',10)


def spin_until(pred,seconds):
    end=time.monotonic()+seconds
    while time.monotonic()<end:
        rclpy.spin_once(n,timeout_sec=.05)
        if pred():return True
    return False


if not spin_until(lambda:all(k in state for k in
                           ('amcl','map','global','local','safety','camera','drive'))
                  and b.can_transform('map','base_footprint',rclpy.time.Time()),30):
    raise RuntimeError('preflight inputs missing')
if state['safety']['motion_requested'] or state['drive']!=[0.,0.]:
    raise RuntimeError('requires stopped motion-disabled launch')
report={'before':{'amcl':state['amcl'],'map':state['map'],
                  'global':state['global'],'local':state['local'],
                  'camera_state':state['camera']['perception_state'],
                  'camera_stamp':state['camera']['stamp'],
                  'safety':state['safety'],'drive':state['drive']}}
for _ in range(10):
    stop.publish(Bool(data=True))
    spin_until(lambda:False,.1)
if not spin_until(lambda:state['safety']['emergency_latched'],3):
    raise RuntimeError('emergency latch did not set')
report['after']={'emergency_latched':state['safety']['emergency_latched'],
                 'last_command':state['safety']['last_command'],
                 'drive':state['drive']}
Path('reports/phase5g_disabled_safety_probe.json').write_text(json.dumps(report,indent=2))
print('emergency_latched',report['after']['emergency_latched'])
n.destroy_node()
rclpy.shutdown()
