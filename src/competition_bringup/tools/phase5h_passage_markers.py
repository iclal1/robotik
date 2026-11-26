"""RViz MarkerArray preview of Phase 5H planning-only passage evidence."""
import json
import math
import time
from pathlib import Path

import rclpy
from geometry_msgs.msg import Point
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from visualization_msgs.msg import Marker, MarkerArray

from footprint_clearance import ROBOT_FOOTPRINT, rotated_polygon

data=json.loads(Path('reports/phase5h_passage_probe.json').read_text())
map_margin=data['margin']['map_m']
rclpy.init()
n=Node('phase5h_passage_markers')
n.set_parameters([Parameter('use_sim_time',Parameter.Type.BOOL,True)])
q=QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,
             durability=DurabilityPolicy.TRANSIENT_LOCAL)
publisher=n.create_publisher(MarkerArray,'/phase5h/passage_markers',q)


def marker(kind,name,index,color,scale=.02):
    m=Marker();m.header.frame_id='map';m.header.stamp=n.get_clock().now().to_msg()
    m.ns=name;m.id=index;m.type=kind;m.action=Marker.ADD
    m.pose.orientation.w=1.;m.scale.x=scale;m.scale.y=scale;m.scale.z=scale
    m.color.r,m.color.g,m.color.b,m.color.a=color
    return m


def points2d(coords,z=.04):
    return [Point(x=float(x),y=float(y),z=z) for x,y in coords]


def offset_polygon(poly,margin):
    # The physical polygon is clockwise. Adjacent outward-offset lines meet
    # at a conservative miter; this is a display envelope, not validation.
    lines=[]
    for i,a in enumerate(poly):
        b=poly[(i+1)%len(poly)]
        dx,dy=b[0]-a[0],b[1]-a[1]
        length=math.hypot(dx,dy)
        nx,ny=-dy/length,dx/length
        lines.append(((a[0]+margin*nx,a[1]+margin*ny),(dx,dy)))
    output=[]
    for i in range(len(poly)):
        p,v=lines[i-1];q,w=lines[i]
        cross=v[0]*w[1]-v[1]*w[0]
        if abs(cross)<1e-9:continue
        t=((q[0]-p[0])*w[1]-(q[1]-p[1])*w[0])/cross
        output.append((p[0]+t*v[0],p[1]+t*v[1]))
    return output


markers=[];index=0
for name,route in data.get('candidate_routes',{}).items():
    line=marker(Marker.LINE_STRIP,'oriented_candidate_'+name,index,
                (.75,.2,1.,.95),.038)
    index+=1;line.points=points2d([p[:2] for p in route['poses']],.12)
    markers.append(line)
    robust=route.get('robust_check',{})
    if not robust.get('safe') and 'pose' in robust:
        fail=marker(Marker.SPHERE,'yaw_uncertainty_failures',index,(1.,0.,0.,1.),.14)
        index+=1;fail.pose.position.x,fail.pose.position.y=robust['pose'][:2]
        fail.pose.position.z=.14;markers.append(fail)
for name,trial in data['scenarios'].items():
    path=trial['path_poses'];check=trial.get('polygon_check',{})
    if not path:continue
    valid=check.get('safe',False)
    color=(.1,.9,.3,.9) if valid else (1.,.2,.15,.9)
    line=marker(Marker.LINE_STRIP,'computed_paths',index,color,.025);index+=1
    line.points=points2d([p[:2] for p in path],.06);markers.append(line)
    for kind,pose in [('start',trial['start']),('goal',trial['goal'])]:
        item=marker(Marker.SPHERE,'approach_points',index,
                    (.2,.6,1.,.9) if kind=='start' else (1.,.8,.15,.9),.08)
        index+=1;item.pose.position.x,item.pose.position.y=pose[:2]
        item.pose.position.z=.07;markers.append(item)
    if not valid and 'pose' in check:
        fail=marker(Marker.SPHERE,'clearance_failures',index,(1.,0.,0.,1.),.12)
        index+=1;fail.pose.position.x,fail.pose.position.y=check['pose'][:2]
        fail.pose.position.z=.1;markers.append(fail)
    if name in ('a_west_to_central','b_central_to_east'):
        for p in path[::max(1,len(path)//5)]:
            poly=rotated_polygon(ROBOT_FOOTPRINT,p)
            outline=marker(Marker.LINE_STRIP,'sampled_physical_footprints',index,
                           (.1,.85,1.,.8),.008)
            index+=1;outline.points=points2d([*poly,poly[0]],.085)
            markers.append(outline)

for name,x,y in [('wall_A',2.05,1.407),('wall_B',3.3,1.007)]:
    physical=rotated_polygon(ROBOT_FOOTPRINT,(x,y,0.))
    for label,poly,color,width in [
            ('physical',physical,(.1,.85,1.,1.),.018),
            ('margin_envelope',offset_polygon(physical,map_margin),
             (1.,.65,.1,.9),.012)]:
        line=marker(Marker.LINE_STRIP,name+'_'+label,index,color,width)
        index+=1;line.points=points2d([*poly,poly[0]],.10);markers.append(line)
    text=marker(Marker.TEXT_VIEW_FACING,name+'_label',index,(1.,1.,1.,1.),.15)
    index+=1;text.text=name.replace('_',' ')+' | robot cyan | margin orange'
    text.pose.position.x=x;text.pose.position.y=y;text.pose.position.z=.35
    text.scale.z=.12;markers.append(text)

message=MarkerArray(markers=markers)
try:
    while rclpy.ok():
        for item in message.markers:item.header.stamp=n.get_clock().now().to_msg()
        publisher.publish(message)
        rclpy.spin_once(n,timeout_sec=.5)
except KeyboardInterrupt:
    pass
finally:
    n.destroy_node()
    if rclpy.ok():rclpy.shutdown()
