"""Conservative oriented-polygon clearance against occupancy cells and cubes."""
import math


ROBOT_FOOTPRINT = ((-.20,-.1675),(-.20,.1675),(.20,.15),
                   (.2375,.12),(.2375,-.12),(.20,-.15))


def rotated_polygon(vertices, pose):
    x,y,yaw=pose
    c,s=math.cos(yaw),math.sin(yaw)
    return tuple((x+c*a-s*b,y+s*a+c*b) for a,b in vertices)


def point_segment_distance(p,a,b):
    dx,dy=b[0]-a[0],b[1]-a[1]
    t=0. if dx*dx+dy*dy==0 else max(0.,min(1.,
        ((p[0]-a[0])*dx+(p[1]-a[1])*dy)/(dx*dx+dy*dy)))
    return math.dist(p,(a[0]+t*dx,a[1]+t*dy))


def _cross(a,b,c):
    return (b[0]-a[0])*(c[1]-a[1])-(b[1]-a[1])*(c[0]-a[0])


def segments_intersect(a,b,c,d):
    return (max(min(a[0],b[0]),min(c[0],d[0]))<=
            min(max(a[0],b[0]),max(c[0],d[0])) and
            max(min(a[1],b[1]),min(c[1],d[1]))<=
            min(max(a[1],b[1]),max(c[1],d[1])) and
            _cross(a,b,c)*_cross(a,b,d)<=0 and
            _cross(c,d,a)*_cross(c,d,b)<=0)


def point_in_convex_polygon(p,vertices):
    cross=[_cross(vertices[i],vertices[(i+1)%len(vertices)],p)
           for i in range(len(vertices))]
    return min(cross)>=-1e-12 or max(cross)<=1e-12


def polygon_distance(a,b):
    """Exact planar separation for two convex polygons (zero if touching)."""
    if any(point_in_convex_polygon(p,b) for p in a):return 0.
    if any(point_in_convex_polygon(p,a) for p in b):return 0.
    result=math.inf
    for i in range(len(a)):
        u,v=a[i],a[(i+1)%len(a)]
        for j in range(len(b)):
            w,z=b[j],b[(j+1)%len(b)]
            if segments_intersect(u,v,w,z):return 0.
            result=min(result,point_segment_distance(u,w,z),
                       point_segment_distance(v,w,z),
                       point_segment_distance(w,u,v),
                       point_segment_distance(z,u,v))
    return result


def square(x,y,side):
    return ((x,y),(x+side,y),(x+side,y+side),(x,y+side))


def grid_clearance(grid,polygon,blocked):
    """Distance to occupied/unknown *cell areas*, not just their centers."""
    res=grid['resolution'];ox,oy=grid['origin'];width=grid['width'];height=grid['height']
    xmin=min(p[0] for p in polygon);xmax=max(p[0] for p in polygon)
    ymin=min(p[1] for p in polygon);ymax=max(p[1] for p in polygon)
    # All current required margins are below 0.32 m; 0.50 m also measures
    # useful surplus while keeping long path checks bounded.
    reach=.5
    lo_x=math.floor((xmin-reach-ox)/res);hi_x=math.floor((xmax+reach-ox)/res)
    lo_y=math.floor((ymin-reach-oy)/res);hi_y=math.floor((ymax+reach-oy)/res)
    boundary=min(xmin-ox,ox+width*res-xmax,ymin-oy,oy+height*res-ymax)
    best=max(0.,boundary);source=('outside',None)
    lo_x=max(0,lo_x);hi_x=min(width-1,hi_x)
    lo_y=max(0,lo_y);hi_y=min(height-1,hi_y)
    data=grid['data']
    for row in range(lo_y,hi_y+1):
        for col in range(lo_x,hi_x+1):
            value=data[row*width+col]
            if not blocked(value):continue
            d=polygon_distance(polygon,square(ox+col*res,oy+row*res,res))
            if d<best:
                best=d;source=((col,row),value)
                if d==0:return best,source
    return best,source


def interpolate_pose(a,b,position_step=.025,angle_step=.05):
    dyaw=math.atan2(math.sin(b[2]-a[2]),math.cos(b[2]-a[2]))
    count=max(1,math.ceil(math.dist(a[:2],b[:2])/position_step),
              math.ceil(abs(dyaw)/angle_step))
    for i in range(count+1):
        t=i/count
        yield (a[0]*(1-t)+b[0]*t,a[1]*(1-t)+b[1]*t,a[2]+dyaw*t)


def validate_sweep(poses,grids,cubes=(),map_margin=.3126,
                   live_margin=.2626,cube_margin=.2626,
                   yaw_uncertainty=0.):
    """Check translation and rotation with the full physical polygon."""
    minimum={'map':math.inf,'global':math.inf,'cube':math.inf}
    if not poses:return {'safe':False,'reason':'empty_path'}
    segments=list(zip(poses,poses[1:])) or [(poses[0],poses[0])]
    if not math.isfinite(yaw_uncertainty) or yaw_uncertainty<0:
        raise ValueError('yaw uncertainty must be finite and nonnegative')
    yaw_offsets=(0.,) if yaw_uncertainty==0 else (
        -yaw_uncertainty,-yaw_uncertainty/2,0.,
        yaw_uncertainty/2,yaw_uncertainty)
    for index,(a,b) in enumerate(segments):
        for pose in interpolate_pose(a,b):
            for offset in yaw_offsets:
                examined=(pose[0],pose[1],pose[2]+offset)
                robot=rotated_polygon(ROBOT_FOOTPRINT,examined)
                for key,margin,blocked in (
                        ('map',map_margin,lambda v:v!=0),
                        ('global',live_margin,lambda v:v<0 or v>=100)):
                    if key not in grids:continue
                    distance,source=grid_clearance(grids[key],robot,blocked)
                    minimum[key]=min(minimum[key],distance)
                    if distance<margin:
                        return {'safe':False,'reason':key+'_clearance',
                                'segment':index,'pose':examined,
                                'nominal_pose':pose,'yaw_offset_rad':offset,
                                'distance_m':distance,'required_m':margin,
                                'source':source,'minimum':minimum}
                for name,cube in cubes:
                    distance=polygon_distance(robot,cube)
                    minimum['cube']=min(minimum['cube'],distance)
                    if distance<cube_margin:
                        return {'safe':False,'reason':'cube_clearance',
                                'cube':name,'segment':index,'pose':examined,
                                'nominal_pose':pose,'yaw_offset_rad':offset,
                                'distance_m':distance,'required_m':cube_margin,
                                'minimum':minimum}
    return {'safe':True,'minimum':minimum}
