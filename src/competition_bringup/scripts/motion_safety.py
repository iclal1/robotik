"""Local, command-specific clearance for the original fixed competition arena.

The fixed exclusions below are conservative layout assumptions, never camera
observations. They must be removed for an arena with movable unknown objects.
"""
import math

from autonomous_core import ROBOT_SWEEP_RADIUS_M, CUBE_RADIUS_M

INITIAL_CUBE_ZONES = ((-1.6, 1.2), (-1.6, 0.0), (-1.6, -1.2))
START_WORLD_XY = (-2.4, -1.4)
ZONE_UNCERTAINTY_M = .18
SAFETY_MARGIN_M = .10
LINEAR_DECEL_MPS2 = 1.0
ANGULAR_DECEL_RADPS2 = 2.0
LATENCY_S = .35  # camera period, processing, controller, and guard response


def stopping_distance(speed, decel, latency=LATENCY_S):
    speed = abs(speed)
    return speed * latency + speed * speed / (2 * decel)


def scan_clearance(scan, xy, required_margin=.05):
    """Require a measured or max-range ray covering each footprint point."""
    radius = math.hypot(*xy)
    if radius < .08:
        return True, math.inf
    angle = math.atan2(xy[1], xy[0])
    if not scan.ranges or scan.angle_increment <= 0:
        return False, 0.
    index = round((angle - scan.angle_min) / scan.angle_increment)
    if not 0 <= index < len(scan.ranges):
        return False, 0.
    if abs(scan.angle_min + index * scan.angle_increment - angle) > .025:
        return False, 0.
    value = scan.ranges[index]
    if math.isnan(value) or value < scan.range_min:
        return False, 0.
    if math.isinf(value):
        value = scan.range_max if value > 0 else 0.
    return value >= radius + required_margin, value - radius


def swept_clearance(grid, scan, pose, linear, angular, horizon=.5):
    """Check map disk and LiDAR rays through the stopping tail of a command.

    The disk bounds chassis, wheels and pusher during any orientation. Camera
    cube tracks and initial-layout zones are already inflated in ``grid``.
    """
    x, y, yaw = pose
    travel = abs(linear) * horizon + stopping_distance(linear, LINEAR_DECEL_MPS2)
    turn = abs(angular) * horizon + stopping_distance(angular, ANGULAR_DECEL_RADPS2)
    samples = max(2, math.ceil(max(travel / .025, turn / .05)))
    min_clearance = math.inf
    for i in range(samples + 1):
        t = i / samples
        a = yaw + math.copysign(turn * t, angular) if angular else yaw
        distance = math.copysign(travel * t, linear) if linear else 0.
        px = x + distance * math.cos(yaw + (a-yaw)/2)
        py = y + distance * math.sin(yaw + (a-yaw)/2)
        if not grid.safe(*grid.world_to_cell(px, py)):
            return False, 0., 'map_or_cube_exclusion'
        # Sample perimeter plus interior at every swept pose. The disk radius
        # bounds the pusher corner (0.2375, 0.12) and both wheels.
        for radius in (.16, ROBOT_SWEEP_RADIUS_M + SAFETY_MARGIN_M):
            for j in range(24):
                theta = a + j * 2 * math.pi / 24
                wx = px + radius * math.cos(theta)
                wy = py + radius * math.sin(theta)
                dx, dy = wx-x, wy-y
                ok, clearance = scan_clearance(scan,
                    (math.cos(yaw)*dx + math.sin(yaw)*dy,
                     -math.sin(yaw)*dx + math.cos(yaw)*dy))
                min_clearance = min(min_clearance, clearance)
                if not ok:
                    return False, min_clearance, 'scan_swept_footprint'
    return True, min_clearance, None


def camera_candidate_blocks(status, reach_m=None, tracks=None, now=None):
    """Fail closed unless a fresh unranged candidate is beyond this sweep.

    A tentative lower bound can rule out a nearby collision. It never creates
    a cube track or makes any occupancy-grid cell free.
    """
    if not status or status.get('perception_state') == 'invalid_perception':
        return True
    for item in status.get('detections', ()):
        if item.get('valid', False):
            continue
        track = (tracks or {}).get(item.get('color'))
        same_single_cube_at_edge = (
            item.get('track_id') is None and
            item.get('reason') == 'contact_or_outline_clipped' and
            isinstance(item.get('bbox'), (list, tuple)) and
            len(item['bbox']) == 4 and
            (item['bbox'][0] <= 1 or
             item['bbox'][0]+item['bbox'][2] >= 639) and
            0 < item['bbox'][2] <= 24)
        if (track is not None and now is not None and
                track.get('confirmed') and not track.get('inconsistent') and
                ((item.get('track_id') is not None and
                  item.get('track_id') == track.get('track_id')) or
                 same_single_cube_at_edge) and
                0 <= now-track['seen'] <= 1.2):
            # The official arena has one cube of each color. A narrow edge
            # crop can briefly lose the detector ID, while its recently
            # confirmed cube remains in the map sweep with uncertainty.
            continue
        bounds = item.get('tentative_center_range_bounds_m')
        if (reach_m is None or item.get('reason') not in
                ('range_uncertainty_too_large','contact_or_outline_clipped') or
                not isinstance(bounds, (list, tuple)) or len(bounds) != 2 or
                not all(isinstance(v, (float, int)) and math.isfinite(v)
                        for v in bounds) or
                bounds[0] <= reach_m + CUBE_RADIUS_M + .05):
            return True
    return False


def first_unsafe_path_point(grid, points):
    for start, end in zip(points, points[1:]):
        count = max(1, math.ceil(math.dist(start, end)/(grid.resolution*.5)))
        for i in range(count+1):
            t = i/count
            xy = (start[0]*(1-t)+end[0]*t,
                  start[1]*(1-t)+end[1]*t)
            if not grid.safe(*grid.world_to_cell(*xy)):
                return xy
    return None
