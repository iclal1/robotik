"""Small, deterministic perception and map-planning helpers.

These functions propose observations and goals; they never command motion.
"""
import math

import cv2
import numpy as np


COLOR_RANGES = {
    'red': ((0, 90, 45), (12, 255, 255)),
    'green': ((35, 75, 40), (90, 255, 255)),
    'blue': ((95, 75, 35), (135, 255, 255)),
}


def detect_colored_cubes(bgr_image, fx, cx, min_area=250, min_side=18):
    """Return color and bearing candidates; monocular size is not a safe range."""
    if bgr_image.ndim != 3 or bgr_image.shape[2] != 3:
        raise ValueError('Expected a BGR image')
    hsv = cv2.cvtColor(bgr_image, cv2.COLOR_BGR2HSV)
    detections = []
    for color, (low, high) in COLOR_RANGES.items():
        mask = cv2.inRange(hsv, np.array(low), np.array(high))
        if color == 'red':
            mask |= cv2.inRange(hsv, np.array((170, 90, 45)),
                                np.array((179, 255, 255)))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,
                                np.ones((3, 3), np.uint8))
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                       cv2.CHAIN_APPROX_SIMPLE)
        for contour in contours:
            area = cv2.contourArea(contour)
            x, y, width, height = cv2.boundingRect(contour)
            partial_view = (x == 0 or y == 0 or
                            x + width >= bgr_image.shape[1] or
                            y + height >= bgr_image.shape[0])
            shaped_like_cube = (height >= min_side and width >= min_side and
                                0.45 <= width / height <= 2.0 and
                                area / (width * height) >= 0.55)
            if area < min_area or not (shaped_like_cube or
                                  (partial_view and area >= 1000)):
                continue
            bearing = math.atan2((x + width / 2) - cx, fx)
            detections.append({
                'color': color, 'bbox': [x, y, width, height],
                'bearing_rad': round(bearing, 4),
                'area_px': round(area),
                'partial_view': partial_view,
            })
    return sorted(detections, key=lambda item: (-item['area_px'], item['color']))


def frontier_goal(data, width, height, resolution, origin, robot,
                  clearance_m=0.38):
    """Find a reachable free-cell standoff near unknown space.

    Unknown cells are impassable. Clearance is measured from both unknown and
    occupied cells and includes the robot's pusher plus a margin. The result
    is diagnostic only until low obstacles are reliably perceived.
    """
    if width <= 0 or height <= 0 or resolution <= 0:
        return None
    grid = np.asarray(data, dtype=np.int16).reshape((height, width))
    free = (grid >= 0) & (grid <= 20)
    unknown = grid < 0
    if not np.any(free) or not np.any(unknown):
        return None
    distance = cv2.distanceTransform(free.astype(np.uint8),
                                     cv2.DIST_L2, 5) * resolution
    traversable = free & (distance >= clearance_m)
    ox, oy, yaw = origin
    dx, dy = robot[0] - ox, robot[1] - oy
    c, s = math.cos(yaw), math.sin(yaw)
    col = int(math.floor((c * dx + s * dy) / resolution))
    row = int(math.floor((-s * dx + c * dy) / resolution))
    if not (0 <= col < width and 0 <= row < height and traversable[row, col]):
        return None
    _, labels = cv2.connectedComponents(traversable.astype(np.uint8), 8)
    reachable = labels == labels[row, col]
    frontier = free & (cv2.dilate(unknown.astype(np.uint8),
                                 np.ones((3, 3), np.uint8)) > 0)
    if not np.any(frontier):
        return None
    to_frontier = cv2.distanceTransform((~frontier).astype(np.uint8),
                                        cv2.DIST_L2, 5) * resolution
    rows, cols = np.nonzero(reachable & (to_frontier <= 0.9))
    if len(rows) == 0:
        return None
    robot_distance = np.hypot((cols - col) * resolution,
                              (rows - row) * resolution)
    valid = robot_distance >= 0.30
    if not np.any(valid):
        return None
    rows, cols = rows[valid], cols[valid]
    robot_distance = robot_distance[valid]
    score = to_frontier[rows, cols] + 0.15 * robot_distance
    choice = int(np.argmin(score))
    local_x = (cols[choice] + 0.5) * resolution
    local_y = (rows[choice] + 0.5) * resolution
    return {
        'x': ox + c * local_x - s * local_y,
        'y': oy + s * local_x + c * local_y,
        'clearance_m': float(distance[rows[choice], cols[choice]]),
        'frontier_distance_m': float(to_frontier[rows[choice], cols[choice]]),
        'unknown_fraction': float(np.count_nonzero(unknown) / grid.size),
    }


def input_failures(last_seen, now, tf_age, emergency_stop):
    """List independent reasons a future motion controller must stop."""
    limits = {'scan': 0.35, 'camera': 0.5, 'odom': 0.2, 'map': 8.0}
    failures = [name + '_stale' for name, limit in limits.items()
                if name not in last_seen or now - last_seen[name] > limit]
    if tf_age is None or tf_age < 0 or tf_age > 0.5:
        failures.append('tf_stale')
    if emergency_stop:
        failures.append('emergency_stop')
    return failures
