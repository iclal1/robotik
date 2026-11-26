"""Deterministic costmap, frontier planning, and conservative path control.

This module has no ROS publishers and never uses Gazebo truth. Occupancy is
free only for 0..20; unknown and ambiguous cells are non-traversable.
"""
from dataclasses import dataclass
import heapq
import math

import cv2
import numpy as np


ROBOT_SWEEP_RADIUS_M = math.hypot(.2375, .12)
SAFETY_MARGIN_M = .10
CUBE_RADIUS_M = math.sqrt(2) * .18 / 2
NEIGHBORS = ((1, 0, 1.), (-1, 0, 1.), (0, 1, 1.), (0, -1, 1.),
             (1, 1, math.sqrt(2)), (1, -1, math.sqrt(2)),
             (-1, 1, math.sqrt(2)), (-1, -1, math.sqrt(2)))


@dataclass
class Grid:
    values: np.ndarray
    resolution: float
    origin_x: float
    origin_y: float
    origin_yaw: float
    traversable: np.ndarray
    clearance: np.ndarray

    @property
    def shape(self):
        return self.values.shape

    def world_to_cell(self, x, y):
        dx, dy = x - self.origin_x, y - self.origin_y
        c, s = math.cos(self.origin_yaw), math.sin(self.origin_yaw)
        return (int(math.floor((c*dx + s*dy) / self.resolution)),
                int(math.floor((-s*dx + c*dy) / self.resolution)))

    def cell_to_world(self, col, row):
        lx, ly = (col + .5)*self.resolution, (row + .5)*self.resolution
        c, s = math.cos(self.origin_yaw), math.sin(self.origin_yaw)
        return (self.origin_x + c*lx - s*ly,
                self.origin_y + s*lx + c*ly)

    def contains(self, col, row):
        return 0 <= row < self.shape[0] and 0 <= col < self.shape[1]

    def safe(self, col, row):
        return self.contains(col, row) and bool(self.traversable[row, col])


def make_grid(data, width, height, resolution, origin, cube_obstacles=(),
              margin=SAFETY_MARGIN_M):
    """Inflate walls, unknown cells and separately tracked cube exclusions."""
    if width <= 0 or height <= 0 or resolution <= 0:
        raise ValueError('Invalid occupancy-grid dimensions')
    values = np.asarray(data, np.int16).reshape(height, width)
    free = (values >= 0) & (values <= 20)
    padded = cv2.copyMakeBorder(free.astype(np.uint8), 1, 1, 1, 1,
                                cv2.BORDER_CONSTANT, value=0)
    clearance = cv2.distanceTransform(padded, cv2.DIST_L2, 5)[1:-1, 1:-1]
    clearance *= resolution
    traversable = free & (clearance >= ROBOT_SWEEP_RADIUS_M + margin)
    grid = Grid(values, resolution, *origin, traversable, clearance)
    for cube in cube_obstacles:
        x, y, uncertainty, age = cube
        radius = (ROBOT_SWEEP_RADIUS_M + margin + CUBE_RADIUS_M +
                  max(0., uncertainty) + .03*max(0., age))
        col, row = grid.world_to_cell(x, y)
        reach = int(math.ceil(radius / resolution)) + 1
        for rr in range(max(0, row-reach), min(height, row+reach+1)):
            for cc in range(max(0, col-reach), min(width, col+reach+1)):
                wx, wy = grid.cell_to_world(cc, rr)
                if math.hypot(wx-x, wy-y) <= radius:
                    traversable[rr, cc] = False
    return grid


def path_clear(grid, points):
    """Sample every half-cell; disk inflation covers the complete footprint."""
    if not points:
        return False
    for start, end in zip(points, points[1:]):
        length = math.dist(start, end)
        samples = max(1, int(math.ceil(length / (grid.resolution*.5))))
        for i in range(samples + 1):
            t = i/samples
            col, row = grid.world_to_cell(start[0]*(1-t)+end[0]*t,
                                          start[1]*(1-t)+end[1]*t)
            if not grid.safe(col, row):
                return False
    col, row = grid.world_to_cell(*points[0])
    return grid.safe(col, row)


def astar(grid, start, goal):
    """Eight-connected A* without diagonal corner cutting."""
    if not grid.safe(*start) or not grid.safe(*goal):
        return None
    frontier = [(0., start)]
    cost = {start: 0.}
    parent = {start: None}
    while frontier:
        _, current = heapq.heappop(frontier)
        if current == goal:
            cells = []
            while current is not None:
                cells.append(current)
                current = parent[current]
            cells.reverse()
            return cells
        for dc, dr, step in NEIGHBORS:
            nxt = (current[0]+dc, current[1]+dr)
            if not grid.safe(*nxt):
                continue
            if dc and dr and (not grid.safe(current[0]+dc, current[1]) or
                              not grid.safe(current[0], current[1]+dr)):
                continue
            clearance = grid.clearance[nxt[1], nxt[0]]
            extra = .04 / max(clearance, .05)
            new = cost[current] + step*(1+extra)
            if new >= cost.get(nxt, math.inf):
                continue
            cost[nxt] = new
            parent[nxt] = current
            estimate = math.hypot(goal[0]-nxt[0], goal[1]-nxt[1])
            heapq.heappush(frontier, (new+estimate, nxt))
    return None


def reachable_paths(grid, start):
    """One shortest-path tree for all frontier candidates."""
    if not grid.safe(*start):
        return {}, {}
    queue = [(0., start)]
    costs = {start: 0.}
    parents = {start: None}
    while queue:
        current_cost, current = heapq.heappop(queue)
        if current_cost > costs[current]:
            continue
        for dc, dr, step in NEIGHBORS:
            nxt = (current[0]+dc, current[1]+dr)
            if not grid.safe(*nxt):
                continue
            if dc and dr and (not grid.safe(current[0]+dc, current[1]) or
                              not grid.safe(current[0], current[1]+dr)):
                continue
            extra = .04 / max(grid.clearance[nxt[1], nxt[0]], .05)
            new_cost = current_cost + step*(1+extra)
            if new_cost >= costs.get(nxt, math.inf):
                continue
            costs[nxt] = new_cost
            parents[nxt] = current
            heapq.heappush(queue, (new_cost, nxt))
    return costs, parents


def frontier_groups(grid, min_cells=3):
    """Group free cells touching unknown space into connected frontiers."""
    free = (grid.values >= 0) & (grid.values <= 20)
    unknown = grid.values < 0
    touches = cv2.dilate(unknown.astype(np.uint8),
                         cv2.getStructuringElement(cv2.MORPH_CROSS, (3, 3))) > 0
    frontier = free & touches
    count, labels = cv2.connectedComponents(frontier.astype(np.uint8), 8)
    groups = []
    for label in range(1, count):
        rows, cols = np.nonzero(labels == label)
        if len(rows) >= min_cells:
            groups.append(list(zip(cols.tolist(), rows.tolist())))
    return groups


def select_frontier(grid, robot_xy, blacklist=(), min_goal_distance=.30):
    """Choose a reachable known-free standoff with useful unknown boundary."""
    start = grid.world_to_cell(*robot_xy)
    if not grid.safe(*start):
        return None
    costs, parents = reachable_paths(grid, start)
    blocked = set(blacklist)
    best = None
    for group in frontier_groups(grid):
        mask = np.zeros(grid.shape, np.uint8)
        for col, row in group:
            mask[row, col] = 1
        reach = max(1, int(math.ceil(.9/grid.resolution)))
        near = cv2.dilate(mask, np.ones((2*reach+1, 2*reach+1), np.uint8)) > 0
        rows, cols = np.nonzero(near & grid.traversable)
        options = sorted((cell for cell in zip(cols.tolist(), rows.tolist())
                          if math.dist(robot_xy, grid.cell_to_world(*cell)) >=
                          min_goal_distance),
                         key=lambda cell: math.hypot(cell[0]-start[0],
                                                     cell[1]-start[1]))[:25]
        for cell in options:
            if cell in blocked or cell not in costs:
                continue
            xy = grid.cell_to_world(*cell)
            cells = []
            current = cell
            while current is not None:
                cells.append(current)
                current = parents[current]
            cells.reverse()
            path = [grid.cell_to_world(*point) for point in cells]
            if not path_clear(grid, path):
                continue
            length = sum(math.dist(a, b) for a, b in zip(path, path[1:]))
            gain = len(group) * grid.resolution
            score = length - .20*min(gain, 2.) - .1*grid.clearance[cell[1], cell[0]]
            if best is None or score < best['score']:
                best = {'cell': cell, 'goal': xy, 'path': path,
                        'score': score, 'frontier_cells': len(group),
                        'path_length_m': length}
    return best


def control_command(pose, path, max_linear=.12, max_angular=.35):
    """Return a bounded (linear, angular, complete) path-following command."""
    if not path:
        return 0., 0., True
    x, y, yaw = pose
    if math.dist((x, y), path[-1]) <= .12:
        return 0., 0., True
    target = path[-1]
    for point in path:
        if math.dist((x, y), point) > .28:
            target = point
            break
    heading = math.atan2(target[1]-y, target[0]-x)
    error = math.atan2(math.sin(heading-yaw), math.cos(heading-yaw))
    angular = max(-max_angular, min(max_angular, 1.5*error))
    if abs(error) > .40:
        return 0., angular, False
    distance = math.dist((x, y), target)
    linear = min(max_linear, .6*distance) * max(0., math.cos(error))
    return linear, angular, False


def motion_segment_clear(grid, pose, linear, angular, horizon=.5):
    """Check a simulated short command arc in the inflated costmap."""
    x, y, yaw = pose
    samples = max(5, int(math.ceil(horizon/.05)))
    for _ in range(samples):
        dt = horizon/samples
        yaw += angular*dt
        x += linear*math.cos(yaw)*dt
        y += linear*math.sin(yaw)*dt
        if not grid.safe(*grid.world_to_cell(x, y)):
            return False
    return True


def rate_limit_command(previous, requested, dt, linear_accel=1., angular_accel=2.):
    """Respect the drive's acceleration caps before publishing /cmd_vel."""
    dt = max(0., min(dt, .2))
    def clip(value, old, step):
        return max(old-step, min(old+step, value))
    return (clip(requested[0], previous[0], linear_accel*dt),
            clip(requested[1], previous[1], angular_accel*dt))
