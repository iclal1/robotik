#!/usr/bin/env python3
"""Read-only coverage audit of a sensor-derived occupancy grid."""
import argparse
import json
from collections import deque
from pathlib import Path

import numpy as np
from PIL import Image
import yaml


# These rectangles only report where the saved map has evidence. They never
# generate occupancy values or enter production localization or planning.
WORLD_REGIONS = {
    'west_start_lane': (-2.95, -.60, -1.95, 1.95),
    'central_opening': (-.60, 1.10, -.60, .60),
    'east_target_lane': (1.10, 2.95, -1.95, 1.95),
    'red_target_vicinity': (1.95, 2.75, .95, 1.45),
    'green_target_vicinity': (1.95, 2.75, -.25, .25),
    'blue_target_vicinity': (1.95, 2.75, -1.45, -.95),
}
START_WORLD_XY = (-2.4, -1.4)


def audit(map_yaml):
    metadata = yaml.safe_load(Path(map_yaml).read_text())
    image = np.array(Image.open(Path(map_yaml).parent / metadata['image']))
    if image.ndim != 2:
        raise ValueError('Expected grayscale occupancy PGM')
    height, width = image.shape
    resolution = float(metadata['resolution'])
    origin_x, origin_y, yaw = map(float, metadata['origin'])
    if abs(yaw) > 1e-6:
        raise ValueError('Rotated maps require a coordinate transform')
    # Nav2 map saver uses 205 for unknown, 254 for free, and 0 for occupied.
    known = image != 205
    free = image >= 250
    occupied = image <= 100
    regions = {}
    for name, (xlo, xhi, ylo, yhi) in WORLD_REGIONS.items():
        cells = []
        for row in range(height):
            y = origin_y + (height - row - .5)*resolution + START_WORLD_XY[1]
            if not ylo <= y < yhi:
                continue
            for col in range(width):
                x = origin_x + (col + .5)*resolution + START_WORLD_XY[0]
                if xlo <= x < xhi:
                    cells.append((row, col))
        total = len(cells)
        count = sum(bool(known[row, col]) for row, col in cells)
        regions[name] = {'cells': total, 'known_cells': count,
                         'known_fraction': count/total if total else None}
    # Connected free space is useful for detecting disconnected map islands.
    start_col = int((0-origin_x)/resolution)
    start_row = height-1-int((0-origin_y)/resolution)
    reachable = np.zeros_like(free, dtype=bool)
    if (0 <= start_row < height and 0 <= start_col < width and
            free[start_row, start_col]):
        queue = deque([(start_row, start_col)])
        reachable[start_row, start_col] = True
        while queue:
            row, col = queue.popleft()
            for nr, nc in ((row-1,col),(row+1,col),(row,col-1),(row,col+1)):
                if (0 <= nr < height and 0 <= nc < width and
                        free[nr,nc] and not reachable[nr,nc]):
                    reachable[nr,nc] = True
                    queue.append((nr,nc))
    return {
        'map_yaml': str(Path(map_yaml).resolve()),
        'resolution_m': resolution, 'origin': metadata['origin'],
        'width_cells': width, 'height_cells': height,
        'known_cells': int(known.sum()), 'unknown_cells': int((~known).sum()),
        'free_cells': int(free.sum()), 'occupied_cells': int(occupied.sum()),
        'known_grid_fraction': float(known.mean()),
        'start_reachable_free_cells': int(reachable.sum()),
        'start_reachable_fraction_of_free':
            float(reachable.sum()/free.sum()) if free.any() else 0.,
        'regions': regions,
        'coverage_note': 'Known fraction of grid and named regions; this is not proof of safe robot-footprint reachability.',
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('map_yaml', type=Path)
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    result = json.dumps(audit(args.map_yaml), indent=2) + '\n'
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(result)
    print(result, end='')


if __name__ == '__main__':
    main()
