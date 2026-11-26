#!/usr/bin/env python3
"""Repeatable, small archived-Gazebo-image color-detection benchmark.

This image set has one visible cube per image. It is a regression check,
not a representative precision/recall estimate for the whole arena.
"""
import json
import importlib.machinery
import math
from pathlib import Path
import sys
import time

import cv2

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / 'src/competition_bringup/scripts'))
from cube_vision import detect_cube_candidates
from cube_geometry import calibration_from_info, estimate_cube
from exploration_core import detect_colored_cubes
from types import SimpleNamespace


SAMPLES = [(ROOT / 'docs' / f'camera_{color}_centered.png', color)
           for color in ('red', 'green', 'blue')]
SAMPLES += [(ROOT / 'docs' / f'camera_blue_{view}.png', 'blue')
            for view in ('initial', 'precontact', 'contact')]
SAMPLES += [(ROOT / 'reports/perception_validation/20261122_live_baseline.png',
             'blue')]
SAMPLES += [(ROOT / 'reports/perception_validation/20261122_yaw1.png',
             ('green', 'red')),
            (ROOT / 'reports/perception_validation/20261122_targets.png',
             ())]


def run():
    original_path = ROOT / '.perception_backups/20261122/__pycache__/cube_geometry.cpython-310.pyc'
    original_geometry = importlib.machinery.SourcelessFileLoader(
        'baseline_cube_geometry', str(original_path)).load_module()
    calibration = calibration_from_info(SimpleNamespace(
        k=[381.361, 0, 320, 0, 381.361, 240, 0, 0, 1], d=[0]*5,
        width=640, height=480, distortion_model='plumb_bob'))
    import numpy as np
    rotation = np.array([[0, -math.sin(.2), math.cos(.2)], [-1, 0, 0],
                         [0, -math.cos(.2), -math.sin(.2)]])
    translation = np.array([.205, 0, .195])
    rows = []
    for path, truth in SAMPLES:
        image = cv2.imread(str(path))
        if image is None:
            raise FileNotFoundError(path)
        small = cv2.resize(image, (320, 240), interpolation=cv2.INTER_AREA)
        old = detect_colored_cubes(small, 381.361 / 2, 160, 60, 8)
        new = detect_cube_candidates(image, 381.361, 320)
        for candidate in old:
            candidate['bbox'] = [value*2 for value in candidate['bbox']]
            candidate['area_px'] *= 4
        old_published = [item['color'] for item in old if
            original_geometry.estimate_cube(item, calibration, rotation,
                translation).get('reason') != 'flat_target_or_bad_geometry']
        new_published = [item['color'] for item in new if
            estimate_cube(item, calibration, rotation,
                translation).get('reason') != 'flat_target_or_bad_geometry']
        truth = (truth,) if isinstance(truth, str) else truth
        rows.append({'file': str(path.relative_to(ROOT)), 'truth': truth,
                     'baseline': old_published,
                     'improved': new_published,
                     'clipped': any(d['partial_view'] for d in new)})
    metrics = {}
    for version in ('baseline', 'improved'):
        by_color = {}
        for color in ('red', 'green', 'blue'):
            tp = fp = fn = 0
            for row in rows:
                detected = color in row[version]
                present = color in row['truth']
                tp += int(detected and present)
                fp += int(detected and not present)
                fn += int(not detected and present)
            by_color[color] = {'tp': tp, 'fp': fp, 'fn': fn,
                               'precision': tp/(tp+fp) if tp+fp else None,
                               'recall': tp/(tp+fn) if tp+fn else None}
        tp = sum(v['tp'] for v in by_color.values())
        fp = sum(v['fp'] for v in by_color.values())
        fn = sum(v['fn'] for v in by_color.values())
        metrics[version] = {'by_color': by_color, 'precision': tp/(tp+fp),
                            'recall': tp/(tp+fn), 'tp': tp, 'fp': fp, 'fn': fn}
    return {'samples': rows, 'metrics': metrics,
            'limitation': ('Nine archived Gazebo frames at a few fixed poses; '
                           'published 2D candidates after geometry rejection; '
                           'not a representative arena precision/recall estimate.')}


if __name__ == '__main__':
    print(json.dumps(run(), indent=2))
