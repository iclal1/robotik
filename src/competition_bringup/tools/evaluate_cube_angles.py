#!/usr/bin/env python3
"""Compare test-only fixed-pose captures to independent Gazebo poses."""
import json
from pathlib import Path
import statistics

from validate_cube_perception import compare


ROOT = Path(__file__).resolve().parents[3]
DATA = ROOT / 'reports/perception_validation'
TRUTH = {
    'blue_right_edge': {'blue'},
    'blue_frontal': {'blue'},
    'blue_green_oblique': {'blue', 'green'},
    'multiple_oblique': {'blue', 'green', 'red'},
    'red_green_oblique': {'red', 'green'},
}


def evaluate(version):
    rows = json.loads((DATA / f'phase3d_{version}/manifest.json').read_text())
    tp = fp = fn = 0
    details = []
    for row in rows:
        actual = TRUTH[row['label']]
        observed = {item['color'] for item in row['status']['detections']}
        tp += len(actual & observed)
        fp += len(observed - actual)
        fn += len(actual - observed)
        details.append({'label': row['label'], 'truth': sorted(actual),
                        'observed': sorted(observed),
                        'gazebo_comparison': compare(
                            row['status'], row['gazebo_poses']),
                        'latency_ms': row['status']['processing_latency_ms']})
    errors = [d['position_error_m'] for row in details
              for d in row['gazebo_comparison'] if 'position_error_m' in d]
    return {'cases': len(rows), 'true_appearances': sum(len(v) for v in TRUTH.values()),
            'tp': tp, 'fp': fp, 'fn': fn,
            'precision': tp/(tp+fp) if tp+fp else None,
            'recall': tp/(tp+fn) if tp+fn else None,
            'valid_3d_count': len(errors),
            'valid_3d_mean_position_error_m': statistics.mean(errors) if errors else None,
            'mean_status_latency_ms': statistics.mean(
                row['latency_ms'] for row in details),
            'details': details}


if __name__ == '__main__':
    print(json.dumps({'baseline': evaluate('baseline'),
                      'improved': evaluate('improved')}, indent=2))
