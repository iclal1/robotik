import math
import sys
import unittest
from pathlib import Path
from types import SimpleNamespace

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from autonomous_core import make_grid
from motion_safety import (INITIAL_CUBE_ZONES, START_WORLD_XY, camera_candidate_blocks,
                           stopping_distance, swept_clearance)


class MotionSafetyTests(unittest.TestCase):
    def setUp(self):
        self.grid = make_grid(np.zeros(10000, np.int8), 100, 100, .05,
                              (-2.5, -2.5, 0.))
        self.scan = SimpleNamespace(ranges=[5.] * 720,
                                    angle_min=-math.pi,
                                    angle_increment=2*math.pi/719,
                                    range_min=.08, range_max=8.)

    def test_rotation_checks_full_sweep(self):
        self.assertTrue(swept_clearance(self.grid, self.scan,
                                        (0., 0., 0.), 0., .25)[0])
        self.scan.ranges[360] = .25
        self.assertFalse(swept_clearance(self.grid, self.scan,
                                         (0., 0., 0.), 0., .25)[0])

    def test_translation_checks_stopping_tail(self):
        self.assertGreater(stopping_distance(.1, 1.), .03)
        self.scan.ranges[360] = .38
        self.assertFalse(swept_clearance(self.grid, self.scan,
                                         (0., 0., 0.), .1, 0.)[0])

    def test_unknown_map_and_missing_ray_fail_closed(self):
        values = np.zeros((100, 100), np.int8)
        values[50, 51] = -1
        grid = make_grid(values.ravel(), 100, 100, .05, (-2.5, -2.5, 0.))
        self.assertFalse(swept_clearance(grid, self.scan,
                                         (0., 0., 0.), .1, 0.)[0])
        self.scan.ranges[360] = math.nan
        self.assertFalse(swept_clearance(self.grid, self.scan,
                                         (0., 0., 0.), .1, 0.)[0])

    def test_fixed_layout_zones_are_separate_obstacles(self):
        grid = make_grid(np.zeros(14400, np.int8), 120, 120, .05,
                         (-3., -3., 0.),
                         [(x, y, .18, 0.) for x, y in INITIAL_CUBE_ZONES])
        self.assertFalse(grid.safe(*grid.world_to_cell(-1.6, -1.2)))
        self.assertTrue(grid.safe(*grid.world_to_cell(-2.4, -1.4)))

    def test_layout_coordinates_match_initial_odom_frame(self):
        blue = INITIAL_CUBE_ZONES[2]
        self.assertAlmostEqual(blue[0] - START_WORLD_XY[0], .8)
        self.assertAlmostEqual(blue[1] - START_WORLD_XY[1], .2)

    def test_unranged_visible_cube_blocks(self):
        self.assertTrue(camera_candidate_blocks({'perception_state': 'visible_obstacle_distance_uncertain',
                                                 'detections': [{'valid': False}]}))
        self.assertFalse(camera_candidate_blocks({'perception_state': 'unknown_or_unobserved',
                                                  'detections': []}))

    def test_distant_tentative_lower_bound_only_clears_near_field(self):
        status = {'perception_state': 'visible_obstacle_distance_uncertain',
                  'detections': [{'valid': False,
                                  'reason': 'range_uncertainty_too_large',
                                  'tentative_center_range_bounds_m': [2.2, 3.2]}]}
        self.assertFalse(camera_candidate_blocks(status, .5))
        self.assertTrue(camera_candidate_blocks(status, 2.1))
        self.assertTrue(camera_candidate_blocks(status))
        status['detections'][0]['tentative_center_range_bounds_m'] = [math.nan, 3.2]
        self.assertTrue(camera_candidate_blocks(status, .5))

    def test_recent_matching_track_covers_brief_clipping_only(self):
        status = {'perception_state': 'confirmed_visible_obstacle',
                  'detections': [{'color': 'blue', 'track_id': 0,
                                  'valid': False,
                                  'reason': 'contact_or_outline_clipped'}]}
        tracks = {'blue': {'track_id': 0, 'confirmed': True,
                           'inconsistent': False, 'seen': 10.}}
        self.assertFalse(camera_candidate_blocks(status, .5, tracks, 10.4))
        self.assertTrue(camera_candidate_blocks(status, .5, tracks, 11.3))
        status['detections'][0]['track_id'] = 7
        self.assertTrue(camera_candidate_blocks(status, .5, tracks, 10.4))

    def test_recent_single_cube_narrow_edge_crop_keeps_track_exclusion(self):
        status = {'perception_state':'visible_obstacle_distance_uncertain',
                  'detections':[{'color':'green','track_id':None,'valid':False,
                                 'reason':'contact_or_outline_clipped',
                                 'bbox':[0,168,22,66]}]}
        tracks = {'green':{'track_id':1,'confirmed':True,
                           'inconsistent':False,'seen':10.}}
        self.assertFalse(camera_candidate_blocks(status,.4,tracks,10.2))
        self.assertTrue(camera_candidate_blocks(status,.4,tracks,11.3))
        status['detections'][0]['bbox']=[12,168,22,66]
        self.assertTrue(camera_candidate_blocks(status,.4,tracks,10.2))

    def test_thin_edge_crop_lower_bound_is_only_nearfield_evidence(self):
        status={'perception_state':'visible_obstacle_distance_uncertain',
                'detections':[{'valid':False,'reason':'contact_or_outline_clipped',
                               'tentative_center_range_bounds_m':[.57,1.04]}]}
        self.assertFalse(camera_candidate_blocks(status,.35))
        self.assertTrue(camera_candidate_blocks(status,.5))


if __name__ == '__main__':
    unittest.main()
