import sys
import unittest
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from exploration_core import detect_colored_cubes, frontier_goal, input_failures


class ExplorationCoreTests(unittest.TestCase):
    def test_cube_detector_reports_bearing_without_untrusted_range(self):
        image = np.zeros((240, 320, 3), np.uint8)
        cv2.rectangle(image, (210, 80), (280, 170), (200, 60, 10), -1)
        detections = detect_colored_cubes(image, fx=200, cx=160)
        self.assertEqual(len(detections), 1)
        self.assertEqual(detections[0]['color'], 'blue')
        self.assertGreater(detections[0]['bearing_rad'], 0)
        self.assertNotIn('distance_m', detections[0])

    def test_thin_colored_floor_region_is_not_a_cube(self):
        image = np.zeros((240, 320, 3), np.uint8)
        cv2.rectangle(image, (40, 200), (210, 207), (0, 0, 220), -1)
        self.assertEqual(detect_colored_cubes(image, 200, 160), [])

    def test_near_contact_cube_cut_by_image_edges_is_hazard(self):
        image = np.zeros((240, 320, 3), np.uint8)
        cv2.rectangle(image, (0, 100), (319, 239), (200, 60, 10), -1)
        detections = detect_colored_cubes(image, 200, 160)
        self.assertEqual(len(detections), 1)
        self.assertTrue(detections[0]['partial_view'])

    def test_frontier_goal_is_reachable_and_clear(self):
        grid = np.full((40, 40), -1, np.int16)
        grid[5:35, 5:35] = 0
        grid[15:25, 28] = 100
        goal = frontier_goal(grid.ravel(), 40, 40, 0.1,
                             (0.0, 0.0, 0.0), (2.0, 2.0))
        self.assertIsNotNone(goal)
        self.assertGreaterEqual(goal['clearance_m'], 0.38)
        self.assertTrue(0.0 < goal['unknown_fraction'] < 1.0)

    def test_frontier_rejects_robot_without_clearance(self):
        grid = np.full((20, 20), -1, np.int16)
        grid[2:18, 2:18] = 0
        self.assertIsNone(frontier_goal(grid.ravel(), 20, 20, 0.1,
                                        (0.0, 0.0, 0.0), (0.25, 0.25)))

    def test_stale_and_emergency_stop_fail_closed(self):
        seen = dict.fromkeys(('scan', 'camera', 'odom', 'map'), 10.0)
        self.assertEqual(input_failures(seen, 10.1, 0.1, False), [])
        self.assertIn('camera_stale', input_failures(seen, 10.6, 0.1, False))
        self.assertIn('tf_stale', input_failures(seen, 10.1, None, False))
        self.assertIn('emergency_stop', input_failures(seen, 10.1, 0.1, True))


if __name__ == '__main__':
    unittest.main()
