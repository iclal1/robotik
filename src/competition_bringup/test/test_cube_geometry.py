import math
import sys
import unittest
from pathlib import Path
from types import SimpleNamespace

import cv2
import numpy as np

PROJECT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(PROJECT / 'src/competition_bringup/scripts'))
from cube_geometry import (calibration_from_info, estimate_cube,
                           CUBE_RADIUS_M, ROBOT_RADIUS_M, SAFETY_MARGIN_M,
                           perception_state, plane_intersection,
                           rotation_from_quaternion)
from exploration_core import detect_colored_cubes
from cube_vision import detect_cube_candidates
from cube_visualization import (marker_ids, smooth_display_range,
                                sudden_area_loss,
                                visual_state)


class CubeGeometryTests(unittest.TestCase):
    def setUp(self):
        # Runtime values captured from /camera/camera_info and robot TF.
        self.info = SimpleNamespace(
            k=[381.361, 0, 320, 0, 381.361, 240, 0, 0, 1],
            d=[0] * 5, width=640, height=480,
            distortion_model='plumb_bob')
        self.calibration = calibration_from_info(self.info)
        self.rotation = rotation_from_quaternion(
            SimpleNamespace(x=.547, y=-.547, z=.448, w=-.448))
        self.translation = np.array([.205, 0, .195])

    def estimate_from_recorded_image(self, color):
        image = cv2.imread(str(PROJECT / 'docs' /
                               f'camera_{color}_centered.png'))
        self.assertIsNotNone(image)
        small = cv2.resize(image, (320, 240))
        candidates = detect_colored_cubes(
            small, 381.361 / 2, 160, min_area=60, min_side=8)
        matches = [item for item in candidates if item['color'] == color]
        self.assertEqual(len(matches), 1)
        item = matches[0]
        item['bbox'] = [v * 2 for v in item['bbox']]
        item['area_px'] *= 4
        return estimate_cube(item, self.calibration, self.rotation,
                             self.translation)

    def test_recorded_red_green_blue_cubes(self):
        for color in ('red', 'green', 'blue'):
            with self.subTest(color=color):
                result = self.estimate_from_recorded_image(color)
                self.assertTrue(result['valid'], result)
                self.assertGreater(result['geometry_confidence'], .55)
                self.assertLess(result['position_uncertainty_m'], .20)
                self.assertGreater(result['clearance_lower_bound_m'], 0)
                self.assertAlmostEqual(
                    result['clearance_lower_bound_m'],
                    max(0.0, result['surface_range_bounds_m'][0] -
                        CUBE_RADIUS_M - ROBOT_RADIUS_M - SAFETY_MARGIN_M))

    def test_red_hue_wraparound(self):
        hsv = np.zeros((240, 320, 3), np.uint8)
        hsv[80:145, 130:190] = (175, 220, 220)
        image = cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)
        detections = detect_colored_cubes(image, 190, 160,
                                          min_area=60, min_side=8)
        self.assertEqual([item['color'] for item in detections], ['red'])

    def test_floor_target_does_not_pass_cube_height_geometry(self):
        # A square color region can pass simple contour shape checks.
        target = {'bbox': [250, 300, 100, 100],
                  'area_px': 10000, 'partial_view': False}
        result = estimate_cube(target, self.calibration,
                               self.rotation, self.translation)
        self.assertFalse(result['valid'])
        self.assertEqual(result['reason'], 'flat_target_or_bad_geometry')

    def test_clipped_cube_has_no_distance(self):
        target = {'bbox': [0, 190, 640, 290],
                  'area_px': 100000, 'partial_view': True}
        result = estimate_cube(target, self.calibration,
                               self.rotation, self.translation)
        self.assertFalse(result['valid'])
        self.assertNotIn('center_range_m', result)

    def test_distant_cube_retains_measured_uncertainty(self):
        candidate = {'bbox': [282, 166, 70, 56],
                     'area_px': 3560, 'partial_view': False}
        result = estimate_cube(candidate, self.calibration,
                               self.rotation, self.translation)
        self.assertTrue(result['valid'])
        self.assertLess(result['position_uncertainty_m'], .30)
        self.assertGreater(result['position_uncertainty_m'], .05)
        self.assertLess(result['surface_range_bounds_m'][0], 1.6)
        self.assertGreater(result['surface_range_bounds_m'][1], 1.6)

    def test_native_resolution_silhouettes(self):
        for color in ('red', 'green', 'blue'):
            with self.subTest(color=color):
                image = cv2.imread(str(PROJECT / 'docs' /
                                       f'camera_{color}_centered.png'))
                candidates = detect_cube_candidates(image, 381.361, 320)
                self.assertEqual([c['color'] for c in candidates], [color])
                candidate = candidates[0]
                self.assertGreater(candidate['contact_pixel'][1],
                                   candidate['bbox'][1] + .8*candidate['bbox'][3])
                result = estimate_cube(candidate, self.calibration,
                                       self.rotation, self.translation)
                self.assertTrue(result['valid'], result)
                self.assertGreater(result['geometry_confidence'], .85)

    def test_actual_gazebo_target_and_distant_red(self):
        target_image = cv2.imread(str(PROJECT /
            'reports/perception_validation/20261122_targets.png'))
        self.assertIsNotNone(target_image)
        self.assertEqual(detect_cube_candidates(target_image, 381.361, 320), [])
        distant_image = cv2.imread(str(PROJECT /
            'reports/perception_validation/20261122_yaw1.png'))
        self.assertIsNotNone(distant_image)
        candidates = detect_cube_candidates(distant_image, 381.361, 320)
        self.assertEqual({c['color'] for c in candidates}, {'red', 'green'})
        pitch = .2
        exact_rotation = np.array([
            [0, -math.sin(pitch), math.cos(pitch)],
            [-1, 0, 0], [0, -math.cos(pitch), -math.sin(pitch)]])
        red = next(c for c in candidates if c['color'] == 'red')
        estimate = estimate_cube(red, self.calibration,
                                 exact_rotation, self.translation)
        self.assertEqual(estimate['reason'], 'range_uncertainty_too_large')
        self.assertGreater(estimate['appearance_confidence'], .9)

    def test_recorded_multi_angle_gazebo_views(self):
        expected = {
            'blue_right_edge': {'blue'},
            'blue_frontal': {'blue'},
            'blue_green_oblique': {'blue', 'green'},
            'multiple_oblique': {'red', 'green', 'blue'},
            'red_green_oblique': {'red', 'green'},
        }
        base = PROJECT / 'reports/perception_validation/phase3d_baseline'
        for name, colors in expected.items():
            with self.subTest(view=name):
                image = cv2.imread(str(base / f'{name}.png'))
                self.assertIsNotNone(image)
                found = detect_cube_candidates(image, 381.361, 320)
                self.assertEqual({c['color'] for c in found}, colors)
                for candidate in found:
                    if candidate['partial_view']:
                        estimate = estimate_cube(candidate, self.calibration,
                                                 self.rotation, self.translation)
                        if estimate['valid']:
                            self.assertTrue(candidate['bbox'][0] <= 1 or
                                sum((candidate['bbox'][0], candidate['bbox'][2])) >= 639)
                            self.assertGreater(estimate['position_uncertainty_m'], .10)
                        else:
                            self.assertGreater(estimate['appearance_confidence'], .4)

    def test_horizontal_crop_with_full_vertical_edges_has_conservative_range(self):
        candidate = {'bbox': [0, 167, 60, 70], 'area_px': 3562.5,
                     'partial_view': True, 'contact_pixel': [30, 236],
                     'fill_ratio': .848, 'solidity': .95}
        estimate = estimate_cube(candidate, self.calibration,
                                 self.rotation, self.translation)
        self.assertTrue(estimate['valid'], estimate)
        self.assertGreater(estimate['surface_range_bounds_m'][0], 1.1)
        self.assertGreater(estimate['position_uncertainty_m'], .15)
        candidate['bbox'] = [0, 0, 60, 70]
        self.assertFalse(estimate_cube(candidate, self.calibration,
                                       self.rotation, self.translation)['valid'])

    def test_narrow_side_crop_retains_conservative_range(self):
        candidate = {'bbox':[0,168,22,66], 'area_px':1200,
                     'partial_view':True, 'contact_pixel':[11,233],
                     'fill_ratio':.83, 'solidity':.95}
        estimate = estimate_cube(candidate,self.calibration,
                                 self.rotation,self.translation)
        self.assertTrue(estimate['valid'],estimate)
        self.assertGreater(estimate['surface_range_bounds_m'][0],1.2)
        self.assertGreater(estimate['position_uncertainty_m'],.25)
        candidate['bbox']=[0,168,18,66]
        self.assertFalse(estimate_cube(candidate,self.calibration,
                                       self.rotation,self.translation)['valid'])

    def test_thin_edge_crop_has_nearfield_bound_but_no_position(self):
        candidate={'bbox':[630,173,10,161],'area_px':1200,
                   'partial_view':True,'fill_ratio':.80,'solidity':.95}
        estimate=estimate_cube(candidate,self.calibration,
                               self.rotation,self.translation)
        self.assertFalse(estimate['valid'])
        self.assertEqual(estimate['reason'],'contact_or_outline_clipped')
        self.assertGreater(estimate['tentative_center_range_bounds_m'][0],.55)
        self.assertNotIn('center_position_base_m',estimate)

    def test_invalid_calibration_is_rejected(self):
        self.info.k[0] = 0
        self.assertIsNone(calibration_from_info(self.info))
        self.info.k[0] = math.nan
        self.assertIsNone(calibration_from_info(self.info))

    def test_ground_ray_and_frame_orientation(self):
        forward = self.rotation @ np.array([0., 0., 1.])
        self.assertGreater(forward[0], .9)
        center_ground = plane_intersection(320, 360, 0., self.calibration,
                                           self.rotation, self.translation)
        self.assertGreater(center_ground[0], .2)
        self.assertAlmostEqual(center_ground[1], 0., delta=.02)

    def test_missing_and_stale_frames_cannot_grant_clearance(self):
        detected = [{'valid': True, 'confirmed': True}]
        self.assertEqual(perception_state(None, 0, None, detected)[0],
                         'invalid_perception')
        self.assertEqual(perception_state(.6, 0, None, detected)[1],
                         'camera_frames_stale')
        self.assertEqual(perception_state(0, 1.1, None, detected)[1],
                         'camera_info_stale')
        self.assertEqual(perception_state(0, 0, 'tf_unavailable', detected)[0],
                         'invalid_perception')
        self.assertEqual(perception_state(0, 0, None, [
            {'valid': False, 'confirmed': False}])[0],
            'visible_obstacle_distance_uncertain')

    def test_marker_identity_and_expiration(self):
        track = {'id': 7, 'position': np.array([1., 0.]),
                 'marker_visible': True, 'seen': 10.0}
        self.assertEqual(marker_ids(7), (14, 15))
        self.assertEqual(visual_state(track, 10.1, {7}, set()), 'confirmed')
        self.assertEqual(visual_state(track, 10.2, set(), {7}), 'uncertain')
        self.assertEqual(visual_state(track, 10.3, set(), set()), 'stale')
        self.assertEqual(visual_state(track, 11.21, set(), set()), 'expired')

    def test_display_range_filters_small_jitter_not_motion(self):
        self.assertAlmostEqual(smooth_display_range(.80, .82, 1., 1.07), .808)
        self.assertAlmostEqual(smooth_display_range(.80, 1.10, 1., 1.07), 1.025)
        self.assertAlmostEqual(smooth_display_range(.80, 1.10, 1., 1.60), 1.10)

    def test_sudden_partial_silhouette_does_not_imply_new_range(self):
        self.assertTrue(sudden_area_loss(18000, 7000, .07))
        self.assertFalse(sudden_area_loss(18000, 14000, .07))
        self.assertFalse(sudden_area_loss(18000, 7000, .5))
        original = cv2.imread(str(PROJECT /
            'reports/perception_validation/phase3d_baseline/blue_frontal.png'))
        occluded = cv2.imread(str(PROJECT /
            'reports/perception_validation/phase3d_occluded_fixture.png'))
        self.assertIsNotNone(original)
        self.assertIsNotNone(occluded)
        full = detect_cube_candidates(original, 381.361, 320)[0]
        partial = detect_cube_candidates(occluded, 381.361, 320)[0]
        self.assertTrue(sudden_area_loss(
            full['area_px'], partial['area_px'], 1/15))


if __name__ == '__main__':
    unittest.main()
