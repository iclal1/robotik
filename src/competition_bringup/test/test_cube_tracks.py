import sys
import unittest
from pathlib import Path
from types import SimpleNamespace

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from cube_tracks import CubeTracks


def stamp(seconds):
    whole = int(seconds)
    return SimpleNamespace(sec=whole, nanosec=int((seconds-whole)*1e9))


class CubeTracksTests(unittest.TestCase):
    def test_rotation_changes_base_bearing_but_not_odom_track(self):
        tracks = CubeTracks()
        self.assertTrue(tracks.observe('blue', (.8, .2), .05, stamp(1.), 10.))
        tracks.note_visibility(set())
        self.assertEqual(tracks.interruptions, 1)
        self.assertEqual(list(tracks.tracks), ['blue'])
        self.assertTrue(tracks.observe('blue', (.81, .19), .05, stamp(1.3), 10.3))
        self.assertEqual(tracks.reacquisitions, 1)
        self.assertEqual(len(tracks.tracks), 1)

    def test_uncertainty_grows_and_old_track_is_not_fresh(self):
        tracks = CubeTracks()
        tracks.observe('green', (.8, 1.4), .1, stamp(1.), 10.)
        recent = list(tracks.obstacles(10.1))[0][1]
        old = list(tracks.obstacles(13.))[0][1]
        self.assertGreater(old, recent + .05)
        self.assertFalse(tracks.fresh('green', 13.))
        self.assertEqual(len(tracks.tracks), 1)  # remains an exclusion

    def test_jump_and_out_of_order_measurements_rejected(self):
        tracks = CubeTracks()
        tracks.observe('red', (.5, .5), .05, stamp(1.), 10.)
        self.assertFalse(tracks.observe('red', (.5, .5), .05, stamp(.9), 10.1))
        self.assertFalse(tracks.observe('red', (2., 2.), .05, stamp(1.1), 10.1))
        self.assertTrue(tracks.tracks['red']['inconsistent'])
        self.assertFalse(tracks.observe('red', (.5, .5), .05, stamp(1.2), 10.2))

    def test_same_timestamp_is_deduplicated_and_confirmation_promoted(self):
        tracks = CubeTracks()
        tracks.observe('blue', (.8, .2), .05, stamp(1.), 10., 0, False)
        tracks.observe('blue', (.8, .2), .05, stamp(1.), 10.1, 0, True)
        self.assertEqual(len(tracks.tracks), 1)
        self.assertTrue(tracks.tracks['blue']['confirmed'])
        self.assertEqual(tracks.tracks['blue']['seen'], 10.)

    def test_side_crop_keeps_precise_prior_center_and_expands_uncertainty(self):
        tracks = CubeTracks()
        tracks.observe('blue', (.8, .2), .05, stamp(1.), 10., 0, True)
        tracks.observe('blue', (.7, .3), .14, stamp(1.1), 10.1, 0, True, True)
        self.assertGreater(tracks.tracks['blue']['odom_xy'][0], .78)
        self.assertLess(tracks.tracks['blue']['uncertainty'], .14)
        self.assertGreater(tracks.tracks['blue']['uncertainty'], .11)

    def test_repeated_valid_side_crops_do_not_accumulate_unbounded_uncertainty(self):
        tracks = CubeTracks(growth_mps=.005)
        tracks.observe('green', (.8, 1.4), .1815, stamp(1.), 10., 1, True)
        for index in range(1, 1001):
            self.assertTrue(tracks.observe(
                'green', (.8, 1.4), .1815, stamp(1. + index*.13),
                10. + index*.13, 1, True, True))
        self.assertLess(tracks.tracks['green']['uncertainty'], .43)
        self.assertGreater(tracks.tracks['green']['uncertainty'], .24)
        self.assertEqual(tracks.statuses(140.)['green']['state'], 'uncertain')
        self.assertFalse(tracks.fresh('green', 140.))

    def test_states_and_reacquisition_preserve_obstacle_until_measured(self):
        tracks = CubeTracks(growth_mps=.005)
        tracks.observe('blue', (.8, .2), .08, stamp(1.), 10., 2, True)
        self.assertEqual(tracks.statuses(10.)['blue']['state'], 'visible')
        tracks.note_visibility(set())
        self.assertEqual(tracks.statuses(10.2)['blue']['state'], 'recently_observed')
        self.assertEqual(tracks.statuses(12.)['blue']['state'], 'stale')
        old = tracks.statuses(50.)['blue']
        self.assertEqual(old['state'], 'lost')
        self.assertGreater(old['uncertainty_m'], .3)
        self.assertEqual(len(list(tracks.obstacles(50.))), 1)
        self.assertTrue(tracks.observe('blue', (.81, .2), .08,
                                       stamp(41.), 50., 2, True))
        self.assertEqual(tracks.statuses(50.)['blue']['state'], 'visible')
        self.assertLess(tracks.statuses(50.)['blue']['uncertainty_m'], old['uncertainty_m'])
        self.assertEqual(tracks.reacquisitions, 1)
        self.assertEqual(len(tracks.tracks), 1)

    def test_partial_measurement_expands_when_displaced_but_cannot_certify_reobservation(self):
        tracks = CubeTracks()
        tracks.observe('red', (.5, .5), .05, stamp(1.), 10., 3, True)
        tracks.observe('red', (.64, .5), .18, stamp(1.1), 10.1, 3, True, True)
        self.assertGreaterEqual(tracks.tracks['red']['uncertainty'], .11)
        self.assertFalse(tracks.fresh('red', 10.1))
        tracks.observe('red', (.63, .5), .08, stamp(1.2), 10.2, 3, True)
        self.assertTrue(tracks.fresh('red', 10.2))

    def test_three_colors_keep_identity_and_timestamp_order(self):
        tracks = CubeTracks()
        for index, (color, center) in enumerate((('red', (.8, 2.6)),
                                                  ('green', (.8, 1.4)),
                                                  ('blue', (.8, .2)))):
            self.assertTrue(tracks.observe(color, center, .08,
                                           stamp(1.), 10., index, True))
        self.assertFalse(tracks.observe('green', (.8, 1.4), .08,
                                        stamp(.9), 10.1, 1, True))
        self.assertEqual(set(tracks.statuses(10.1)), {'red', 'green', 'blue'})
        self.assertEqual(tracks.tracks['red']['odom_xy'], (.8, 2.6))
        self.assertEqual(tracks.tracks['green']['odom_xy'], (.8, 1.4))
        self.assertEqual(tracks.tracks['blue']['odom_xy'], (.8, .2))


if __name__ == '__main__':
    unittest.main()
