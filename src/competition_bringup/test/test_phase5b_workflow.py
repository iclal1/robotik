import math
import sys
import unittest
from pathlib import Path
from types import SimpleNamespace


ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0,str(ROOT/'src/competition_bringup/scripts'))
sys.path.insert(0,str(ROOT/'src/competition_bringup/tools'))
from audit_saved_map import audit
from nav2_safety_supervisor import inverse_transform_xy, segment_distance
from cube_tracks import CubeTracks


class Phase5BTests(unittest.TestCase):
    def test_old_map_does_not_cover_target_lane(self):
        result=audit(ROOT/'maps/autonomous_exploration/20261122T131614Z_ec1fb799/map.yaml')
        self.assertAlmostEqual(result['known_grid_fraction'],.2787918296,places=6)
        self.assertEqual(result['regions']['east_target_lane']['known_cells'],0)
        self.assertEqual(result['regions']['central_opening']['known_cells'],87)
        self.assertGreater(result['start_reachable_free_cells'],0)

    def test_segment_distance_selects_route_intersecting_track(self):
        self.assertAlmostEqual(segment_distance((.8,.2),(0,0),(0,.25)),.8)
        self.assertAlmostEqual(segment_distance((0,.1),(0,0),(0,.25)),0.)
        self.assertAlmostEqual(segment_distance((1,1),(0,0),(0,0)),math.sqrt(2))

    def test_map_goal_inverse_transform(self):
        q=SimpleNamespace(x=0.,y=0.,z=math.sin(math.pi/4),w=math.cos(math.pi/4))
        transform=SimpleNamespace(transform=SimpleNamespace(
            translation=SimpleNamespace(x=1.,y=2.),rotation=q))
        point=inverse_transform_xy(1.,3.,transform)
        self.assertAlmostEqual(point[0],1.)
        self.assertAlmostEqual(point[1],0.)

    def test_pre_push_cube_uncertainty_still_grows(self):
        stamp=SimpleNamespace(sec=1,nanosec=0)
        tracks=CubeTracks(growth_mps=.005)
        self.assertTrue(tracks.observe('blue',(.8,.2),.05,stamp,10.,
                                       track_id=0,confirmed=True))
        initial=list(tracks.obstacles(10.))[0][1]
        delayed=list(tracks.obstacles(30.))[0][1]
        self.assertAlmostEqual(delayed-initial,.10)
        with self.assertRaises(ValueError):
            CubeTracks(growth_mps=0.)


if __name__=='__main__':
    unittest.main()
