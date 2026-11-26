"""Saved-map Nav2 contract and independently measured trial regression checks."""
import hashlib
import json
import math
from collections import Counter
from pathlib import Path
import unittest

from PIL import Image
import yaml


ROOT = Path(__file__).resolve().parents[3]
BRINGUP = ROOT / 'src/competition_bringup'
CONFIG = yaml.safe_load((BRINGUP/'config/nav2_saved_map.yaml').read_text())
MAP = ROOT/'maps/autonomous_exploration/20261122T131614Z_ec1fb799'


class Nav2IntegrationTests(unittest.TestCase):
    def test_saved_map_is_unmodified_and_has_unknown_space(self):
        metadata = yaml.safe_load((MAP/'map.yaml').read_text())
        self.assertEqual(metadata['resolution'], .05)
        self.assertEqual(metadata['origin'], [-.559,-.554,0.])
        self.assertEqual(hashlib.sha256((MAP/'map.pgm').read_bytes()).hexdigest(),
                         'b2dcc49870af31407c48cc915fa206b9dcb5a1229575b5c28a4a325873dabb59')
        self.assertIn('allow_unknown: false',
                      (BRINGUP/'config/nav2_saved_map.yaml').read_text())

    def test_complete_footprint_shared_by_costmaps(self):
        local = CONFIG['local_costmap']['local_costmap']['ros__parameters']
        global_ = CONFIG['global_costmap']['global_costmap']['ros__parameters']
        footprint = json.loads(local['footprint'])
        self.assertEqual(footprint,json.loads(global_['footprint']))
        self.assertGreaterEqual(max(x for x,y in footprint),.2375)
        self.assertLessEqual(min(x for x,y in footprint),-.20)
        self.assertGreaterEqual(max(abs(y) for x,y in footprint),.1675)
        self.assertIn('cubes',local['obstacle_layer']['observation_sources'])
        self.assertIn('cubes',global_['obstacle_layer']['observation_sources'])
        self.assertEqual(local['obstacle_layer']['cubes']['topic'],
                         '/nav2/cube_obstacles')

    def test_phase5b_nav2_metadata_preserves_saved_unknown_cells(self):
        saved = ROOT/'maps/competition_mapping/20261125T103259Z_572d1928'
        source = yaml.safe_load((saved/'map.yaml').read_text())
        nav = yaml.safe_load((BRINGUP/'config/phase5b_nav2_map.yaml').read_text())
        self.assertEqual(Path(nav['image']), saved/'map.pgm')
        for key in ('mode','resolution','origin','negate','occupied_thresh'):
            self.assertEqual(nav[key],source[key])
        counts = Counter(Image.open(nav['image']).getdata())
        self.assertEqual((counts[254],counts[205],counts[0]),(8519,275,607))
        unknown_probability = (255-205)/255
        self.assertGreater(source['free_thresh'],unknown_probability)
        self.assertLess(nav['free_thresh'],unknown_probability)
        self.assertIn('PHASE5B_MAP_SHA256',
                      (BRINGUP/'launch/competition_nav2.launch.py').read_text())

    def test_single_guarded_motion_path_and_disabled_default(self):
        launch=(BRINGUP/'launch/competition_nav2.launch.py').read_text()
        supervisor=(BRINGUP/'scripts/nav2_safety_supervisor.py').read_text()
        self.assertIn("DeclareLaunchArgument('enable_motion',default_value='false')",launch)
        self.assertEqual(launch.count("('cmd_vel','/nav2_cmd_vel')"),2)
        self.assertIn("self.cmd_pub = self.create_publisher(Twist, '/cmd_vel'",supervisor)
        self.assertIn('safety_halt_latched',supervisor)
        self.assertIn("'/emergency_stop'",supervisor)

    def test_amcl_updates_before_short_goal_can_finish(self):
        amcl=CONFIG['amcl']['ros__parameters']
        goal=CONFIG['controller_server']['ros__parameters']['goal_checker']
        map_resolution=CONFIG['local_costmap']['local_costmap']['ros__parameters']['resolution']
        self.assertGreater(amcl['update_min_d'],0)
        self.assertLessEqual(amcl['update_min_d'],map_resolution/2)
        self.assertLess(amcl['update_min_d'],goal['xy_goal_tolerance']/2)
        self.assertGreater(amcl['update_min_a'],0)
        self.assertLessEqual(amcl['update_min_a'],goal['yaw_goal_tolerance']/2)
        self.assertEqual(amcl['resample_interval'],1)
        self.assertFalse(amcl['set_initial_pose'])

    def test_recorded_nav2_action_reached_goal_without_cube_motion(self):
        report=json.loads((ROOT/'reports/phase5_nav2_goal_trial2.json').read_text())
        self.assertTrue(report['navigate_to_pose_succeeded'])
        self.assertGreater(report['robot_displacement_m'],.1)
        self.assertLessEqual(report['position_error_m'],.08)
        self.assertLessEqual(report['heading_error_rad'],.15)
        self.assertGreater(report['minimum_nominal_geometry_clearance_m'],0)
        self.assertFalse(report['safety_halt_latched'])
        self.assertEqual(report['emergency_stop_events'],0)
        self.assertTrue(all(math.isclose(value,0.,abs_tol=1e-6)
                            for value in report['cube_displacement_m'].values()))


if __name__=='__main__':
    unittest.main()
