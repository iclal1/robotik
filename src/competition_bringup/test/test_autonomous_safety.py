import sys
import unittest
from pathlib import Path
from types import SimpleNamespace

import rclpy
from std_msgs.msg import Bool
import numpy as np

PROJECT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(PROJECT / 'src/competition_bringup/scripts'))
from autonomous_exploration import AutonomousExploration
from autonomous_core import make_grid


class AutonomousSafetyTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = AutonomousExploration()

    def tearDown(self):
        self.node.destroy_node()

    def test_default_is_planning_only_and_sensors_fail_closed(self):
        self.assertFalse(self.node.motion_requested)
        failures = self.node.failures()
        for key in ('scan_stale', 'camera_stale', 'odom_stale', 'map_stale',
                    'cube_perception_invalid'):
            self.assertIn(key, failures)
        self.assertIsNone(self.node.map_transform())
        self.node.control_tick()
        self.assertEqual(self.node.last_command, (0., 0.))

    def test_emergency_stop_is_latched(self):
        self.node.on_stop(Bool(data=True))
        self.node.on_stop(Bool(data=False))
        self.assertIn('emergency_stop_latched', self.node.failures())
        self.assertEqual(self.node.last_command, (0., 0.))

    def test_simulation_time_reset_is_latched(self):
        self.node.mark('scan', SimpleNamespace(sec=30, nanosec=0))
        self.node.mark('scan', SimpleNamespace(sec=2, nanosec=0))
        self.assertIn('simulation_time_reset_latched', self.node.failures())

    def test_global_clearance_does_not_gate_local_motion(self):
        self.node.cube_clearance = {'safe_to_move': False}
        self.node.cube_status = {'perception_state': 'confirmed_visible_obstacle',
                                 'detections': []}
        self.assertNotIn('cube_clearance_not_validated', self.node.failures())

    def test_sensor_failure_after_command_latches_stop(self):
        self.node.motion_requested = True
        self.node.motion_ever_commanded = True
        self.node.control_tick()
        self.assertTrue(self.node.safety_halt_latched)
        self.assertEqual(self.node.last_command, (0., 0.))

    def test_bounded_goal_is_not_replaced_by_frontier_planner(self):
        self.node.validation_mode = 'goal'
        self.node.goal = {'cell': (40, 40), 'goal': (0., .25),
                          'goal_yaw': 1.57}
        self.node.path = [(0., 0.), (0., .25)]
        self.node.map_msg = object()
        self.node.map_transform = lambda: object()
        self.node.robot_pose = lambda _: (0., 0., 0.)
        self.node.grid = lambda _: make_grid(np.zeros(6400), 80, 80,
                                             .05, (-2., -2., 0.))
        self.node.plan_tick()
        self.assertEqual(self.node.goal['goal'], (0., .25))
        self.assertEqual(self.node.replans, 0)

    def test_completed_trial_stays_stopped_as_tracks_age(self):
        self.node.validation_mode = 'goal'
        self.node.validation_done = True
        self.node.motion_requested = True
        self.node.motion_ever_commanded = True
        self.node.goal = {'cell': (40, 40), 'goal': (0., .25),
                          'goal_yaw': 1.57}
        self.node.path = [(0., 0.), (0., .25)]
        self.node.map_msg = object()
        self.node.map_transform = lambda: object()
        self.node.robot_pose = lambda _: (0., 0., 0.)
        self.node.grid = lambda _: make_grid(np.full(6400, -1), 80, 80,
                                             .05, (-2., -2., 0.))
        self.node.plan_tick()
        self.node.control_tick()
        self.assertFalse(self.node.safety_halt_latched)
        self.assertEqual(self.node.last_command, (0., 0.))


if __name__ == '__main__':
    unittest.main()
