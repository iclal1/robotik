import math
from pathlib import Path
import sys
import unittest

import numpy as np

PROJECT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(PROJECT / 'src/competition_bringup/scripts'))
from autonomous_core import (ROBOT_SWEEP_RADIUS_M, astar,
                             control_command, frontier_groups, make_grid,
                             motion_segment_clear, path_clear,
                             rate_limit_command, select_frontier)


class AutonomousCoreTests(unittest.TestCase):
    def free_grid(self, size=80):
        values = np.zeros((size, size), np.int16)
        return make_grid(values.ravel(), size, size, .05, (-2., -2., 0.))

    def test_map_coordinates_with_rotated_origin(self):
        values = np.zeros((30, 30), np.int16)
        grid = make_grid(values.ravel(), 30, 30, .1, (1., 2., math.pi/2))
        xy = grid.cell_to_world(12, 13)
        self.assertEqual(grid.world_to_cell(*xy), (12, 13))

    def test_unknown_and_ambiguous_cells_are_not_free(self):
        values = np.zeros((60, 60), np.int16)
        values[30, 30] = -1
        values[30, 45] = 35
        grid = make_grid(values.ravel(), 60, 60, .05, (-1.5, -1.5, 0.))
        self.assertFalse(grid.safe(30, 30))
        self.assertFalse(grid.safe(45, 30))
        self.assertFalse(grid.safe(34, 30))

    def test_full_swept_footprint_rejects_narrow_gap(self):
        values = np.zeros((80, 80), np.int16)
        values[:, 40] = 100
        values[34:46, 40] = 0  # 0.6 m opening, less than swept diameter+margin.
        grid = make_grid(values.ravel(), 80, 80, .05, (-2., -2., 0.))
        self.assertGreater(2*(ROBOT_SWEEP_RADIUS_M+.10), .6)
        self.assertIsNone(astar(grid, (20, 40), (60, 40)))

    def test_wide_gap_is_plannable_without_corner_cutting(self):
        values = np.zeros((80, 80), np.int16)
        values[:, 40] = 100
        values[29:51, 40] = 0
        grid = make_grid(values.ravel(), 80, 80, .05, (-2., -2., 0.))
        cells = astar(grid, (20, 40), (60, 40))
        self.assertIsNotNone(cells)
        self.assertTrue(path_clear(grid, [grid.cell_to_world(*c) for c in cells]))

    def test_cube_layer_uses_uncertainty_and_age(self):
        values = np.zeros((80, 80), np.int16)
        new = make_grid(values.ravel(), 80, 80, .05, (-2., -2., 0.),
                        [(0., 0., .05, 0.)])
        old = make_grid(values.ravel(), 80, 80, .05, (-2., -2., 0.),
                        [(0., 0., .05, 10.)])
        self.assertFalse(new.safe(*new.world_to_cell(.4, 0.)))
        self.assertTrue(new.safe(*new.world_to_cell(.75, 0.)))
        self.assertFalse(old.safe(*old.world_to_cell(.75, 0.)))

    def test_path_rejects_hidden_crossing_between_waypoints(self):
        values = np.zeros((80, 80), np.int16)
        values[35:45, 40] = 100
        grid = make_grid(values.ravel(), 80, 80, .05, (-2., -2., 0.))
        self.assertFalse(path_clear(grid, [(-1., 0.), (1., 0.)]))

    def test_frontier_grouping_goal_and_blacklist(self):
        values = np.full((80, 80), -1, np.int16)
        values[10:70, 10:50] = 0
        grid = make_grid(values.ravel(), 80, 80, .05, (-2., -2., 0.))
        self.assertTrue(frontier_groups(grid))
        robot = grid.cell_to_world(25, 40)
        selected = select_frontier(grid, robot)
        self.assertIsNotNone(selected)
        self.assertTrue(path_clear(grid, selected['path']))
        next_goal = select_frontier(grid, robot, {selected['cell']})
        self.assertIsNotNone(next_goal)
        self.assertNotEqual(selected['cell'], next_goal['cell'])

    def test_unreachable_frontier_rejected(self):
        values = np.full((80, 80), -1, np.int16)
        values[10:70, 10:30] = 0
        values[10:70, 40:60] = 0
        grid = make_grid(values.ravel(), 80, 80, .05, (-2., -2., 0.))
        robot = grid.cell_to_world(20, 40)
        selected = select_frontier(grid, robot)
        self.assertIsNotNone(selected)
        self.assertLess(selected['cell'][0], 30)

    def test_controller_bounds_completion_and_swept_arc(self):
        grid = self.free_grid()
        linear, angular, done = control_command((0., 0., 0.), [(0., 1.)])
        self.assertFalse(done)
        self.assertEqual(linear, 0.)
        self.assertLessEqual(abs(angular), .35)
        self.assertEqual(control_command((0., .95, 0.), [(0., 1.)]), (0., 0., True))
        self.assertTrue(motion_segment_clear(grid, (0., 0., 0.), .12, .35))
        values = np.zeros((80, 80), np.int16)
        values[40, 42] = 100
        blocked = make_grid(values.ravel(), 80, 80, .05, (-2., -2., 0.))
        self.assertFalse(motion_segment_clear(blocked, (0., 0., 0.), .12, 0.))
        self.assertEqual(rate_limit_command((0., 0.), (.8, 1.5), .1),
                         (.1, .2))


if __name__ == '__main__':
    unittest.main()
