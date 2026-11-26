"""Regression for Phase 5E's blocked ROS monitor and ambiguous stale warning."""
from pathlib import Path
import sys
import threading
import time
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'tools'))
from nav_trial_monitor import AsyncGazeboSampler, sensor_ages


class NavTrialMonitorTests(unittest.TestCase):
    def test_slow_gazebo_query_does_not_block_sensor_callbacks(self):
        release = threading.Event()
        started = threading.Event()

        def slow_query():
            started.set()
            release.wait(timeout=1.0)
            return {'pusher_robot': (0, 0)}

        sampler = AsyncGazeboSampler(query=slow_query, interval_s=0,
                                     deadline_s=1.1)
        try:
            begin = time.monotonic()
            self.assertIsNone(sampler.tick())
            self.assertTrue(started.wait(timeout=0.2))
            # The old synchronous query would have prevented this receipt.
            latest = {'odom': {'wall': time.monotonic(), 't': 10.0},
                      'scan': {'wall': time.monotonic(), 't': 10.0}}
            self.assertIsNone(sampler.tick())
            self.assertLess(time.monotonic()-begin, 0.2)
            ages, failures = sensor_ages(latest, time.monotonic(), 10.05,
                                         {'odom': .3, 'scan': .4})
            self.assertEqual(failures, [])
            self.assertLess(ages['odom']['receive_wall_s'], .3)
            release.set()
            deadline = time.monotonic()+.5
            sample = None
            while sample is None and time.monotonic()<deadline:
                sample = sampler.tick()
                if sample is None:
                    time.sleep(.01)
            self.assertEqual(sample['poses']['pusher_robot'], (0, 0))
        finally:
            release.set()
            sampler.close()

    def test_wall_delivery_and_simulation_stamp_failures_are_distinct(self):
        ages, failures = sensor_ages(
            {'odom': {'wall': 1.0, 't': 20.0},
             'scan': {'wall': 1.4, 't': 19.0},
             'camera': {'wall': 1.4, 't': 20.0}},
            now_wall=1.41, clock_sim=20.1,
            limits={'odom': .3, 'scan': .4, 'camera': .6})
        self.assertIn('odom_receipt_stale', failures)
        self.assertNotIn('odom_stamp_stale', failures)
        self.assertIn('scan_stamp_stale', failures)
        self.assertNotIn('scan_receipt_stale', failures)
        self.assertAlmostEqual(ages['camera']['stamp_sim_s'], .1)


if __name__ == '__main__':
    unittest.main()
