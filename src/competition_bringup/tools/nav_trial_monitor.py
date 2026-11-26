"""Nonblocking Gazebo sampling and explicit freshness checks for trial tools.

This module never publishes velocity and is not part of the production safety
path. Wall time measures callback delivery; simulation time measures message age.
"""
from concurrent.futures import ThreadPoolExecutor
import subprocess
import time

from validate_cube_perception import parse_pose_info


def read_gazebo_poses(timeout_s=2.0):
    output = subprocess.run(
        ['ign', 'topic', '-e', '-t', '/world/competition/pose/info', '-n', '1'],
        capture_output=True, text=True, check=True, timeout=timeout_s).stdout
    return parse_pose_info(output)


class AsyncGazeboSampler:
    """A bounded one-worker poller; callers keep spinning ROS while it runs."""

    def __init__(self, query=read_gazebo_poses, interval_s=0.4,
                 deadline_s=2.5, clock=time.monotonic):
        self.query = query
        self.interval_s = interval_s
        self.deadline_s = deadline_s
        self.clock = clock
        self.pool = ThreadPoolExecutor(max_workers=1)
        self.future = None
        self.started = None
        self.last_started = float('-inf')
        self.last_sample = None

    def tick(self):
        """Return a completed sample, None while pending, or raise on failure."""
        now = self.clock()
        if self.future is not None:
            if self.future.done():
                result = self.future.result()
                sample = {'started_wall': self.started, 'finished_wall': now,
                          'duration_wall_s': now-self.started, 'poses': result}
                self.last_sample = sample
                self.future = None
                return sample
            if now-self.started > self.deadline_s:
                raise TimeoutError('Gazebo pose query exceeded wall-time deadline')
            return None
        if now-self.last_started >= self.interval_s:
            self.started = now
            self.last_started = now
            self.future = self.pool.submit(self.query)
        return None

    def close(self):
        self.pool.shutdown(wait=False, cancel_futures=True)


def sensor_ages(latest, now_wall, clock_sim, limits):
    """Return both ages and the exact sensor failures in a single time domain.

    ``latest`` values carry ``wall`` (monotonic receipt) and ``t`` (ROS sim
    header). A missing, repeated, future, or old stamp is reported separately.
    """
    ages = {}
    failures = []
    for name, limit in limits.items():
        item = latest.get(name)
        if item is None:
            failures.append(name+'_missing')
            ages[name] = None
            continue
        receive_age = now_wall-item['wall']
        stamp_age = clock_sim-item['t']
        ages[name] = {'receive_wall_s': receive_age,
                      'stamp_sim_s': stamp_age,
                      'limit_s': limit, 'stamp_sim': item['t']}
        if receive_age > limit:
            failures.append(name+'_receipt_stale')
        if stamp_age > limit:
            failures.append(name+'_stamp_stale')
        if stamp_age < -0.15:
            failures.append(name+'_stamp_future')
    return ages, failures
