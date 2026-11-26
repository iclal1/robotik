#!/usr/bin/env python3
"""Exercise async Gazebo polling while ROS sensor callbacks keep running."""
import argparse
import json
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, LaserScan
from std_msgs.msg import String

from nav_trial_monitor import AsyncGazeboSampler, read_gazebo_poses, sensor_ages


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--seconds', type=float, default=8)
    parser.add_argument('--inject-query-delay', type=float, default=.55)
    args = parser.parse_args()
    rclpy.init()
    node = Node('phase5e2_stationary_monitor')
    latest = {}
    stamps = {k: [] for k in ('odom', 'scan', 'camera')}
    def record(name, message):
        current = time.monotonic()
        stamp = message.header.stamp.sec + message.header.stamp.nanosec*1e-9
        latest[name] = {'t': stamp, 'wall': current}
        stamps[name].append((current, stamp))
    node.create_subscription(Clock, '/clock',
        lambda m: latest.update({'clock': m.clock.sec+m.clock.nanosec*1e-9}), 10)
    node.create_subscription(Odometry, '/odom',
        lambda m: record('odom', m), 10)
    node.create_subscription(LaserScan, '/scan',
        lambda m: record('scan', m), qos_profile_sensor_data)
    node.create_subscription(Image, '/camera/image_raw',
        lambda m: record('camera', m), qos_profile_sensor_data)
    node.create_subscription(String, '/nav2/safety_status',
        lambda m: latest.update({'safety': json.loads(m.data)}), 10)
    def delayed_query():
        time.sleep(args.inject_query_delay)
        return read_gazebo_poses()
    sampler = AsyncGazeboSampler(query=delayed_query, interval_s=.4,
                                 deadline_s=2.5)
    query_durations = []
    failures = []
    max_ages = {k: {'receipt_wall_s': 0., 'stamp_sim_s': 0.}
                for k in stamps}
    started = time.monotonic()
    try:
        while time.monotonic()-started < args.seconds:
            rclpy.spin_once(node, timeout_sec=.01)
            sample = sampler.tick()
            if sample is not None:
                query_durations.append(sample['duration_wall_s'])
            if 'clock' not in latest or any(k not in latest for k in stamps):
                continue
            ages, bad = sensor_ages(latest, time.monotonic(), latest['clock'],
                                     {'odom': .3, 'scan': .4, 'camera': .6})
            if time.monotonic()-started > 2:
                for k in stamps:
                    max_ages[k]['receipt_wall_s'] = max(
                        max_ages[k]['receipt_wall_s'], ages[k]['receive_wall_s'])
                    max_ages[k]['stamp_sim_s'] = max(
                        max_ages[k]['stamp_sim_s'], ages[k]['stamp_sim_s'])
                if bad:
                    failures.append({'wall': time.monotonic(),
                                     'sim': latest['clock'], 'failures': bad,
                                     'ages': ages})
    finally:
        sampler.close()
        node.destroy_node()
        rclpy.shutdown()
    def gaps(values):
        return {'max_callback_wall_s': max((b[0]-a[0]
                for a,b in zip(values,values[1:])), default=None),
                'max_stamp_sim_s': max((b[1]-a[1]
                for a,b in zip(values,values[1:])), default=None)}
    out = {'duration_wall_s': time.monotonic()-started,
           'injected_query_delay_s': args.inject_query_delay,
           'gazebo_queries_completed': len(query_durations),
           'query_durations_wall_s': query_durations,
           'sensor_counts': {k: len(v) for k,v in stamps.items()},
           'sensor_gaps': {k: gaps(v) for k,v in stamps.items()},
           'max_ages': max_ages, 'freshness_failures': failures,
           'last_safety': latest.get('safety')}
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(out, indent=2)+'\n')
    print(json.dumps({k:v for k,v in out.items() if k!='freshness_failures'},
                     indent=2))


if __name__ == '__main__':
    main()
