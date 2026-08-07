#!/usr/bin/env python3
"""Measure the REAL /scan rate and write it to a file. A feedback sensor, not a tool.

Spawned by sim_runner.py --ros (which runs in Isaac's Python 3.11 and cannot import
rclpy at all) so the render pacing can be closed-loop on the rate a subscriber actually
receives, rather than calibrated against an emission constant that has now drifted
twice: 72 messages per render-second in one arena build, ~84 in the next. A constant
measured yesterday is testimony; this file is the measurement.

Runs under SYSTEM python. Writes "<hz> <n_msgs> <unix_time>" atomically every period.
"""
import argparse
import os
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', required=True)
    ap.add_argument('--period', type=float, default=5.0)
    args = ap.parse_args()

    rclpy.init()
    n = Node('scan_rate_probe')
    stamps = []
    n.create_subscription(LaserScan, '/scan',
                          lambda m: stamps.append(time.time()),
                          qos_profile_sensor_data)
    tmp = args.out + '.tmp'
    while True:
        t_end = time.time() + args.period
        while time.time() < t_end:
            rclpy.spin_once(n, timeout_sec=0.2)
        cutoff = time.time() - args.period
        recent = [t for t in stamps if t >= cutoff]
        del stamps[:max(0, len(stamps) - len(recent) - 10)]
        hz = len(recent) / args.period
        with open(tmp, 'w') as f:
            f.write(f'{hz:.3f} {len(recent)} {time.time():.1f}\n')
        os.replace(tmp, args.out)      # atomic: the reader never sees a partial write


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        pass
    sys.exit(0)
