#!/usr/bin/env python3
"""Time-aligned comparison of wheel-odometry yaw rate against the IMU gyro.

    ./tools/check_odom_vs_imu.py MicroROS-assets/bags/<bag-with-traction>

On selftest-20260806-004826 (the first recording with traction) this gives correlation
+0.977 and a median IMU/odom ratio of 0.928 -- the odometry over-reports yaw by ~7.7%.

Peak-vs-peak agreement can be luck. This resamples both onto a common timebase and
compares them where the robot was actually turning, which is the honest version of the
question "does the odometry agree with an independent sensor?"

Only meaningful for a bag recorded WITH TRACTION. Elevated, the IMU correctly reads zero
and the comparison is vacuous.
"""
import os
import sys

from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions
from rclpy.serialization import deserialize_message
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu

path = sys.argv[1]
r = SequentialReader()
r.open(StorageOptions(uri=path, storage_id='mcap'), ConverterOptions('cdr', 'cdr'))

odom, imu = [], []
while r.has_next():
    topic, data, t = r.read_next()
    if topic == '/odom_raw':
        odom.append((t * 1e-9, deserialize_message(data, Odometry).twist.twist.angular.z))
    elif topic == '/imu':
        imu.append((t * 1e-9, deserialize_message(data, Imu).angular_velocity.z))

if not odom or not imu:
    sys.exit('need both /odom_raw and /imu')

# Nearest-neighbour resample of the IMU onto odom timestamps (odom is the slower
# stream). Shared implementation: the private copy of this loop deadlocked on
# timestamp ties (strict `<` never crossed equal-stamp runs; audit 2026-08-10
# measured 8/7,541 pairs on a bag where nearly all should pair) — see
# tools/_pairing.py for the fix and its test.
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from _pairing import nearest_pairs                              # noqa: E402
pairs = nearest_pairs(odom, imu, max_dt=0.15)

turning = [(a, b) for a, b in pairs if abs(a) > 0.2]   # where the robot was rotating
print(f'samples paired      : {len(pairs)}')
print(f'samples while turning: {len(turning)}')
if not turning:
    print('no rotation in this bag; nothing to compare')
    sys.exit(0)

# Sign convention may differ; report both raw and sign-aligned agreement.
num = sum(a * b for a, b in turning)
den_a = sum(a * a for a, b in turning) ** 0.5
den_b = sum(b * b for a, b in turning) ** 0.5
corr = num / (den_a * den_b) if den_a and den_b else 0.0

ratios = [b / a for a, b in turning if abs(a) > 0.3]
ratios.sort()
median_ratio = ratios[len(ratios) // 2] if ratios else float('nan')
errs = [abs(abs(b) - abs(a)) for a, b in turning]
mean_abs_err = sum(errs) / len(errs)

print(f'correlation (signed) : {corr:+.3f}')
print(f'median IMU/odom ratio: {median_ratio:+.3f}   (1.0 = perfect agreement)')
print(f'mean |error|         : {mean_abs_err:.3f} rad/s')
print()
if abs(corr) > 0.8 and 0.75 < abs(median_ratio) < 1.25:
    print('=> wheel odometry AGREES with the gyro. Yaw odometry is trustworthy enough')
    print('   to calibrate against; UMBmark is worth running.')
elif abs(corr) > 0.8:
    print(f'=> shapes agree but SCALE is off by {abs(median_ratio):.2f}x. That is exactly')
    print('   the systematic error UMBmark measures -- worth running, expect a large Eb.')
else:
    print('=> poor agreement. Investigate before trusting odometry-based work.')
