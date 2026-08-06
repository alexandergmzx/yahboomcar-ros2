#!/usr/bin/env python3
"""Did the BODY move, or only the wheels?

    ./tools/check_body_moved.py MicroROS-assets/bags/<bag> [...]

Validated against real recordings: elevated runs report IMU wz exactly 0.000 while the
wheels spin at 0.7-0.9 rad/s; the one desk run (selftest-20260806-004826) reports IMU wz
0.719 against odom 0.693.

The encoders say the wheels turned. The IMU is an independent witness to whether the
chassis actually went anywhere: elevated on a stand, wheels spin while the IMU reads
~zero; on a desk with traction, the IMU must register the rotation and acceleration.
"""
import sys

from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions
from rclpy.serialization import deserialize_message
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist


def analyse(path):
    r = SequentialReader()
    r.open(StorageOptions(uri=path, storage_id='mcap'), ConverterOptions('cdr', 'cdr'))
    odom_wz = odom_vx = 0.0
    imu_wz = imu_ax = 0.0
    cmd_seen = False
    n_imu = n_odom = 0
    while r.has_next():
        topic, data, _t = r.read_next()
        if topic == '/odom_raw':
            m = deserialize_message(data, Odometry)
            odom_wz = max(odom_wz, abs(m.twist.twist.angular.z))
            odom_vx = max(odom_vx, abs(m.twist.twist.linear.x))
            n_odom += 1
        elif topic == '/imu':
            m = deserialize_message(data, Imu)
            imu_wz = max(imu_wz, abs(m.angular_velocity.z))
            imu_ax = max(imu_ax, abs(m.linear_acceleration.x))
            n_imu += 1
        elif topic == '/cmd_vel':
            m = deserialize_message(data, Twist)
            if abs(m.linear.x) > 0.01 or abs(m.angular.z) > 0.01:
                cmd_seen = True
    return dict(odom_vx=odom_vx, odom_wz=odom_wz, imu_wz=imu_wz, imu_ax=imu_ax,
                n_imu=n_imu, n_odom=n_odom, cmd=cmd_seen)


print(f'{"bag":<34}{"odom vx":>9}{"odom wz":>9}{"IMU wz":>9}{"IMU |ax|":>10}  verdict')
print('-' * 88)
for path in sys.argv[1:]:
    try:
        a = analyse(path)
    except Exception as e:
        print(f'{path.split("/")[-1]:<34}  unreadable: {e}')
        continue
    name = path.rstrip('/').split('/')[-1]
    if not a['cmd']:
        verdict = 'no motion commanded'
    elif a['imu_wz'] > 0.15:
        verdict = 'BODY ROTATED -- real motion, traction'
    elif a['odom_wz'] > 0.3:
        verdict = 'wheels spun, body still -> elevated'
    else:
        verdict = 'inconclusive'
    print(f'{name:<34}{a["odom_vx"]:>9.3f}{a["odom_wz"]:>9.3f}'
          f'{a["imu_wz"]:>9.3f}{a["imu_ax"]:>10.3f}  {verdict}')
