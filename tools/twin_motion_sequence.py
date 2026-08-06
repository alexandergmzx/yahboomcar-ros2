#!/usr/bin/env python3
"""Drive a scripted motion sequence, for capturing a twin-development dataset.

Exercises every degree of freedom the digital twin has to reproduce: all three mecanum
axes (forward/back, strafe, yaw) and both gimbal servos. Run it while `ros2 bag record`
is capturing.

    ros2 bag record -o MicroROS-assets/bags/twin_dataset /odom /odom_raw /imu ...
    ./tools/twin_motion_sequence.py

SAFETY: run this only with the car ELEVATED (wheels off the ground) unless you have
clear floor space. It commands real motion. A stop is always sent on exit, including
on Ctrl-C.

Note the odometry this produces is not physically meaningful: with the wheels off the
ground there is no traction, so /odom reports travel that never happened. That is fine
for animating a twin -- which is the point -- but the bag must not be used to judge
odometry accuracy or to build a map.
"""
import sys
import time

import argparse

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32, UInt16

LIN = 0.15   # m/s
STRAFE = 0.15
YAW = 0.8    # rad/s


class Sequencer(Node):
    def __init__(self, direct=False):
        super().__init__('twin_motion_sequence')
        # Through the governor by default; this drives real wheels.
        topic = '/cmd_vel' if direct else '/cmd_vel_raw'
        self.get_logger().info(
            f'commanding on {topic}'
            + ('  (DIRECT -- governor bypassed)' if direct else '  (via governor)'))
        self.cmd = self.create_publisher(Twist, topic, 10)
        self.s1 = self.create_publisher(Int32, '/servo_s1', 10)
        self.s2 = self.create_publisher(Int32, '/servo_s2', 10)
        self.beep = self.create_publisher(UInt16, '/beep', 10)

    def drive(self, vx, vy, wz, secs, label):
        self.get_logger().info(f'{label}: vx={vx} vy={vy} wz={wz} for {secs}s')
        t = Twist()
        t.linear.x, t.linear.y, t.angular.z = float(vx), float(vy), float(wz)
        end = time.time() + secs
        while time.time() < end:
            self.cmd.publish(t)
            time.sleep(0.05)

    def stop(self, secs=1.0):
        self.drive(0.0, 0.0, 0.0, secs, 'stop')

    def sweep(self, pub, lo, hi, label, step=10, dwell=0.12):
        self.get_logger().info(f'{label}: {lo} -> {hi} -> {lo} deg')
        rng = list(range(lo, hi + 1, step))
        for a in rng + rng[::-1]:
            pub.publish(Int32(data=int(a)))
            time.sleep(dwell)
        pub.publish(Int32(data=0))
        time.sleep(0.3)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--direct', action='store_true',
                    help='bypass the safety governor (elevated bench use only)')
    args = ap.parse_args()
    rclpy.init()
    n = Sequencer(direct=args.direct)
    try:
        n.get_logger().info('baseline idle')
        n.stop(3.0)

        n.drive(LIN, 0, 0, 4.0, 'forward')
        n.stop()
        n.drive(-LIN, 0, 0, 4.0, 'backward')
        n.stop()

        n.drive(0, STRAFE, 0, 4.0, 'strafe left')
        n.stop()
        n.drive(0, -STRAFE, 0, 4.0, 'strafe right')
        n.stop()

        n.drive(0, 0, YAW, 4.0, 'rotate CCW')
        n.stop()
        n.drive(0, 0, -YAW, 4.0, 'rotate CW')
        n.stop()

        # A combined move: mecanum can do all three at once, and the twin must
        # reproduce that, not just the pure axes.
        n.drive(LIN * 0.7, STRAFE * 0.7, YAW * 0.5, 4.0, 'combined x+y+yaw')
        n.stop(2.0)

        # Gimbal: s1 spans -90..90, s2 is asymmetric at -90..20.
        n.sweep(n.s1, -60, 60, 'servo_s1 (yaw)')
        n.sweep(n.s2, -60, 20, 'servo_s2 (pitch)')

        n.beep.publish(UInt16(data=200))
        n.get_logger().info('sequence complete')
    except KeyboardInterrupt:
        n.get_logger().warn('interrupted')
    finally:
        # Always leave the robot stopped, whatever happened above.
        t = Twist()
        for _ in range(10):
            n.cmd.publish(t)
            time.sleep(0.05)
        n.get_logger().info('stopped')
        n.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass
    return 0


if __name__ == '__main__':
    sys.exit(main())
