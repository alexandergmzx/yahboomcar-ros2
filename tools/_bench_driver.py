#!/usr/bin/env python3
"""bench_motion.py's driver child: publish one twist at 20 Hz until killed.

    python3 _bench_driver.py <vx> <wz> <topic>

Deliberately has NO cleanup handler of any kind: bench_motion's latch phases exist
to prove what the firmware does when a publisher dies WITHOUT sending a zero, so a
zero escaping from here would destroy the very scenario under test. The parent owns
the finalizer (SafeCmdVel) and the caps; values arriving here are already clamped.
"""
import sys
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


def main():
    vx, wz, topic = float(sys.argv[1]), float(sys.argv[2]), sys.argv[3]
    rclpy.init()
    n = Node('bench_driver')
    p = n.create_publisher(Twist, topic, 10)
    # Publishing before discovery matches goes nowhere; wait for a subscriber.
    t0 = time.time()
    while p.get_subscription_count() == 0 and time.time() - t0 < 10.0:
        time.sleep(0.1)
    m = Twist()
    m.linear.x, m.angular.z = vx, wz
    while True:
        p.publish(m)
        time.sleep(0.05)


if __name__ == '__main__':
    main()
