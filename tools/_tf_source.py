#!/usr/bin/env python3
"""Publish a known odom -> base_footprint TF trajectory. Test fixture, not a robot tool.

    python3 tools/_tf_source.py --hold 2.5

Runs under the SYSTEM python (ROS 2 Jazzy, 3.12), not Isaac's (3.11) -- rclpy cannot be
imported into Isaac's interpreter at all, the C extension ABI does not match. That is why
the twin's TF path has to go through the OmniGraph ROS 2 bridge, which is C++ and carries
its own ROS 2, and why verifying it needs two processes.

DISCRETE WAYPOINTS, NOT A SMOOTH PATH, on purpose. The verifier has to compare a pose in
Isaac against a pose published here, from two processes with no shared clock. Holding
each waypoint still for a couple of seconds makes the comparison insensitive to that:
"did the twin arrive at B" needs no time sync, whereas "was the twin at the right point
along a curve at t=1.37 s" would measure clock skew as much as anything else.
"""
import argparse
import math
import time

# (x, y, yaw). A square with a turn at each corner, so an axis swap or a sign error in
# the yaw path shows up as a gross failure rather than a subtle offset.
WAYPOINTS = [
    (0.0, 0.0, 0.0),
    (0.5, 0.0, 0.0),
    (0.5, 0.5, math.pi / 2),
    (0.0, 0.5, math.pi),
    (0.0, 0.0, -math.pi / 2),
]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--hold', type=float, default=2.5, help='seconds per waypoint')
    ap.add_argument('--rate', type=float, default=30.0)
    ap.add_argument('--domain', type=int, default=20)
    ap.add_argument('--loops', type=int, default=1)
    args = ap.parse_args()

    import os
    os.environ.setdefault('ROS_DOMAIN_ID', str(args.domain))

    import rclpy
    from rclpy.node import Node
    from geometry_msgs.msg import TransformStamped
    from nav_msgs.msg import Odometry
    from tf2_ros import TransformBroadcaster

    rclpy.init()
    node = Node('tf_source')
    br = TransformBroadcaster(node)
    # Both, because the twin reads /odom (a plain message the generic ROS2Subscriber
    # node can decode) while /tf is what a TF-tree consumer would want. Publishing the
    # same waypoints on both keeps the fixture honest whichever path is used.
    odom_pub = node.create_publisher(Odometry, 'odom', 10)

    def send(x, y, yaw):
        t = TransformStamped()
        t.header.stamp = node.get_clock().now().to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_footprint'
        t.transform.translation.x = float(x)
        t.transform.translation.y = float(y)
        t.transform.translation.z = 0.0
        t.transform.rotation.z = math.sin(yaw / 2.0)
        t.transform.rotation.w = math.cos(yaw / 2.0)
        br.sendTransform(t)

        o = Odometry()
        o.header.stamp = t.header.stamp
        o.header.frame_id = 'odom'
        o.child_frame_id = 'base_footprint'
        o.pose.pose.position.x = float(x)
        o.pose.pose.position.y = float(y)
        o.pose.pose.orientation.z = math.sin(yaw / 2.0)
        o.pose.pose.orientation.w = math.cos(yaw / 2.0)
        odom_pub.publish(o)

    period = 1.0 / args.rate
    for _ in range(args.loops):
        for i, (x, y, yaw) in enumerate(WAYPOINTS):
            print(f'waypoint {i}: x={x:.2f} y={y:.2f} yaw={math.degrees(yaw):.0f}',
                  flush=True)
            end = time.time() + args.hold
            while time.time() < end:
                send(x, y, yaw)
                rclpy.spin_once(node, timeout_sec=0.0)
                time.sleep(period)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
