#!/usr/bin/env python3
"""Publish lidar-derived odometry on /odom_laser.

    ros2 run yahboomcar_localization laser_odometry

Subscribes /scan, matches consecutive scans, integrates the result into a pose, and
publishes nav_msgs/Odometry. Deliberately does NOT broadcast a TF: the EKF owns
odom->base_footprint, and a second broadcaster of the same transform is the bug this
repo already documents for yahboomcar_base_node.

WHAT THIS IS FOR
----------------
It is a SECOND OPINION, not a replacement. It drifts too -- every scan-to-scan estimate
does -- but it drifts for unrelated reasons, so where it disagrees with the wheels the
disagreement is informative. Wheel slip is invisible to encoders and obvious here.

COVARIANCE IS DERIVED FROM GEOMETRY, NOT GUESSED
------------------------------------------------
A scan matcher can only resist motion along a surface normal, so in a corridor -- or a
flat-walled room -- the along-wall direction is unconstrained and the estimate there is
meaningless however confidently ICP converged. The published covariance is inflated along
the weak eigenvector by the measured isotropy, and degenerate matches are dropped
entirely rather than published with a large number attached. A filter given a bad
measurement with an honest covariance still degrades; one given a bad measurement with a
confident covariance diverges.

THE SENSOR IS NOT THE ROBOT
---------------------------
Scan matching measures the motion of the LASER, and this publishes the motion of
base_footprint. Those differ whenever the lidar sits off the robot's centre of rotation:
a pure turn about base_footprint drags an offset sensor sideways, and reporting that as
base motion would be simply wrong.

On this robot the offset is small. `/scan` is stamped `laser_frame`, which the URDF does
not define at all -- it has `radar_Link` -- so the number comes from the vendor's own nav
launches: x = -4.6 mm, y = 0. At the maximum per-scan rotation the car can produce
(0.125 rad at 12 Hz) that is **0.58 mm**, well under the ~8 mm the matcher is good for.

So the transform is applied not because it changes the answer today, but because "the
lidar is close enough to the centre" was an unstated assumption, and unstated assumptions
are what stop being true quietly when a sensor gets moved.

When the transform is unavailable -- bag replay has no TF tree -- it falls back to the
identity, warns exactly once, and keeps publishing. Refusing to run would make the node
useless for offline analysis, which is where most of its testing happens.

UNVALIDATED ON A MOVING ROBOT ON THE FLOOR
------------------------------------------
As of writing this has been exercised against recorded bags and against the car on its
stand, where it correctly reports no translation while the wheels spin. It has NOT been
run on a robot driving across a floor, because that room was not available. It must not
be fused into anything safety-bearing until it has.
"""
import math

import numpy as np
import rclpy
import tf2_ros
from geometry_msgs.msg import Quaternion
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

from yahboomcar_localization.scan_geometry import (compose, laser_step_to_base_step,
                                                   scan_to_xy, wrap)
from yahboomcar_localization.scan_matcher import estimate_motion


class LaserOdometry(Node):
    def __init__(self):
        super().__init__('laser_odometry')
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('publish_topic', '/odom_laser')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('min_points', 40)
        # Both of these are SAFETY NETS that no longer bind on the fast path, and they
        # are kept because the slow path still exists.
        #
        # With scipy's cKDTree, full-resolution 355-point scans run at mean 15.8 ms and
        # p95 39.1 ms against the 83 ms interval -- 0/99 over budget, so no decimation is
        # needed and the extra points are free accuracy. Without scipy the same work is
        # 6.1x slower (mean 110 ms, p95 485 ms) and both limits become load-bearing.
        #
        # max_points is therefore set high enough not to bind on a 360-beam lidar, and
        # the time budget stays as the thing that keeps the numpy fallback real-time.
        self.declare_parameter('max_points', 1000)
        self.declare_parameter('time_budget', 0.05)
        self.declare_parameter('publish_degenerate', False)
        # Baseline sigmas for a well-conditioned match. 5 mm and 10 mrad are of the order
        # measured on synthetic scenes at the car's real per-scan motion; they are scaled
        # up by the observed geometry, never down.
        self.declare_parameter('sigma_xy', 0.005)
        self.declare_parameter('sigma_yaw', 0.010)

        self.min_points = self.get_parameter('min_points').value
        self.max_points = self.get_parameter('max_points').value
        self.time_budget = self.get_parameter('time_budget').value
        self.publish_degenerate = self.get_parameter('publish_degenerate').value
        self.sigma_xy = self.get_parameter('sigma_xy').value
        self.sigma_yaw = self.get_parameter('sigma_yaw').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value

        # base_footprint <- laser_frame, resolved lazily from TF on first use.
        self.declare_parameter('laser_frame', 'laser_frame')
        self.laser_frame = self.get_parameter('laser_frame').value
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.base_from_laser = None       # (dx, dy, dyaw), None until resolved
        self._warned_no_tf = False

        self.pose = (0.0, 0.0, 0.0)
        self.prev_xy = None
        self.prev_stamp = None
        self.n_matched = self.n_dropped = self.n_degenerate = self.n_over = 0

        self.create_subscription(LaserScan, self.get_parameter('scan_topic').value,
                                 self._on_scan, qos_profile_sensor_data)
        self.pub = self.create_publisher(
            Odometry, self.get_parameter('publish_topic').value, 10)
        self.create_timer(10.0, self._report)

        self.get_logger().info(
            f'laser odometry up: {self.get_parameter("scan_topic").value} -> '
            f'{self.get_parameter("publish_topic").value}')
        self.get_logger().warn(
            'SECOND OPINION ONLY. Never validated on a robot driving across a floor; '
            'do not fuse into a safety path until it has been.')

    def _resolve_extrinsic(self):
        """base_footprint <- laser_frame as a planar (dx, dy, dyaw). Cached once found."""
        if self.base_from_laser is not None:
            return self.base_from_laser
        try:
            t = self.tf_buffer.lookup_transform(
                self.base_frame, self.laser_frame, rclpy.time.Time())
        except Exception:
            if not self._warned_no_tf:
                self._warned_no_tf = True
                self.get_logger().warn(
                    f'no {self.base_frame} <- {self.laser_frame} transform; treating the '
                    'lidar as coincident with the base. Correct under bag replay, where '
                    'there is no TF tree. On the robot it means bringup is not running, '
                    'and a real sensor offset would go uncorrected.')
            return None
        q = t.transform.rotation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.base_from_laser = (t.transform.translation.x,
                                t.transform.translation.y, yaw)
        self.get_logger().info(
            f'{self.base_frame} <- {self.laser_frame}: '
            f'x={self.base_from_laser[0]*1000:.1f} mm '
            f'y={self.base_from_laser[1]*1000:.1f} mm '
            f'yaw={math.degrees(yaw):.2f} deg')
        return self.base_from_laser

    def _report(self):
        total = self.n_matched + self.n_dropped
        if total:
            self.get_logger().info(
                f'matched {self.n_matched}/{total}, {self.n_degenerate} degenerate, '
                f'{self.n_over} over time budget')

    def _on_scan(self, msg: LaserScan):
        xy = scan_to_xy(list(msg.ranges), msg.angle_min, msg.angle_increment,
                        range_min=max(msg.range_min, 0.05), range_max=msg.range_max)
        stamp = msg.header.stamp

        if self.prev_xy is None or len(xy) < self.min_points:
            if len(xy) >= self.min_points:
                self.prev_xy, self.prev_stamp = xy, stamp
            return

        # Uniform decimation, not truncation: taking the first N points would keep one
        # angular sector and throw the rest of the room away, which is a fast way to
        # manufacture the degeneracy this node exists to detect.
        # CEIL, not floor. With 355 points and a 180 cap, 355 // 180 == 1, so a floor
        # step decimates by 1 -- i.e. not at all -- and the cap silently does nothing.
        # Measured: timing was identical at 355 and "180" points until this was fixed.
        def thin(pts):
            if len(pts) <= self.max_points:
                return pts
            return pts[:: -(-len(pts) // self.max_points)]

        a, b = thin(self.prev_xy), thin(xy)

        r = estimate_motion(a, b, time_budget=self.time_budget)
        if r.budget_exceeded:
            self.n_over += 1
        self.prev_xy, self.prev_stamp = xy, stamp

        if not r.converged:
            self.n_dropped += 1
            return
        deg = r.degeneracy
        if deg is not None and deg.degenerate:
            self.n_degenerate += 1
            if not self.publish_degenerate:
                # Dropping beats publishing a number the geometry cannot support. The
                # filter then coasts on its other inputs, which is the correct behaviour
                # for a missing measurement and not for a wrong one.
                self.n_dropped += 1
                return

        # Laser-frame motion -> base-frame motion. A no-op when the extrinsic is the
        # identity, which is the fallback when no TF is available. See
        # scan_geometry.laser_step_to_base_step for the derivation.
        step = (r.dx, r.dy, r.dtheta)
        E = self._resolve_extrinsic()
        if E is not None:
            step = laser_step_to_base_step(step, E)

        self.n_matched += 1
        self.pose = compose(step, self.pose)

        out = Odometry()
        out.header.stamp = stamp
        out.header.frame_id = self.odom_frame
        out.child_frame_id = self.base_frame
        out.pose.pose.position.x = self.pose[0]
        out.pose.pose.position.y = self.pose[1]
        out.pose.pose.orientation = Quaternion(
            z=math.sin(self.pose[2] / 2.0), w=math.cos(self.pose[2] / 2.0))

        cov = np.zeros((6, 6))
        sx = sy = self.sigma_xy
        if deg is not None:
            # Inflate along the weak eigenvector in proportion to how badly it is
            # constrained. isotropy -> 1 leaves it alone; isotropy -> 0 blows it up.
            iso = max(deg.isotropy, 1e-3)
            weak = np.array(deg.weak_dir)
            strong = np.array(deg.strong_dir)
            C = (self.sigma_xy ** 2) * (
                np.outer(strong, strong) + np.outer(weak, weak) / iso)
            cov[0, 0], cov[0, 1] = C[0, 0], C[0, 1]
            cov[1, 0], cov[1, 1] = C[1, 0], C[1, 1]
            sx, sy = math.sqrt(C[0, 0]), math.sqrt(C[1, 1])
        else:
            cov[0, 0] = sx ** 2
            cov[1, 1] = sy ** 2
        cov[5, 5] = self.sigma_yaw ** 2
        # Unobservable in 2D. Large, not zero: zero reads as perfect knowledge.
        cov[2, 2] = cov[3, 3] = cov[4, 4] = 1e6
        out.pose.covariance = cov.flatten().tolist()

        # Twist is left unset and its covariance made huge, on purpose. Publishing both
        # pose and twist derived from the same match is exactly the double-counting that
        # makes the vendor ekf.yaml overconfident -- pose IS the integral of twist, so
        # fusing both feeds one measurement in twice. See docs/sensor-fusion-research.md.
        tcov = np.zeros((6, 6))
        np.fill_diagonal(tcov, 1e6)
        out.twist.covariance = tcov.flatten().tolist()

        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = LaserOdometry()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
