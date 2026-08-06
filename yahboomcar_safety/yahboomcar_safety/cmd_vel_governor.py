#!/usr/bin/env python3
"""Lidar safety governor: /cmd_vel_raw + /scan -> /cmd_vel.

    ros2 run yahboomcar_safety cmd_vel_governor

Sits between anything that wants to drive the robot and the firmware:

    teleop / tests ---> /cmd_vel_raw ---> [governor] ---> /cmd_vel ---> firmware
                                              ^
                                            /scan

Every driving tool must publish to /cmd_vel_raw. Anything publishing directly to
/cmd_vel bypasses this entirely, so the governor warns when it sees another publisher
there -- a bypass that goes unnoticed is the failure mode that matters.

The decision logic lives in governor.py, deliberately free of ROS types so it can be
unit-tested exhaustively without hardware. See test/test_governor.py.

LIMITATIONS, stated plainly:
  * The lidar sector faces FORWARD. Nothing here protects the rear, and reversing is
    deliberately unrestricted.
  * It limits commands; it cannot exceed the robot's own braking. Stopping distance is
    a measured quantity, not a guarantee -- see docs/safety-governor.md.
  * It assumes /scan is trustworthy. A lidar reporting confidently wrong ranges defeats
    it, which is why a stale or empty scan stops the robot rather than being ignored.
"""
import math

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Float32

from yahboomcar_safety.governor import GovernorConfig, decide, forward_min_range


class CmdVelGovernor(Node):
    def __init__(self):
        super().__init__('cmd_vel_governor')

        cfg = GovernorConfig()
        self.declare_parameter('stop_distance', cfg.stop_distance)
        self.declare_parameter('slow_distance', cfg.slow_distance)
        self.declare_parameter('sector_half_angle', cfg.sector_half_angle)
        self.declare_parameter('max_speed', cfg.max_speed)
        self.declare_parameter('max_yaw', cfg.max_yaw)
        self.declare_parameter('max_yaw_near', cfg.max_yaw_near)
        self.declare_parameter('max_reverse_speed', cfg.max_reverse_speed)
        self.declare_parameter('allow_lateral', cfg.allow_lateral)
        self.declare_parameter('scan_timeout', cfg.scan_timeout)
        self.declare_parameter('cmd_timeout', cfg.cmd_timeout)
        self.declare_parameter('rate', 20.0)

        self.cfg = GovernorConfig(
            stop_distance=self.get_parameter('stop_distance').value,
            slow_distance=self.get_parameter('slow_distance').value,
            sector_half_angle=self.get_parameter('sector_half_angle').value,
            max_speed=self.get_parameter('max_speed').value,
            max_yaw=self.get_parameter('max_yaw').value,
            max_yaw_near=self.get_parameter('max_yaw_near').value,
            max_reverse_speed=self.get_parameter('max_reverse_speed').value,
            allow_lateral=self.get_parameter('allow_lateral').value,
            scan_timeout=self.get_parameter('scan_timeout').value,
            cmd_timeout=self.get_parameter('cmd_timeout').value,
        )

        self.min_range = math.inf
        self.last_scan = None
        self.last_cmd = None
        self.req = (0.0, 0.0, 0.0)
        self._last_reason = None
        self._warned_bypass = set()

        self.create_subscription(LaserScan, '/scan', self._on_scan,
                                 qos_profile_sensor_data)
        self.create_subscription(Twist, '/cmd_vel_raw', self._on_cmd, 10)

        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        # Published so an operator can see the governor's view without reading logs.
        self.pub_range = self.create_publisher(Float32, '~/forward_range', 10)
        self.pub_limiting = self.create_publisher(Bool, '~/limiting', 10)

        rate = self.get_parameter('rate').value
        self.create_timer(1.0 / rate, self._tick)
        self.create_timer(5.0, self._check_bypass)

        self.get_logger().info(
            f'governor up: stop<{self.cfg.stop_distance} m, slow<{self.cfg.slow_distance} m, '
            f'sector +/-{math.degrees(self.cfg.sector_half_angle):.0f} deg, '
            f'max {self.cfg.max_speed} m/s')
        self.get_logger().warn(
            'drive via /cmd_vel_raw. Publishing to /cmd_vel directly bypasses this node.')

    def _on_scan(self, msg: LaserScan):
        self.min_range = forward_min_range(list(msg.ranges), msg.angle_min,
                                           msg.angle_increment, self.cfg)
        self.last_scan = self.get_clock().now()

    def _on_cmd(self, msg: Twist):
        self.req = (msg.linear.x, msg.linear.y, msg.angular.z)
        self.last_cmd = self.get_clock().now()

    def _age(self, stamp):
        if stamp is None:
            return None
        return (self.get_clock().now() - stamp).nanoseconds * 1e-9

    def _bypassing_nodes(self):
        """Which nodes publish /cmd_vel besides us.

        A count alone ("2 publishers are bypassing me") is not actionable at 2am with a
        robot moving. Naming them turns it into "yahboom_keyboard is bypassing me".
        """
        me = (self.get_namespace().rstrip('/') + '/' + self.get_name()).replace('//', '/')
        names = []
        for name, ns in self.get_node_names_and_namespaces():
            full = (ns.rstrip('/') + '/' + name).replace('//', '/')
            if full == me:
                continue
            try:
                for topic, _types in self.get_publisher_names_and_types_by_node(name, ns):
                    if topic == '/cmd_vel':
                        names.append(full)
                        break
            except Exception:
                continue          # node vanished between listing and querying
        return names

    def _check_bypass(self):
        names = self._bypassing_nodes()
        if not names:
            self._warned_bypass = set()
            return
        current = set(names)
        if current != getattr(self, '_warned_bypass', set()):
            self.get_logger().error(
                f'BYPASSED by {len(names)} publisher(s) on /cmd_vel: {", ".join(names)}. '
                'These drive the robot with NO obstacle limiting. Remap them to '
                '/cmd_vel_raw, or accept that they are unprotected.')
            self._warned_bypass = current

    def _tick(self):
        vx, vy, wz = self.req
        d = decide(vx, vy, wz, self.min_range,
                   self._age(self.last_scan), self._age(self.last_cmd), self.cfg)

        out = Twist()
        out.linear.x, out.linear.y, out.angular.z = d.vx, d.vy, d.wz
        self.pub.publish(out)

        self.pub_range.publish(Float32(
            data=float(self.min_range if math.isfinite(self.min_range) else -1.0)))
        self.pub_limiting.publish(Bool(data=d.limited))

        # Log transitions only; a message every tick would bury the interesting ones.
        if d.reason != self._last_reason:
            if d.reason:
                self.get_logger().info(f'limiting: {d.reason}')
            elif self._last_reason:
                self.get_logger().info('clear; passing commands through')
            self._last_reason = d.reason


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelGovernor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Leave the robot stopped, whatever happened.
        try:
            node.pub.publish(Twist())
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
