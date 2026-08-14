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
  * The lidar sector faces FORWARD. Nothing here protects the rear or the sides.
    Reverse and near-obstacle yaw stay possible but are bounded, not unrestricted.
  * cmd_vel_deadman is an EXPECTED co-publisher on /cmd_vel, not a bypass. Its absence
    is warned about, because the firmware retains commands forever without it.
  * It limits commands; it cannot exceed the robot's own braking. Stopping distance is
    a measured quantity, not a guarantee -- see docs/safety-governor.md.
  * It assumes /scan is trustworthy. A lidar reporting confidently wrong ranges defeats
    it, which is why a stale or empty scan stops the robot rather than being ignored.
"""
import math

import rclpy
from geometry_msgs.msg import Twist, Vector3Stamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Float32

from yahboomcar_safety.governor import (DockingApproach, EXPECTED_PUBLISHERS,
                                        GovernorConfig, bypassing_nodes, decide,
                                        disc_from_declaration,
                                        forward_min_range)


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
        self.declare_parameter('expected_publishers', list(EXPECTED_PUBLISHERS))
        # How long a declared terminal approach stays valid without a refresh.
        # Matched to `scan_timeout` deliberately: the mask is only ever as
        # trustworthy as the scan that justified it, so it must not outlive one.
        self.declare_parameter('docking_timeout', cfg.scan_timeout)
        # The slack added around the declared target silhouette. The FILTER's
        # number, not the caller's: it covers declaration staleness (about 4 mm
        # of translation per scan period at creep speed, plus up to ~34 mm of
        # lateral shift during a pivot), and it must stay well under the
        # clearance to the nearest real hazard.
        self.declare_parameter('docking_margin', 0.10)
        # The largest target a caller may declare. The radius is the one number
        # in a declaration that WIDENS the masked region, so it is bounded here
        # rather than trusted. 0.25 m comfortably covers the delivery target and
        # is far short of anything that would hide a wall.
        self.declare_parameter('docking_max_target_radius', 0.25)

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

        self.cfg_docking_timeout = self.get_parameter('docking_timeout').value
        self.cfg_docking_margin = self.get_parameter('docking_margin').value
        self.cfg_docking_max_radius = self.get_parameter(
            'docking_max_target_radius').value

        self.min_range = math.inf
        self.last_scan = None
        self.last_cmd = None
        # Terminal-approach state. Absent until somebody declares one, and
        # expiring on silence -- see `_live_docking`.
        self.docking_request = None
        self.last_docking = None
        self.docking_disc_request = None
        self.last_docking_disc = None
        self.req = (0.0, 0.0, 0.0)
        self._last_reason = None
        self._warned_bypass = set()
        self._seen_expected = set()

        self.create_subscription(LaserScan, '/scan', self._on_scan,
                                 qos_profile_sensor_data)
        self.create_subscription(Twist, '/cmd_vel_raw', self._on_cmd, 10)
        # A docking controller declares its approach here. Deliberately NOT a
        # service or a latched topic: the mask must expire on silence.
        self.create_subscription(Vector3Stamped, '~/docking_approach',
                                 self._on_docking, 10)
        # And here, in the shape that can actually admit a contact. A SEPARATE
        # topic rather than a reinterpretation of the old one, because the
        # third field changes meaning -- margin there, target radius here --
        # and a stale sender must not have its margin read as a radius.
        self.create_subscription(Vector3Stamped, '~/docking_disc',
                                 self._on_docking_disc, 10)

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
                                           msg.angle_increment, self.cfg,
                                           docking=self._live_docking())
        self.last_scan = self.get_clock().now()

    def _on_docking(self, msg: Vector3Stamped):
        """A terminal approach, declared by whoever is doing the docking.

        `x` is the bearing to the confirmed target in the laser frame, `y` its
        measured range, `z` the margin. The caller must keep republishing as it
        closes; see `_live_docking` for why that is a safety property and not an
        inconvenience.
        """

        self.docking_request = DockingApproach(
            bearing_rad=float(msg.vector.x),
            range_m=float(msg.vector.y),
            margin_m=float(msg.vector.z) if msg.vector.z > 0.0 else 0.10,
        )
        self.last_docking = self.get_clock().now()

    def _on_docking_disc(self, msg: Vector3Stamped):
        """A terminal approach shaped like the TARGET. `x` bearing, `y` range,
        `z` the target's authored radius.

        The margin is NOT taken from the sender. It is this node's own
        `docking_margin` parameter, because the slack on a safety mask belongs
        to the filter, not to the thing asking to be let through. For the same
        reason the radius is bounded: it is the one caller-supplied number that
        WIDENS the masked region, and `docking_max_target_radius` bounds how
        much of the world a bad declaration can hide.

        The validation itself lives in `governor.disc_from_declaration` so it
        can be tested without a ROS graph -- see the refusal cases there.
        """

        disc = disc_from_declaration(
            float(msg.vector.x), float(msg.vector.y), float(msg.vector.z),
            margin_m=self.cfg_docking_margin,
            max_target_radius_m=self.cfg_docking_max_radius,
        )
        if disc is None:
            self.get_logger().warn(
                f'docking disc REFUSED: bearing {msg.vector.x:.3f} rad, range '
                f'{msg.vector.y:.3f} m, radius {msg.vector.z:.3f} m (max '
                f'{self.cfg_docking_max_radius}). A refused declaration is not '
                f'clamped into something acceptable: it expires on the ordinary '
                f'timeout and the robot stops.')
            return
        self.docking_disc_request = disc
        self.last_docking_disc = self.get_clock().now()

    def _live_docking(self):
        """The approach, but only while it is FRESH.

        The mode expires on silence rather than on an explicit exit message, so
        every way of losing the docking controller -- it crashes, it is killed,
        its topic is starved, the network drops -- removes the mask by the same
        path. There is no 'exit' message to go missing, which is the failure an
        explicit stop command would have.

        A fresh disc wins over a fresh cone. They expire independently, so a
        controller that switches to the disc and stops sending the cone loses
        the cone on the ordinary timeout, and one still sending only the cone
        keeps the old behaviour.
        """

        age = self._age(self.last_docking_disc)
        if age is not None and age <= self.cfg_docking_timeout:
            return self.docking_disc_request
        age = self._age(self.last_docking)
        if age is None or age > self.cfg_docking_timeout:
            return None
        return self.docking_request

    def _on_cmd(self, msg: Twist):
        self.req = (msg.linear.x, msg.linear.y, msg.angular.z)
        self.last_cmd = self.get_clock().now()

    def _age(self, stamp):
        if stamp is None:
            return None
        return (self.get_clock().now() - stamp).nanoseconds * 1e-9

    def _cmd_vel_publishers(self):
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
        names, expected = bypassing_nodes(
            self._cmd_vel_publishers(),
            (self.get_namespace().rstrip('/') + '/' + self.get_name()).replace('//', '/'),
            tuple(self.get_parameter('expected_publishers').value))

        # Report the safety companion POSITIVELY rather than merely not complaining about
        # it. Its absence is the dangerous state -- the firmware retains commands forever,
        # so with no deadman a crashed driver leaves the car driving.
        if set(expected) != self._seen_expected:
            self._seen_expected = set(expected)
            if expected:
                self.get_logger().info(
                    f'safety companion present on /cmd_vel: {", ".join(expected)}')
        if not expected:
            self.get_logger().warn(
                'NO DEADMAN on /cmd_vel. If whatever is driving dies, the firmware will '
                'hold the last command indefinitely. Do not run on the floor like this.')

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
        docking = self._live_docking()
        d = decide(vx, vy, wz, self.min_range,
                   self._age(self.last_scan), self._age(self.last_cmd), self.cfg,
                   docking=docking)

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
