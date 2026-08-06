#!/usr/bin/env python3
"""Command deadman: zeros /cmd_vel when whoever was driving stops talking.

    ros2 run yahboomcar_safety cmd_vel_deadman

The firmware retains a commanded speed forever -- measured, see docs/safety-case.md. This
node watches /cmd_vel and, if the last command was a move and nothing has followed it
within `timeout`, publishes zeros.

Deliberately minimal. It does no lidar processing, no geometry, no parameter juggling
past construction, because its whole value is being the process that does NOT crash when
the governor does. Every line here is a line that could take it down with the thing it
is supposed to outlive.

SELF-TRIGGERING
---------------
Its own zeros arrive back on its own subscription. That is harmless and in fact useful:
once it has published a zero, the last-seen command is a stop, so it settles. But a
single zero could be dropped -- /cmd_vel crosses Wi-Fi and UDP to reach the board -- so
once triggered it LATCHES and republishes zeros for `hold` seconds rather than trusting
one message to arrive. It only publishes zeros, so any non-zero message is definitely
someone else resuming control, and that releases the latch immediately.
"""
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node

from yahboomcar_safety.deadman import DeadmanConfig, is_moving_command, should_intervene


class CmdVelDeadman(Node):
    def __init__(self):
        super().__init__('cmd_vel_deadman')

        d = DeadmanConfig()
        self.declare_parameter('timeout', d.timeout)
        self.declare_parameter('hold', d.hold)
        self.declare_parameter('rate', 20.0)
        self.cfg = DeadmanConfig(
            timeout=self.get_parameter('timeout').value,
            hold=self.get_parameter('hold').value,
        )

        self.last_moving = False
        self.last_time = None
        self.holding_until = None
        self._triggers = 0

        self.create_subscription(Twist, '/cmd_vel', self._on_cmd, 10)
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)

        rate = self.get_parameter('rate').value
        self.create_timer(1.0 / rate, self._tick)

        self.get_logger().info(
            f'deadman up: zeroing /cmd_vel after {self.cfg.timeout}s of silence '
            f'following a move command, held for {self.cfg.hold}s')
        self.get_logger().warn(
            'This does NOT cover Wi-Fi loss, agent death or a PC freeze -- the firmware '
            'keeps driving through all three. Keep a hand on the power switch.')

    def _on_cmd(self, msg: Twist):
        moving = is_moving_command(msg.linear.x, msg.linear.y, msg.angular.z, self.cfg)
        if moving and self.holding_until is not None:
            # Only we publish zeros, so a non-zero command is someone else taking back
            # control. Release immediately rather than fighting them for `hold` seconds.
            self.get_logger().info('command resumed; releasing deadman')
            self.holding_until = None
        self.last_moving = moving
        self.last_time = self.get_clock().now()

    def _age(self):
        if self.last_time is None:
            return None
        return (self.get_clock().now() - self.last_time).nanoseconds * 1e-9

    def _tick(self):
        now = self.get_clock().now()

        if self.holding_until is not None:
            if now < self.holding_until:
                self.pub.publish(Twist())
                return
            self.holding_until = None
            self.get_logger().info('deadman released; car left stopped')

        if should_intervene(self.last_moving, self._age(), self.cfg):
            self._triggers += 1
            self.get_logger().error(
                f'DEADMAN TRIGGERED (#{self._triggers}): last command was a move and '
                f'/cmd_vel has been silent for {self._age():.2f}s. Whoever was driving '
                'is gone. Sending zeros -- the firmware would otherwise hold that '
                'speed indefinitely.')
            self.holding_until = now + rclpy.duration.Duration(seconds=self.cfg.hold)
            self.pub.publish(Twist())


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelDeadman()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.pub.publish(Twist())
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
