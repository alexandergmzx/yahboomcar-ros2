#!/usr/bin/env python3
"""Publish /joint_states for the Isaac Sim digital twin.

Why this node exists
--------------------
The microROS firmware exposes no per-wheel encoder state. Its topic contract is
/odom_raw, /imu, /scan, /battery out and /cmd_vel, /beep, /servo_s1, /servo_s2 in --
there is no wheel joint position anywhere. So the twin's wheels cannot be replayed from
measurement; they have to be *derived* from body twist by inverse kinematics and
integrated over time.

Be clear about what that means: the wheel angles produced here are **visual only**. They
make the twin's wheels turn plausibly in step with the robot's motion. They are not an
odometry source and must never be fed back into state estimation.

The gimbal is different -- /servo_s1 and /servo_s2 are real commanded angles, and map
directly onto jq1_Joint and jq2_Joint.

Geometry (measured, not guessed)
--------------------------------
Wheel radius 0.024 m, from the bounding box of zq_Link.STL (48 mm diameter, 21.5 mm
wide; the X and Z extents match, confirming rotation about Y). Chassis half-lengths come
from the URDF joint origins: front wheels at x=+0.0455, rear at x=-0.0495 (lx=0.0475),
track +/-0.0675 (ly=0.0675), so lx+ly = 0.115.

Chassis type: differential, measured not assumed
------------------------------------------------
The vendor package ships a `Mcnamu_driver_X3` executable, which suggests mecanum. This
board's firmware is not that. Commanding a pure strafe (0, +/-0.15, 0) produced
*exactly* zero on every axis of /odom_raw -- the motors never ran -- while forward and
yaw tracked their commands closely (0.170 for 0.15, 1.04 for 0.8). The firmware ignores
linear.y, so the default IK is differential. Set `mecanum: true` for the X3 variant.

Sign conventions
----------------
The URDF mirrors the right-hand wheels (axis 0,-1,0) against the left (axis 0,1,0), so a
positive joint value spins left and right wheels opposite ways in world terms. The IK
below assumes a single shared axis convention, so the right wheels get negated when
`mirror_right` is true.

Direction cannot be verified without the sim running, so every sign is a parameter. If
the twin's wheels spin backwards, flip `direction_sign`.
"""
import math

import rclpy
from rclpy.node import Node

from yahboomcar_twin.single_instance import (assert_sole_publisher,
                                             install_signal_handlers)
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from std_msgs.msg import Int32

# Wheel joints, in the order the IK below produces them.
# Names are Yahboom's pinyin abbreviations: z/y = zuo/you (left/right),
# q/h = qian/hou (front/rear).
WHEEL_LF = 'zq_Joint'   # left-front   origin (+0.0455, +0.0675)
WHEEL_RF = 'yq_Joint'   # right-front  origin (+0.0455, -0.0675)
WHEEL_RR = 'yh_Joint'   # right-rear   origin (-0.0495, -0.0675)
WHEEL_LR = 'zh_Joint'   # left-rear    origin (-0.0495, +0.0675)
WHEELS = [WHEEL_LF, WHEEL_RF, WHEEL_RR, WHEEL_LR]

GIMBAL_YAW = 'jq1_Joint'    # driven by /servo_s1, limit +/-1.57 rad
GIMBAL_PITCH = 'jq2_Joint'  # driven by /servo_s2, limit +/-1.57 rad


def wheel_rates(vx, vy, wz, r, ly, lxy, mecanum=False):
    """Body twist -> four wheel angular rates, in rad/s. ROS-free so it can be tested.

    Differential (skid-steer) by default. MEASURED, not assumed: commanding a pure
    strafe (0, +/-0.15, 0) produced EXACTLY zero on every axis of /odom_raw -- the motors
    never ran -- while forward and yaw both tracked their commands. The firmware ignores
    linear.y entirely.

    The mecanum branch is kept for the Mcnamu/X3 chassis variant the vendor package also
    supports, but it is off by default because this board's firmware is not it.
    """
    if r <= 0.0:
        raise ValueError('wheel_radius must be > 0')
    if mecanum:
        return {
            WHEEL_LF: (vx - vy - lxy * wz) / r,
            WHEEL_RF: (vx + vy + lxy * wz) / r,
            WHEEL_RR: (vx - vy + lxy * wz) / r,
            WHEEL_LR: (vx + vy - lxy * wz) / r,
        }
    left = (vx - wz * ly) / r
    right = (vx + wz * ly) / r
    return {WHEEL_LF: left, WHEEL_LR: left, WHEEL_RF: right, WHEEL_RR: right}


class JointStateBridge(Node):
    def __init__(self):
        super().__init__('twin_joint_state_bridge')

        self.declare_parameter('wheel_radius', 0.024)
        self.declare_parameter('lx', 0.0475)
        self.declare_parameter('ly', 0.0675)
        self.declare_parameter('publish_rate', 30.0)
        # /odom_raw, not /odom, and the reason matters. /odom is EKF-filtered and
        # fuses the IMU, so with the car on a stand it correctly reports ~zero motion
        # (the body really isn't moving) even while the wheels spin at 1 rad/s. For
        # animating WHEELS we want what the wheels did, which is /odom_raw. Base pose
        # still comes from the EKF's TF, so both stay correct on the bench and driving.
        self.declare_parameter('odom_topic', '/odom_raw')
        self.declare_parameter('joint_states_topic', '/joint_states')
        self.declare_parameter('mecanum', False)   # measured: this firmware ignores linear.y
        self.declare_parameter('mirror_right', True)
        self.declare_parameter('direction_sign', 1.0)
        # Stop spinning the wheels if odometry goes stale, so a paused bag or a
        # dropped link doesn't leave them turning forever.
        self.declare_parameter('odom_timeout', 0.5)

        self.r = self.get_parameter('wheel_radius').value
        self.ly = self.get_parameter('ly').value
        self.lxy = self.get_parameter('lx').value + self.ly
        self.mecanum = self.get_parameter('mecanum').value
        self.mirror_right = self.get_parameter('mirror_right').value
        self.dir = self.get_parameter('direction_sign').value
        self.odom_timeout = self.get_parameter('odom_timeout').value

        if self.r <= 0.0:
            raise ValueError('wheel_radius must be > 0')

        # Integrated wheel angles, radians.
        self.angles = dict.fromkeys(WHEELS, 0.0)
        self.twist = (0.0, 0.0, 0.0)   # vx, vy, wz
        self.gimbal = {GIMBAL_YAW: 0.0, GIMBAL_PITCH: 0.0}
        self.last_odom_time = None
        self.last_tick = self.get_clock().now()
        self._warned_stale = False

        self.create_subscription(
            Odometry, self.get_parameter('odom_topic').value, self._on_odom, 10)
        self.create_subscription(Int32, '/servo_s1', self._on_servo1, 10)
        self.create_subscription(Int32, '/servo_s2', self._on_servo2, 10)

        topic = self.get_parameter('joint_states_topic').value
        # Discovery needs ~1 s before the graph query is meaningful (measured).
        # The check must come BEFORE we create our own publisher, or we count ourselves.
        import time as _t
        deadline = _t.time() + 1.5
        while _t.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
        assert_sole_publisher(self, topic)

        self.pub = self.create_publisher(JointState, topic, 10)

        rate = self.get_parameter('publish_rate').value
        self.create_timer(1.0 / rate, self._tick)

        self.get_logger().info(
            f'twin bridge up: {"mecanum" if self.mecanum else "differential"}, '
            f'r={self.r} m, ly={self.ly:.4f} m, '
            f'{rate:.0f} Hz -> {self.get_parameter("joint_states_topic").value}')
        self.get_logger().warn(
            'wheel angles are DERIVED from body twist (the firmware has no wheel '
            'encoders) -- visual only, never use as an odometry source')

    def _on_odom(self, msg: Odometry):
        t = msg.twist.twist
        self.twist = (t.linear.x, t.linear.y, t.angular.z)
        self.last_odom_time = self.get_clock().now()

    def _on_servo1(self, msg: Int32):
        self.gimbal[GIMBAL_YAW] = math.radians(float(msg.data))

    def _on_servo2(self, msg: Int32):
        self.gimbal[GIMBAL_PITCH] = math.radians(float(msg.data))

    def _wheel_rates(self):
        """Thin wrapper over the module-level wheel_rates(), which carries the maths."""
        vx, vy, wz = self.twist
        return wheel_rates(vx, vy, wz, self.r, self.ly, self.lxy, self.mecanum)

    def _tick(self):
        now = self.get_clock().now()
        dt = (now - self.last_tick).nanoseconds * 1e-9
        self.last_tick = now
        if dt <= 0.0:
            return

        stale = (self.last_odom_time is None
                 or (now - self.last_odom_time).nanoseconds * 1e-9 > self.odom_timeout)
        if stale:
            if not self._warned_stale and self.last_odom_time is not None:
                self.get_logger().warn('odometry stale; holding wheels still')
                self._warned_stale = True
            self.twist = (0.0, 0.0, 0.0)
        else:
            self._warned_stale = False

        for name, rate in self._wheel_rates().items():
            sign = self.dir
            if self.mirror_right and name in (WHEEL_RF, WHEEL_RR):
                sign = -sign
            # Wrap to keep the value bounded over long runs; these joints are
            # continuous, so absolute winding carries no meaning.
            self.angles[name] = (self.angles[name] + sign * rate * dt) % (2.0 * math.pi)

        msg = JointState()
        msg.header.stamp = now.to_msg()
        msg.name = WHEELS + [GIMBAL_YAW, GIMBAL_PITCH]
        msg.position = [self.angles[w] for w in WHEELS] + \
                       [self.gimbal[GIMBAL_YAW], self.gimbal[GIMBAL_PITCH]]
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    try:
        node = JointStateBridge()
    except RuntimeError as e:
        print(f'\nrefusing to start:\n{e}\n')
        if rclpy.ok():
            rclpy.shutdown()
        return 1
    install_signal_handlers(lambda: node.get_logger().info('shutting down'))
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
