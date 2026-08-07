#!/usr/bin/env python3
"""A simulated Yahboom car that stands in for the FIRMWARE.

    ros2 run yahboomcar_sim fake_robot
    ros2 run yahboomcar_sim fake_robot --ros-args -p slip:=1.0     # car on a stand
    ros2 launch yahboomcar_sim sim_bringup_launch.py               # + the real stack

THE POINT
---------
This replaces the board, not the software. It publishes exactly what the firmware
publishes and subscribes to exactly what the firmware subscribes to -- the contract in
CLAUDE.md -- so yahboomcar_bringup, the EKF, cmd_vel_governor, cmd_vel_deadman,
laser_odometry, SLAM, Nav2 and RViz all run against it **completely unmodified**.

There is deliberately no "simulation mode" anywhere in the stack. Nothing above this node
knows it exists. So anything made to work here is the real stack working, not a
sim-shaped variant of it, and swapping in the car is turning this process off and the
robot on.

FAITHFUL TO THE AWKWARD PARTS
-----------------------------
Rates, QoS and quirks are the measured ones, not tidy ones:

  * /scan 360 beams at 1 deg, 0.12-8.0 m, ~12 Hz, angle_min -pi
  * /odom_raw at 11 Hz, /imu at 25 Hz, /battery at 1 Hz
  * BEST_EFFORT sensor QoS, which is what silently drops messages for a RELIABLE
    subscriber -- a real trap on this robot, so it is reproduced rather than smoothed
  * the IMU publishes gyro and accel and NO orientation, because it is 6-axis with no
    magnetometer
  * a commanded strafe produces exactly zero: the chassis is differential
  * NO COMMAND WATCHDOG. A commanded speed is held forever. Measured on the real
    firmware, and modelling it is what makes tools/test_failsafe.py meaningful here

WHAT IT CANNOT TELL YOU
-----------------------
There is no physics -- no wheel dynamics, no traction, no latency. Stopping distance,
odometry accuracy and braking are floor measurements. See docs/simulation-guide.md.
"""
import math

import numpy as np
import rclpy
from geometry_msgs.msg import Quaternion, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, LaserScan
from std_msgs.msg import UInt16

from yahboomcar_sim.arena import (LIDAR_BEAMS, LIDAR_RANGE_MAX, LIDAR_RANGE_MIN,
                                  default_arena, raycast)
from yahboomcar_sim.physics import (ACCEL_NOISE, FIRMWARE_MAX_SPEED,
                                    FIRMWARE_MAX_YAW, RobotState,
                                    apply_command, imu_sample, step)

# Measured rates of the real firmware.
SCAN_HZ = 12.0
ODOM_HZ = 11.0
IMU_HZ = 25.0
BATTERY_HZ = 1.0
PHYSICS_HZ = 100.0


class FakeRobot(Node):
    def __init__(self):
        super().__init__('fake_robot')
        self.declare_parameter('slip', 0.0)
        self.declare_parameter('scan_noise', 0.01)      # m, 1-sigma
        # m/s^2, 1-sigma. MEASURED: 0.0122-0.0307 across three at-rest selftest bags.
        self.declare_parameter('accel_noise', ACCEL_NOISE)
        # The firmware's acceptance caps. ASSUMPTIONS (vendor keyboard defaults),
        # not measurements -- see physics.FIRMWARE_MAX_SPEED's note.
        self.declare_parameter('max_speed', FIRMWARE_MAX_SPEED)
        self.declare_parameter('max_yaw', FIRMWARE_MAX_YAW)
        self.declare_parameter('dropout', 0.0)          # fraction of scans withheld
        self.declare_parameter('start_x', 0.0)
        self.declare_parameter('start_y', 0.0)
        self.declare_parameter('start_yaw', 0.0)
        self.declare_parameter('battery_volts', 8.3)
        self.declare_parameter('publish_ground_truth', True)
        # TEST FIXTURE, not a model of this robot -- see physics.py. Default 0 keeps the
        # honest no-physics behaviour; set it only to check a measurement tool against
        # KNOWN values.
        self.declare_parameter('decel', 0.0)
        self.declare_parameter('dead_time', 0.0)

        self.slip = float(self.get_parameter('slip').value)
        self.scan_noise = float(self.get_parameter('scan_noise').value)
        self.accel_noise = float(self.get_parameter('accel_noise').value)
        self.max_speed = float(self.get_parameter('max_speed').value)
        self.max_yaw = float(self.get_parameter('max_yaw').value)
        self.dropout = float(self.get_parameter('dropout').value)
        self.battery = float(self.get_parameter('battery_volts').value)
        self.decel = float(self.get_parameter('decel').value)
        self.dead_time = float(self.get_parameter('dead_time').value)

        self.segs = default_arena()
        self.state = RobotState(
            x=float(self.get_parameter('start_x').value),
            y=float(self.get_parameter('start_y').value),
            yaw=float(self.get_parameter('start_yaw').value))
        self.state.odom_x = self.state.x
        self.state.odom_y = self.state.y
        self.state.odom_yaw = self.state.yaw

        # THE COMMAND IS LATCHED, FOREVER. No timeout, no decay, exactly like the real
        # firmware: this is what makes a crashed publisher a runaway car, and modelling
        # it is the whole reason the deadman can be tested without hardware.
        self.cmd = (0.0, 0.0)
        self.rng = np.random.default_rng(0)

        self.create_subscription(Twist, '/cmd_vel', self._on_cmd, 10)

        self.pub_scan = self.create_publisher(LaserScan, '/scan', qos_profile_sensor_data)
        self.pub_odom = self.create_publisher(Odometry, '/odom_raw', qos_profile_sensor_data)
        self.pub_imu = self.create_publisher(Imu, '/imu', qos_profile_sensor_data)
        self.pub_batt = self.create_publisher(UInt16, '/battery', qos_profile_sensor_data)
        self.pub_truth = None
        if self.get_parameter('publish_ground_truth').value:
            # NOT part of the firmware contract. Published on a distinct topic so it can
            # never be mistaken for something the real robot provides -- it exists so
            # tests can score themselves against truth.
            self.pub_truth = self.create_publisher(Odometry, '/sim/ground_truth', 10)

        self.create_timer(1.0 / PHYSICS_HZ, self._tick_physics)
        # Ground truth is a DEBUG channel, not part of the firmware contract, so it is
        # published at physics rate rather than the firmware's 11 Hz. At 11 Hz a "tape"
        # reading taken from it is up to one sample period stale, which adds an error
        # PROPORTIONAL TO SPEED -- and a linear-in-v error is indistinguishable from dead
        # time, so it lands in T_stop and steals from the quadratic term. Measured: it
        # inflated T_stop by 47 ms (half a sample period) and dragged the recovered
        # deceleration from a true 1.5 to 0.54 m/s^2.
        self.create_timer(1.0 / PHYSICS_HZ, self._tick_truth)
        self.create_timer(1.0 / SCAN_HZ, self._tick_scan)
        self.create_timer(1.0 / ODOM_HZ, self._tick_odom)
        self.create_timer(1.0 / IMU_HZ, self._tick_imu)
        self.create_timer(1.0 / BATTERY_HZ, self._tick_battery)

        self._last = self.get_clock().now()
        self._vx = self._wz = 0.0

        self.get_logger().info(
            f'fake_robot up: 4x4 m arena, slip={self.slip:.2f}, '
            f'scan_noise={self.scan_noise:.3f} m, dropout={self.dropout:.2f}')
        self.get_logger().warn(
            'NO COMMAND WATCHDOG, as on the real firmware: a commanded speed is held '
            'indefinitely. This is modelled on purpose.')
        if self.decel > 0:
            self.get_logger().warn(
                f'BRAKING TEST FIXTURE ACTIVE: decel={self.decel} m/s^2, '
                f'dead_time={self.dead_time} s. These are ARBITRARY KNOWN VALUES for '
                'checking a measurement tool. They are not a model of the real car, '
                'whose braking has never been measured.')

    def _on_cmd(self, msg: Twist):
        # vy discarded: differential chassis, measured to produce exactly zero.
        self.cmd = apply_command(msg.linear.x, msg.linear.y, msg.angular.z,
                                 self.max_speed, self.max_yaw)

    def _tick_physics(self):
        now = self.get_clock().now()
        dt = (now - self._last).nanoseconds * 1e-9
        self._last = now
        if dt <= 0 or dt > 0.5:
            return
        vx, wz = self.cmd
        self.state, self._vx, self._wz = step(
            self.state, vx, wz, dt, slip=self.slip,
            decel=self.decel, dead_time=self.dead_time)

    def _stamp(self):
        return self.get_clock().now().to_msg()

    def _tick_scan(self):
        if self.dropout > 0 and self.rng.random() < self.dropout:
            return          # link stall: the real one has been seen between 4 and 13 Hz
        ranges = raycast((self.state.x, self.state.y), self.state.yaw, self.segs)
        if self.scan_noise > 0:
            finite = np.isfinite(ranges)
            ranges[finite] += self.rng.normal(0.0, self.scan_noise, finite.sum())
        m = LaserScan()
        m.header.stamp = self._stamp()
        m.header.frame_id = 'laser_frame'
        m.angle_min = -math.pi
        m.angle_max = math.pi - 2 * math.pi / LIDAR_BEAMS
        m.angle_increment = 2 * math.pi / LIDAR_BEAMS
        m.time_increment = 0.0
        m.scan_time = 1.0 / SCAN_HZ
        m.range_min = LIDAR_RANGE_MIN
        m.range_max = LIDAR_RANGE_MAX
        m.ranges = [float(r) for r in ranges]
        self.pub_scan.publish(m)

    def _tick_odom(self):
        m = Odometry()
        m.header.stamp = self._stamp()
        m.header.frame_id = 'odom'
        m.child_frame_id = 'base_footprint'
        # Wheel-derived pose: carries the slip error, exactly as the real one does.
        m.pose.pose.position.x = self.state.odom_x
        m.pose.pose.position.y = self.state.odom_y
        m.pose.pose.orientation = _quat(self.state.odom_yaw)
        m.twist.twist.linear.x = self._vx
        m.twist.twist.angular.z = self._wz
        self.pub_odom.publish(m)

    def _tick_truth(self):
        if self.pub_truth is None:
            return
        t = Odometry()
        t.header.stamp = self._stamp()
        t.header.frame_id = 'map'
        t.child_frame_id = 'base_footprint_truth'
        t.pose.pose.position.x = self.state.x
        t.pose.pose.position.y = self.state.y
        t.pose.pose.orientation = _quat(self.state.yaw)
        t.twist.twist.linear.x = self._vx
        self.pub_truth.publish(t)

    def _tick_imu(self):
        m = Imu()
        m.header.stamp = self._stamp()
        m.header.frame_id = 'imu_frame'
        # NO ORIENTATION. The ICM-42670-P is 6-axis with no magnetometer, so the firmware
        # has no absolute attitude to report. -1 in covariance[0] is the ROS convention
        # for "this field is not provided", and imu_filter_madgwick is what turns the
        # gyro and accel below into an orientation downstream.
        m.orientation_covariance[0] = -1.0
        # The BODY's rate, not the wheels': on a stand this reads ~0 while the encoders
        # report a brisk turn, which is the disagreement the whole fusion work is about.
        # Gravity is 9.799 and the accelerometer is noisy -- both measured, both in
        # physics.imu_sample, which documents why the gyro is left noiseless.
        gz, ax, ay, az = imu_sample(self.rng, self._wz, self.slip, self.accel_noise)
        m.angular_velocity.z = gz
        m.linear_acceleration.x = ax
        m.linear_acceleration.y = ay
        m.linear_acceleration.z = az
        self.pub_imu.publish(m)

    def _tick_battery(self):
        self.pub_batt.publish(UInt16(data=int(round(self.battery * 10))))


def _quat(yaw):
    return Quaternion(z=math.sin(yaw / 2.0), w=math.cos(yaw / 2.0))


def main(args=None):
    rclpy.init(args=args)
    node = FakeRobot()
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
