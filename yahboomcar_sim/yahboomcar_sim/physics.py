"""Differential-drive motion, and the ways this particular robot's odometry lies.

ROS-free so every claim about the motion model is testable without a graph running.

WHAT IS MODELLED, AND WHY EACH ONE
----------------------------------
Only behaviours that were MEASURED on the real car, because a simulator that invents
plausible-looking dynamics is worse than one that is honestly simple -- it produces
confident numbers about a robot that does not exist.

  differential drive     A commanded strafe produces EXACTLY zero on every /odom_raw
                         axis. Measured. vy is not attenuated here, it is dropped.

  no command watchdog    A commanded speed is retained INDEFINITELY. Measured three ways
                         and reconfirmed after a power cycle: the car held 0.15 m/s for
                         45 s with nothing publishing. This is the single most
                         safety-relevant property of the hardware, and modelling it means
                         cmd_vel_deadman and tools/test_failsafe.py can be exercised with
                         no car at all.

  slip (opt-in)          Wheels turn, world does not. On the stand this reads as 92%
                         translational slip. Injecting it is how SLAM and Nav2 get tested
                         against odometry that lies, which is the realistic case.

WHAT IS NOT MODELLED
--------------------
Wheel dynamics, motor spin-up, inertia, traction limits, latency. There is no physics
here at all -- just kinematics. So this simulator can say whether the navigation stack
WORKS; it can say nothing whatever about how the robot BEHAVES. Stopping distance,
odometry accuracy and braking are floor measurements and this cannot substitute for them.
"""
import math
from dataclasses import dataclass


@dataclass
class RobotState:
    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0
    # What the wheels believe, which diverges from the truth above whenever there is slip.
    odom_x: float = 0.0
    odom_y: float = 0.0
    odom_yaw: float = 0.0


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def step(state, vx, wz, dt, slip=0.0):
    """Advance one tick. Returns (new_state, reported_vx, reported_wz).

    `slip` in [0, 1] is the fraction of commanded motion the wheels turn through WITHOUT
    the body moving: 0 is perfect traction, 1 is the car on its stand. The encoders report
    the full commanded motion regardless, which is exactly why slip is invisible to them
    and visible to the lidar.
    """
    s = RobotState(state.x, state.y, state.yaw,
                   state.odom_x, state.odom_y, state.odom_yaw)

    # True motion: what the body actually does.
    real_vx = vx * (1.0 - slip)
    real_wz = wz * (1.0 - slip)
    s.x += real_vx * math.cos(s.yaw) * dt
    s.y += real_vx * math.sin(s.yaw) * dt
    s.yaw = wrap(s.yaw + real_wz * dt)

    # Wheel-derived motion: what the encoders think happened. Slip never appears here --
    # a wheel cannot tell that the ground moved past it rather than under it.
    s.odom_x += vx * math.cos(s.odom_yaw) * dt
    s.odom_y += vx * math.sin(s.odom_yaw) * dt
    s.odom_yaw = wrap(s.odom_yaw + wz * dt)

    return s, vx, wz


def apply_command(vx, vy, wz, max_speed=0.35, max_yaw=1.5):
    """What the firmware does with a Twist.

    vy is DISCARDED, not scaled: the chassis is differential, and a commanded strafe
    produces exactly zero on every /odom_raw axis. Measured on the real car -- the
    `Mcnamu_driver_X3` executable in the vendor package suggests mecanum, and the firmware
    disagrees.
    """
    del vy
    vx = max(-max_speed, min(max_speed, vx))
    wz = max(-max_yaw, min(max_yaw, wz))
    return vx, wz
