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

THE ONE EXCEPTION, AND IT IS A TEST FIXTURE
-------------------------------------------
`decel` and `dead_time` below give the simulated robot a FINITE braking response instead
of stopping dead. They are NOT a model of this car -- nobody has measured how it brakes,
which is the entire point of the floor session.

They exist so that tools/measure_braking.py can be run end to end against KNOWN values
and checked for whether it recovers them. That tool has never executed its full chain --
calibrate, drive, measure, subtract the run-up, fit, envelope -- and a bug anywhere in it
would otherwise be discovered on the floor, wasting the session it was meant to serve.

Numbers produced this way describe the fixture, never the robot. Any figure derived from
them must be reported as a check on the INSTRUMENT.
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
    # Test-fixture braking state: the speed actually being executed, and how long is left
    # of the dead time before the current command takes effect at all.
    exec_vx: float = 0.0
    exec_wz: float = 0.0
    pending: float = 0.0
    cmd_vx: float = 0.0        # the last command SEEN, so dead time starts on a change


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def step(state, vx, wz, dt, slip=0.0, decel=0.0, dead_time=0.0):
    """Advance one tick. Returns (new_state, reported_vx, reported_wz).

    `slip` in [0, 1] is the fraction of commanded motion the wheels turn through WITHOUT
    the body moving: 0 is perfect traction, 1 is the car on its stand. The encoders report
    the full commanded motion regardless, which is exactly why slip is invisible to them
    and visible to the lidar.

    `decel` (m/s^2) and `dead_time` (s) are a TEST FIXTURE, not a model of this robot --
    see the module docstring. With decel = 0 the command takes effect instantly, which is
    the default and the honest behaviour for a simulator with no physics.
    """
    s = RobotState(state.x, state.y, state.yaw,
                   state.odom_x, state.odom_y, state.odom_yaw,
                   state.exec_vx, state.exec_wz, state.pending, state.cmd_vx)

    if decel <= 0.0:
        s.exec_vx, s.exec_wz = vx, wz          # no fixture: instant, as before
    else:
        # Dead time starts when the COMMAND CHANGES -- not whenever the executed speed
        # happens to differ from it. Comparing against exec_vx re-armed the timer on
        # every tick while the robot was still catching up, so it never expired and the
        # robot never moved at all. Found by running the protocol end to end.
        if abs(vx - s.cmd_vx) > 1e-9:
            s.pending = dead_time
            s.cmd_vx = vx
        if s.pending > 0.0:
            s.pending = max(0.0, s.pending - dt)
        else:
            # Then approach the commanded speed at a bounded rate.
            dv = vx - s.exec_vx
            step_v = decel * dt
            s.exec_vx += math.copysign(min(abs(dv), step_v), dv) if dv else 0.0
            s.exec_wz = wz

    ex, ew = s.exec_vx, s.exec_wz

    # True motion: what the body actually does.
    real_vx = ex * (1.0 - slip)
    real_wz = ew * (1.0 - slip)
    s.x += real_vx * math.cos(s.yaw) * dt
    s.y += real_vx * math.sin(s.yaw) * dt
    s.yaw = wrap(s.yaw + real_wz * dt)

    # Wheel-derived motion: what the encoders think happened. Slip never appears here --
    # a wheel cannot tell that the ground moved past it rather than under it.
    s.odom_x += ex * math.cos(s.odom_yaw) * dt
    s.odom_y += ex * math.sin(s.odom_yaw) * dt
    s.odom_yaw = wrap(s.odom_yaw + ew * dt)

    return s, ex, ew


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


# ------------------------------------------------------------------------- IMU
# MEASURED from three at-rest selftest bags (MicroROS-assets/bags/selftest-*):
#
#   bag                        accel_z mean   accel_z std   accel_x std   accel_y std
#   selftest-20260806-004114        9.8005        0.01216       0.01705       0.01216
#   selftest-20260806-004659        9.7983        0.03072       0.01437       0.01083
#   selftest-20260806-004728        9.7980        0.01260       0.01451       0.01251
#
# Gravity here is 9.799, NOT 9.81 -- that is what this IMU actually reports, and the
# difference is larger than the noise.
GRAVITY = 9.799
ACCEL_NOISE = 0.013


def imu_sample(rng, wz, slip=0.0, accel_noise=ACCEL_NOISE, gravity=GRAVITY):
    """One IMU reading: (gyro_z, accel_x, accel_y, accel_z).

    The accelerometer is NOISY on purpose. A perfectly constant 9.81 is indistinguishable
    from a dead accelerometer, and tools/sensor_health.py flags a zero-variance channel as
    stuck -- correctly, since a constant reads downstream as a confident measurement. The
    simulator failed that check for a real reason before this existed; the fix belonged
    here, not in the check.

    The GYRO is deliberately noiseless. Every at-rest bag from this robot shows gyro_z std
    of exactly 0.00000, so there is no measurement of what a healthy gyro does at rest --
    this one is intermittently faulty. Inventing noise would make the simulated gyro look
    demonstrably alive while stationary when the real one does not, and would defeat
    sensor_health.py's rotate window, which exists precisely because a gyro cannot be
    judged at rest.

    `slip` attenuates the gyro but never the accelerometer: slip means the wheels turn and
    the body does not, so the BODY's rate is what falls. Gravity is unaffected by it.
    """
    return (
        wz * (1.0 - slip),
        float(rng.normal(0.0, accel_noise)),
        float(rng.normal(0.0, accel_noise)),
        float(rng.normal(gravity, accel_noise)),
    )
