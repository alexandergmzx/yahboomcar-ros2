"""The profile for the car's FIRST runs on the floor. Deliberately crippled.

    ros2 launch yahboomcar_safety first_floor_launch.py

Read docs/first-floor-procedure.md before using this. It is not a launch file so much as
one step of a written procedure.

WHY THE LIMITS ARE WHERE THEY ARE
---------------------------------
0.05 m/s is walking-alongside-slowly speed. It is chosen so that when -- not if -- the
car ignores everything this stack does and keeps driving, you can catch it on foot and
reach the power switch. That is the actual last line of defence, because the firmware
retains a commanded speed indefinitely and no software on this PC can revoke it once the
Wi-Fi drops. Measured three ways; see docs/safety-case.md.

Yaw is zero and reverse is zero because the first floor session has exactly one job:
measure straight-line stopping distance. Anything that is not that is a way to lose the
car under a piece of furniture.

The stop distance stays at the un-derived 0.35 m. That number has no physics behind it
yet -- deriving it is what these runs are FOR -- so it stays conservative until
tools/measure_braking.py has produced real values.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    args = [
        DeclareLaunchArgument('max_speed', default_value='0.05',
                              description='FIRST-FLOOR CAP. Raise only per the staging '
                                          'table in docs/first-floor-procedure.md'),
        DeclareLaunchArgument('stop_distance', default_value='0.35'),
        DeclareLaunchArgument('slow_distance', default_value='0.90'),
    ]

    governor = Node(
        package='yahboomcar_safety',
        executable='cmd_vel_governor',
        name='cmd_vel_governor',
        output='screen',
        parameters=[{
            'max_speed': LaunchConfiguration('max_speed'),
            'stop_distance': LaunchConfiguration('stop_distance'),
            'slow_distance': LaunchConfiguration('slow_distance'),
            'max_yaw': 0.0,             # no rotation
            'max_yaw_near': 0.0,
            'max_reverse_speed': 0.0,   # no reverse
            'allow_lateral': False,
        }],
    )

    # Not optional here. Unlike safety_launch.py, this profile exposes no argument to
    # turn it off -- a first floor session without it is not a thing worth offering.
    deadman = Node(
        package='yahboomcar_safety',
        executable='cmd_vel_deadman',
        name='cmd_vel_deadman',
        output='screen',
        parameters=[{'timeout': 0.5}],
    )

    return LaunchDescription(args + [
        LogInfo(msg='FIRST-FLOOR PROFILE: forward only, no yaw, no reverse.'),
        LogInfo(msg='Keep a hand on the power switch. The firmware has no watchdog: '
                    'if the Wi-Fi drops, nothing here can stop the car.'),
        governor, deadman,
    ])
