"""Start the lidar safety governor and the command deadman.

    ros2 launch yahboomcar_safety safety_launch.py

Drive via /cmd_vel_raw, never /cmd_vel. Teleop example:
    ros2 run teleop_twist_keyboard teleop_twist_keyboard \
      --ros-args -r /cmd_vel:=/cmd_vel_raw

The deadman starts by default and should stay that way. The firmware retains a commanded
speed indefinitely -- measured, see docs/safety-case.md -- so without it, a governor
crash leaves the car driving. It is a separate process precisely so it can outlive the
governor.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    args = [
        DeclareLaunchArgument('stop_distance', default_value='0.35',
                              description='hard stop inside this range, metres'),
        DeclareLaunchArgument('slow_distance', default_value='0.90',
                              description='begin scaling speed down here, metres'),
        DeclareLaunchArgument('max_speed', default_value='0.35',
                              description='absolute forward speed cap, m/s'),
        DeclareLaunchArgument('max_yaw', default_value='1.5',
                              description='absolute yaw cap, rad/s'),
        DeclareLaunchArgument('max_yaw_near', default_value='0.4',
                              description='yaw cap inside stop_distance, rad/s'),
        DeclareLaunchArgument('max_reverse_speed', default_value='0.10',
                              description='reverse cap, m/s -- reverse is sensor-blind'),
        DeclareLaunchArgument('deadman', default_value='true',
                              description='run the command deadman (keep this on)'),
        DeclareLaunchArgument('deadman_timeout', default_value='0.5',
                              description='seconds of /cmd_vel silence after a move '
                                          'command before zeroing'),
    ]

    governor = Node(
        package='yahboomcar_safety',
        executable='cmd_vel_governor',
        name='cmd_vel_governor',
        output='screen',
        parameters=[{
            'stop_distance': LaunchConfiguration('stop_distance'),
            'slow_distance': LaunchConfiguration('slow_distance'),
            'max_speed': LaunchConfiguration('max_speed'),
            'max_yaw': LaunchConfiguration('max_yaw'),
            'max_yaw_near': LaunchConfiguration('max_yaw_near'),
            'max_reverse_speed': LaunchConfiguration('max_reverse_speed'),
        }],
    )

    deadman = Node(
        package='yahboomcar_safety',
        executable='cmd_vel_deadman',
        name='cmd_vel_deadman',
        output='screen',
        parameters=[{'timeout': LaunchConfiguration('deadman_timeout')}],
        condition=IfCondition(LaunchConfiguration('deadman')),
    )

    return LaunchDescription(args + [governor, deadman])
