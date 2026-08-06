"""Start the lidar safety governor.

    ros2 launch yahboomcar_safety safety_launch.py

Drive via /cmd_vel_raw, never /cmd_vel. Teleop example:
    ros2 run teleop_twist_keyboard teleop_twist_keyboard \
      --ros-args -r /cmd_vel:=/cmd_vel_raw
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
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
    ]
    return LaunchDescription(args + [
        Node(
            package='yahboomcar_safety',
            executable='cmd_vel_governor',
            name='cmd_vel_governor',
            output='screen',
            parameters=[{
                'stop_distance': LaunchConfiguration('stop_distance'),
                'slow_distance': LaunchConfiguration('slow_distance'),
                'max_speed': LaunchConfiguration('max_speed'),
            }],
        ),
    ])
