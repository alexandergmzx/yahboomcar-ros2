"""Keyboard teleop with the lidar governor in the loop.

    ros2 launch yahboomcar_safety safe_teleop_launch.py

Starts the governor and the vendor keyboard node, with the keyboard REMAPPED onto
/cmd_vel_raw so its commands are speed-limited against /scan before reaching the
firmware.

The vendor node itself is not modified. Running it the way the course PDFs document --

    ros2 run yahboomcar_ctrl yahboom_keyboard

-- still publishes directly to /cmd_vel and is therefore UNPROTECTED. That is a
deliberate scope decision, not an oversight: the PDFs stay correct, and the governor
names any bypassing publisher in its log so the gap is visible rather than silent.

See docs/safety-case.md for exactly which paths are protected and which are not.

NOTE: yahboom_keyboard reads the terminal, so it needs its own tty. If it does not
respond to keys, run the governor from this launch file and the keyboard separately:

    ros2 launch yahboomcar_safety safety_launch.py
    ros2 run yahboomcar_ctrl yahboom_keyboard --ros-args -r /cmd_vel:=/cmd_vel_raw
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
        DeclareLaunchArgument('keyboard', default_value='true',
                              description='also start the vendor keyboard node'),
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

    # Separate process on purpose: it exists to outlive a governor crash. The firmware
    # retains a commanded speed forever, so without this a SIGKILLed governor leaves the
    # car driving -- measured, docs/safety-case.md.
    deadman = Node(
        package='yahboomcar_safety',
        executable='cmd_vel_deadman',
        name='cmd_vel_deadman',
        output='screen',
        condition=IfCondition(LaunchConfiguration('deadman')),
    )

    # The vendor node is unmodified; only its output topic is remapped here.
    keyboard = Node(
        package='yahboomcar_ctrl',
        executable='yahboom_keyboard',
        name='yahboom_keyboard',
        output='screen',
        remappings=[('/cmd_vel', '/cmd_vel_raw')],
        condition=IfCondition(LaunchConfiguration('keyboard')),
    )

    return LaunchDescription(args + [governor, deadman, keyboard])
