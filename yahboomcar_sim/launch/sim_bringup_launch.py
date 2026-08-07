"""The simulated robot, with the REAL stack on top of it.

    ros2 launch yahboomcar_sim sim_bringup_launch.py
    ros2 launch yahboomcar_sim sim_bringup_launch.py slip:=1.0     # car on a stand

This is the whole argument for building the simulator the way it is built. `fake_robot`
stands in for the FIRMWARE, and everything above it -- imu_filter_madgwick, the EKF, the
static transforms, the robot description -- is `yahboomcar_bringup_launch.py`, included
unmodified. Not a copy, not a sim variant: the same file that runs on the real car.

So the TF tree, the fused /odom and the whole sensor chain here are produced by exactly
the code that will produce them on the floor. What is being tested is the stack; only the
robot is fake.

Swapping in the real car is turning this launch off and the robot on.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    args = [
        DeclareLaunchArgument('slip', default_value='0.0',
                             description='0 = perfect traction, 1 = car on its stand '
                                         '(wheels turn, body still)'),
        DeclareLaunchArgument('scan_noise', default_value='0.01',
                             description='lidar range noise, metres 1-sigma'),
        DeclareLaunchArgument('dropout', default_value='0.0',
                             description='fraction of scans withheld; the real link has '
                                         'been seen between 4 and 13 Hz'),
        DeclareLaunchArgument('start_x', default_value='0.0'),
        DeclareLaunchArgument('start_y', default_value='0.0'),
        DeclareLaunchArgument('start_yaw', default_value='0.0'),
    ]

    fake_robot = Node(
        package='yahboomcar_sim',
        executable='fake_robot',
        name='fake_robot',
        output='screen',
        parameters=[{
            'slip': LaunchConfiguration('slip'),
            'scan_noise': LaunchConfiguration('scan_noise'),
            'dropout': LaunchConfiguration('dropout'),
            'start_x': LaunchConfiguration('start_x'),
            'start_y': LaunchConfiguration('start_y'),
            'start_yaw': LaunchConfiguration('start_yaw'),
        }],
    )

    # The real bringup, unmodified: imu_filter_madgwick -> /imu/data, the EKF ->
    # /odom + odom->base_footprint, the static base_link->imu and base_link->laser
    # transforms, and the robot description.
    real_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('yahboomcar_bringup'), 'launch'),
            '/yahboomcar_bringup_launch.py',
        ])
    )

    return LaunchDescription(args + [
        LogInfo(msg='SIMULATED ROBOT. fake_robot replaces the firmware; everything '
                    'above it is the real stack, included unmodified.'),
        LogInfo(msg='It has NO command watchdog, exactly like the real board.'),
        fake_robot,
        real_bringup,
    ])
