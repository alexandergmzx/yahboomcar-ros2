"""robot1's canonical SLAM entry point (fleet D-19).

    ros2 launch yahboomcar_config slam_launch.py
    ros2 launch yahboomcar_config slam_launch.py rviz:=true

Substance ported from Alex's yahboomcar_nav/launch/slam_toolbox_launch.py
(that file stays untouched in the vendor package tree), with ONE deliberate
change, decided on measured data (D-19 / OI-14):

  THE LIFECYCLE BOND IS DISABLED (bond_timeout: 0.0). The vendor-side
  launch supervises with bond_timeout 30.0, and on the dev machine that
  manager declared "no heartbeat for 30000 ms" within ~200 ms of
  bond-connect and DEACTIVATED slam mid-session — 4 kill/respawn cycles in
  50 s [measured, session-3 fleet log], twice reproducible, on both robots'
  managers. The bond-disabled composition has run every robot2 gate since
  (hundreds of minutes) with zero deactivations, and this launch is gated
  the same way (session-4 regression: 0 deactivations in 300 s). In sim the
  manager's only needed job is configure->activate — the Jazzy lifecycle
  trap Alex documented in the original docstring: slam_toolbox comes up
  `unconfigured` and silently does nothing without it. Supervision for
  HARDWARE bringup remains OI-14's open half.

Params: this package's own copy of Alex's tuned slam_toolbox.yaml
(provenance header in the file); rviz view: the extracted slam_debug.rviz.

Needs a robot — real or simulated — publishing /scan and odom->base_footprint.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('yahboomcar_config')
    params = os.path.join(share, 'param', 'slam_toolbox.yaml')
    rviz_cfg = os.path.join(share, 'rviz', 'slam_debug.rviz')

    args = [
        DeclareLaunchArgument('params_file', default_value=params),
        DeclareLaunchArgument('rviz', default_value='false'),
        DeclareLaunchArgument('rviz_cfg', default_value=rviz_cfg),
        # FALSE for every live session, by design: nothing else in this stack sets
        # sim time, so swapping the simulator for the car changes nothing. It exists
        # for ONE caller -- tools/replay_slam_bag.py, which plays a recorded bag with
        # `--clock`. A replay must run on the bag's clock or every transform lookup is
        # compared against wall time and fails.
        DeclareLaunchArgument('use_sim_time', default_value='false'),
    ]
    use_sim_time = LaunchConfiguration('use_sim_time')

    slam = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[LaunchConfiguration('params_file'),
                    {'use_sim_time': use_sim_time}],
    )

    lifecycle = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_slam',
        output='screen',
        parameters=[{
            'autostart': True,
            'node_names': ['slam_toolbox'],
            'bond_timeout': 0.0,     # D-19 — see module docstring
            'use_sim_time': use_sim_time,
        }],
    )

    rviz = Node(
        package='rviz2', executable='rviz2', name='rviz2',
        arguments=['-d', LaunchConfiguration('rviz_cfg')],
        condition=IfCondition(LaunchConfiguration('rviz')),
    )

    return LaunchDescription(args + [slam, lifecycle, rviz])
