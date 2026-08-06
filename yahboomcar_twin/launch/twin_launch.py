"""Digital-twin support stack.

Starts the joint-state bridge and the robot description, with the vendor's zero-emitting
joint_state_publisher suppressed so it does not fight the bridge on /joint_states.

Does NOT start the micro-ROS agent or the EKF -- run yahboomcar_bringup for those, or
replay a bag. Typical uses:

    # against the real car (agent + bringup already running)
    ros2 launch yahboomcar_twin twin_launch.py

    # offline, against a recording
    ros2 bag play --loop MicroROS-assets/bags/twin_dataset
    ros2 launch yahboomcar_twin twin_launch.py use_description:=false
"""
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_description = DeclareLaunchArgument(
        'use_description', default_value='true',
        description='Also start robot_state_publisher and the URDF. Set false when a '
                    'bag already supplies /robot_description and /tf.')

    odom_topic = DeclareLaunchArgument(
        'odom_topic', default_value='/odom',
        description='Body twist source. /odom is EKF-filtered; /odom_raw is the '
                    'firmware estimate.')

    bridge = Node(
        package='yahboomcar_twin',
        executable='joint_state_bridge',
        name='twin_joint_state_bridge',
        output='screen',
        parameters=[{
            'odom_topic': LaunchConfiguration('odom_topic'),
            # Measured from zq_Link.STL, not guessed: 48 mm diameter.
            'wheel_radius': 0.024,
            # From the URDF joint origins: front x=+0.0455, rear x=-0.0495, track +/-0.0675.
            'lx': 0.0475,
            'ly': 0.0675,
            'publish_rate': 30.0,
            # The URDF mirrors the right wheels (axis 0,-1,0) against the left (0,1,0).
            'mirror_right': True,
            'direction_sign': 1.0,
        }],
    )

    description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('yahboomcar_description'), 'launch'),
            '/description_launch.py',
        ]),
        # The bridge is the real source of joint states; the vendor's placeholder
        # publisher would otherwise interleave zeros with it.
        launch_arguments={'use_joint_state_publisher': 'false'}.items(),
        condition=IfCondition(LaunchConfiguration('use_description')),
    )

    return LaunchDescription([use_description, odom_topic, bridge, description])
