"""robot1 bringup with the CORRECTED EKF. The composition physical SLAM runs on.

    ros2 launch yahboomcar_config bringup_corrected_launch.py
    ros2 launch yahboomcar_config bringup_corrected_launch.py ekf_config:=/abs/other.yaml

WHY THIS EXISTS
---------------
`yahboomcar_bringup_launch.py` hardcodes the VENDOR `ekf.yaml`, and that configuration
is measured wrong in the direction that matters most for mapping: on the stand with the
body held still it reported **2239 mm of travel against a true 0**, tracking commanded
wheel distance to 0.5%, because it fuses wheel pose AND wheel twist and nothing in it
observes the world. `yahboomcar_config/param/ekf_corrected.yaml` (twist only, IMU rate
only) reported **6 mm** on the same data — see `tools/ekf_ab_test.py` and
docs/sensor-fusion-research.md.

Feeding SLAM an odometry prior that confident and that wrong is not a tuning question.
A scan matcher trusts its prior; one that claims 2.2 m of motion the robot did not make
will drag every match with it. So physical SLAM runs use this launch, and the vendor
launch stays untouched so the course PDFs remain correct (R-05).

**THE CORRECTED EKF IS STILL UNVALIDATED IN REAL MOTION** [measured on a stand only,
2026-08-06]. That is precisely what a physical mapping run tests, which is why the
config is a launch argument rather than a constant: an A/B against the vendor config on
the SAME recorded bag is a Phase-4 row, not an assumption baked in here.

MIRRORS THE VENDOR LAUNCH, DELIBERATELY
---------------------------------------
Everything except the EKF parameter file is the same composition the vendor bringup
starts: imu_filter_madgwick, the two static transforms, and the description. It is a
copy rather than an include because a launch file cannot override a parameter hardcoded
inside an included one — and a copy drifts, so
`yahboomcar_config/test/test_bringup_corrected.py` asserts node-for-node agreement with
the vendor launch and fails if the vendor side changes.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# Static transforms, copied verbatim from the vendor launch so the two agree rather
# than introducing a third opinion about where the sensors are.
BASE_TO_IMU = ['-0.002999', '-0.0030001', '0.031701', '0', '0', '0',
               'base_link', 'imu_frame']
BASE_TO_LASER = ['-0.0046412', '0', '0.094079', '0', '0', '0',
                 'base_link', 'laser_frame']


def generate_launch_description():
    bringup_share = get_package_share_directory('yahboomcar_bringup')
    config_share = get_package_share_directory('yahboomcar_config')
    imu_filter_config = os.path.join(bringup_share, 'param', 'imu_filter_param.yaml')
    corrected_ekf = os.path.join(config_share, 'param', 'ekf_corrected.yaml')

    args = [
        DeclareLaunchArgument(
            'ekf_config', default_value=corrected_ekf,
            description='EKF parameter file. Point it at the vendor ekf.yaml to run '
                        'the A/B arm.'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
    ]
    use_sim_time = LaunchConfiguration('use_sim_time')

    imu_filter_node = Node(
        package='imu_filter_madgwick',
        executable='imu_filter_madgwick_node',
        name='imu_filter_madgwick',
        output='screen',
        parameters=[imu_filter_config, {'use_sim_time': use_sim_time}],
        remappings=[('imu/data_raw', '/imu')],
    )

    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[LaunchConfiguration('ekf_config'), {'use_sim_time': use_sim_time}],
        remappings=[('odometry/filtered', '/odom')],
    )

    base_link_to_imu_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_to_base_imu',
        arguments=BASE_TO_IMU,
    )

    # Without this, bringup leaves the robot with no transform to its own lidar:
    # /scan is stamped `laser_frame`, which appears nowhere in the URDF.
    base_link_to_laser_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_to_laser',
        arguments=BASE_TO_LASER,
    )

    description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('yahboomcar_description'),
                         'launch'),
            '/description_launch.py',
        ])
    )

    return LaunchDescription(args + [
        LogInfo(msg=['bringup with CORRECTED EKF: ',
                     LaunchConfiguration('ekf_config')]),
        LogInfo(msg='The vendor ekf.yaml reported 2239 mm of phantom travel on a '
                    'stand where this one reported 6 mm [measured]. Still unvalidated '
                    'in real motion.'),
        imu_filter_node,
        ekf_node,
        base_link_to_imu_tf_node,
        base_link_to_laser_tf_node,
        description_launch,
    ])
