"""Launch-level gates for yahboomcar_config, runnable with no robot and no ROS graph.

    python3 -m pytest yahboomcar_config/test/ -q      # needs ROS sourced

`bringup_corrected_launch.py` is a deliberate COPY of the vendor bringup composition
with one substitution (the EKF parameter file), because a launch file cannot override a
parameter hardcoded inside an included one. A copy drifts. These tests are the drift
guard: they read the vendor launch's source and fail if its sensor transforms, its IMU
remapping, or its EKF remapping stop matching ours.

They also parse both first-party launch files, which is the cheapest real gate on a
launch file there is -- `slam_launch.py`'s Jazzy lifecycle trap cost a whole session,
and a launch that does not even generate cannot be debugged on the floor.
"""
import importlib.util
import os
import re

import pytest

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

HERE = os.path.dirname(os.path.abspath(__file__))
LAUNCH_DIR = os.path.join(HERE, '..', 'launch')


def load_launch_module(path, name):
    spec = importlib.util.spec_from_file_location(name, path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def vendor_bringup_source():
    """The vendor launch as TEXT. Read from the installed share so the test checks what
    actually runs, not a source copy that may not be the installed one."""
    share = get_package_share_directory('yahboomcar_bringup')
    path = os.path.join(share, 'launch', 'yahboomcar_bringup_launch.py')
    if not os.path.exists(path):
        pytest.skip(f'vendor bringup launch not installed at {path}')
    with open(path) as f:
        return f.read()


def numbers_after(text, marker, count=6):
    """Pull the first `count` quoted numeric arguments following a marker."""
    idx = text.index(marker)
    window = text[idx:idx + 900]
    return re.findall(r"'(-?\d+\.?\d*)'", window)[:count]


# ------------------------------------------------------------ the launches parse
def test_bringup_corrected_generates_a_launch_description():
    mod = load_launch_module(
        os.path.join(LAUNCH_DIR, 'bringup_corrected_launch.py'), 'bc_launch')
    ld = mod.generate_launch_description()
    assert isinstance(ld, LaunchDescription)
    nodes = [e for e in ld.entities if isinstance(e, Node)]
    # imu filter, ekf, two static transforms
    assert len(nodes) == 4


def test_slam_launch_generates_and_offers_use_sim_time():
    """The replay harness passes use_sim_time:=true; an undeclared argument is a
    launch-time error, so this is the gate that keeps replays runnable."""
    mod = load_launch_module(os.path.join(LAUNCH_DIR, 'slam_launch.py'), 'slam_launch')
    ld = mod.generate_launch_description()
    assert isinstance(ld, LaunchDescription)
    declared = {a.name for a in ld.get_launch_arguments()}
    assert {'params_file', 'rviz', 'rviz_cfg', 'use_sim_time'} <= declared


def test_use_sim_time_defaults_to_false_everywhere():
    """Live sessions must never run on sim time -- that is what lets the simulator be
    swapped for the car with no configuration change."""
    for fname, modname in (('slam_launch.py', 'sl2'),
                           ('bringup_corrected_launch.py', 'bc2')):
        mod = load_launch_module(os.path.join(LAUNCH_DIR, fname), modname)
        ld = mod.generate_launch_description()
        arg = next(a for a in ld.get_launch_arguments() if a.name == 'use_sim_time')
        assert arg.default_value[0].text == 'false', fname


def test_corrected_launch_defaults_to_the_corrected_ekf():
    mod = load_launch_module(
        os.path.join(LAUNCH_DIR, 'bringup_corrected_launch.py'), 'bc3')
    ld = mod.generate_launch_description()
    arg = next(a for a in ld.get_launch_arguments() if a.name == 'ekf_config')
    assert arg.default_value[0].text.endswith('ekf_corrected.yaml')


def test_the_ab_arm_is_reachable_by_argument():
    """Phase 4 needs the vendor config as the A/B baseline, so ekf_config must be an
    argument rather than a constant."""
    mod = load_launch_module(
        os.path.join(LAUNCH_DIR, 'bringup_corrected_launch.py'), 'bc4')
    ld = mod.generate_launch_description()
    assert any(a.name == 'ekf_config' for a in ld.get_launch_arguments())


# ------------------------------------------------------- drift against the vendor
def test_static_transforms_still_match_the_vendor_launch():
    """If the vendor moves a sensor, this copy must fail rather than silently mapping
    with the old geometry."""
    mod = load_launch_module(
        os.path.join(LAUNCH_DIR, 'bringup_corrected_launch.py'), 'bc5')
    src = vendor_bringup_source()
    vendor_imu = numbers_after(src, "name='base_link_to_base_imu'")
    vendor_laser = numbers_after(src, "name='base_link_to_laser'")
    assert vendor_imu == mod.BASE_TO_IMU[:6], 'base_link->imu_frame drifted'
    assert vendor_laser == mod.BASE_TO_LASER[:6], 'base_link->laser_frame drifted'


def test_frame_names_still_match_the_vendor_launch():
    mod = load_launch_module(
        os.path.join(LAUNCH_DIR, 'bringup_corrected_launch.py'), 'bc6')
    src = vendor_bringup_source()
    assert "'base_link', 'imu_frame'" in src
    assert "'base_link', 'laser_frame'" in src
    assert mod.BASE_TO_IMU[-2:] == ['base_link', 'imu_frame']
    assert mod.BASE_TO_LASER[-2:] == ['base_link', 'laser_frame']


def test_the_vendor_still_remaps_the_same_topics():
    """The IMU input remap and the /odom output remap are load-bearing: without them
    imu_filter listens to the wrong topic and the EKF publishes odometry/filtered,
    which nothing downstream subscribes to."""
    src = vendor_bringup_source()
    assert "('imu/data_raw', '/imu')" in src
    assert "('odometry/filtered', '/odom')" in src


def test_the_vendor_launch_still_hardcodes_its_ekf_config():
    """The whole reason this copy exists. If the vendor ever parameterises its EKF
    config, this launch can become a thin include and this test should be the thing
    that tells us."""
    src = vendor_bringup_source()
    assert "ekf_config = os.path.join(bringup_share, 'param', 'ekf.yaml')" in src
