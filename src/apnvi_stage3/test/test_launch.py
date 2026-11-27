"""The launch file builds, and the frames and the map settings are what Stage 3 expects."""

import importlib.util
import os

from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

LAUNCH = os.path.join(os.path.dirname(__file__), '..', 'launch', 'stage3.launch.py')


def _load():
    spec = importlib.util.spec_from_file_location('stage3_launch', LAUNCH)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_launch_builds_with_every_node():
    items = _load().generate_launch_description().entities
    nodes = [i for i in items if isinstance(i, Node)]
    assert len(nodes) == 12     # two frames, LiDAR, scan filter, odometry, camera join, two
    #                             map nodes (one runs), map manager, map picture, Nav2 (two)


def test_sensor_positions_are_options():
    items = _load().generate_launch_description().entities
    names = {i.name for i in items if isinstance(i, DeclareLaunchArgument)}
    for name in ('lidar_height', 'lidar_x', 'lidar_y', 'lidar_yaw', 'camera_height',
                 'camera_x', 'camera_y', 'rate', 'map_view', 'nav2', 'camera_map'):
        assert name in names


def test_settings_are_text_and_flat():
    icp = _load().ICP
    assert all(isinstance(v, str) for v in icp.values())
    assert icp['Reg/Force3DoF'] == 'true' and icp['Reg/Strategy'] == '1'


def test_stage3_runs_at_lower_priority():
    assert _load().LOWER_PRIORITY.startswith('nice -n')


def test_nav2_only_plans():
    """The Nav2 settings have a planner and costmap layers, and no controller or behaviours."""
    import yaml
    path = os.path.join(os.path.dirname(__file__), '..', 'config', 'nav2_params.yaml')
    with open(path) as handle:
        settings = yaml.safe_load(handle)
    assert 'planner_server' in settings and 'global_costmap' in settings
    assert not {'controller_server', 'bt_navigator', 'behavior_server'} & set(settings)
    costmap = settings['global_costmap']['global_costmap']['ros__parameters']
    assert costmap['plugins'] == [
        'static_layer', 'obstacle_layer', 'camera_layer', 'inflation_layer']
    assert costmap['update_frequency'] == 1.0


def test_load_test_launch_builds():
    path = os.path.join(os.path.dirname(__file__), '..', 'launch', 'loadtest.launch.py')
    spec = importlib.util.spec_from_file_location('loadtest_launch', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    assert len(module.generate_launch_description().entities) >= 7
