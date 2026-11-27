"""The launch files build, Nav2's follower has the decided settings, Stage 3 is unchanged."""

import importlib.util
import os

from launch.actions import DeclareLaunchArgument, GroupAction
from launch_ros.actions import Node
import yaml

HERE = os.path.dirname(os.path.realpath(__file__))
STAGE3_PARAMS = os.path.join(HERE, '..', '..', 'apnvi_stage3', 'config', 'nav2_params.yaml')


def _load(name):
    path = os.path.join(HERE, '..', 'launch', name)
    spec = importlib.util.spec_from_file_location(name.replace('.', '_'), path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _follow_settings():
    with open(os.path.join(HERE, '..', 'config', 'nav2_follow.yaml')) as handle:
        return yaml.safe_load(handle)


def test_stage4_launch_builds_with_every_node():
    items = _load('stage4.launch.py').generate_launch_description().entities
    nodes = [i for i in items if isinstance(i, Node)]
    # drive link, hazard gate, vibration, free mode, health, guide, follower + manager
    assert len(nodes) == 8
    programs = {node.node_executable for node in nodes}
    assert {'free_mode_node', 'health_node', 'vibration_node'} <= programs


def test_free_mode_steering_is_off_by_default():
    items = _load('stage4.launch.py').generate_launch_description().entities
    steer = [i for i in items if isinstance(i, DeclareLaunchArgument) and i.name == 'free_steer']
    assert steer[0].default_value[0].text == 'false'
    node = [i for i in items if isinstance(i, Node) and i.node_executable == 'free_mode_node']
    assert node[0].condition is not None


def test_stage4_options():
    items = _load('stage4.launch.py').generate_launch_description().entities
    names = {i.name for i in items if isinstance(i, DeclareLaunchArgument)}
    for name in ('drive', 'dry_run', 'master_port', 'follow', 'rear_check', 'test_power',
                 'wheel_spacing_m', 'wheel_diameter_m', 'speed_at_cap_mps', 'max_speed_mps'):
        assert name in names


def test_rear_check_is_off_by_default():
    items = _load('stage4.launch.py').generate_launch_description().entities
    rear = [i for i in items if isinstance(i, DeclareLaunchArgument) and i.name == 'rear_check']
    assert rear[0].default_value[0].text == 'false'


def test_whole_device_launch_has_stages_and_a_stage4_switch():
    items = _load('apnvi.launch.py').generate_launch_description().entities
    assert len([i for i in items if isinstance(i, GroupAction)]) == 3
    names = {i.name for i in items if isinstance(i, DeclareLaunchArgument)}
    assert {'stage3', 'stage4'} <= names


def test_follower_is_regulated_pure_pursuit_at_the_decided_numbers():
    settings = _follow_settings()['controller_server']['ros__parameters']
    follow = settings['FollowPath']
    assert follow['plugin'].endswith('RegulatedPurePursuitController')
    assert follow['desired_linear_vel'] == 0.5
    assert follow['allow_reversing'] is False
    assert settings['general_goal_checker']['xy_goal_tolerance'] == 0.5
    assert settings['controller_frequency'] == 10.0


def test_lifecycle_manager_starts_only_the_follower():
    manager = _follow_settings()['lifecycle_manager_follow']['ros__parameters']
    assert manager['node_names'] == ['controller_server'] and manager['autostart'] is True


def test_stage3_planner_settings_are_untouched():
    with open(STAGE3_PARAMS) as handle:
        stage3 = yaml.safe_load(handle)
    assert 'planner_server' in stage3 and 'controller_server' not in stage3
