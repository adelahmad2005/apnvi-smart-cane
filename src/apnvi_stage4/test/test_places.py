"""Goals from words: a map point, or a place saved in the running map's table."""

import os

from apnvi_stage3.map_names import add_place, empty_table, map_paths, save_table
from apnvi_stage4.places import clean_name, find_goal, map_point


def saved_map(root, name='ESB117'):
    table_path = map_paths(str(root), name)[1]
    os.makedirs(os.path.dirname(table_path), exist_ok=True)
    table = empty_table(name)
    add_place(table, 'Restroom', 4.0, -2.0, 0.0)
    table['signs'].append({'text': 'Exit', 'x': 1.0, 'y': 7.5})
    save_table(table_path, table)


def test_map_point():
    assert map_point('2.5 -1') == (2.5, -1.0)
    assert map_point('2.5, -1') == (2.5, -1.0)
    assert map_point('restroom') is None
    assert map_point('1 2 3') is None


def test_clean_name():
    assert clean_name('the Restroom.') == 'Restroom'
    assert clean_name('  a door ') == 'door'


def test_point_goal_needs_no_map(tmp_path):
    goal = find_goal('3 4', str(tmp_path), '')
    assert (goal.x, goal.y) == (3.0, 4.0)


def test_saved_place_and_sign(tmp_path):
    saved_map(tmp_path)
    goal = find_goal('the restroom', str(tmp_path), 'ESB117')
    assert (goal.x, goal.y, goal.name) == (4.0, -2.0, 'Restroom')
    sign = find_goal('exit', str(tmp_path), 'ESB117')
    assert (sign.x, sign.y) == (1.0, 7.5)


def test_unknown_place_or_no_map(tmp_path):
    saved_map(tmp_path)
    assert find_goal('kitchen', str(tmp_path), 'ESB117') is None
    assert find_goal('restroom', str(tmp_path), '') is None        # a new map: no places
    assert find_goal('restroom', str(tmp_path), 'Other') is None   # no table for this map


# ---- things and signs the device pinned (16 Nov) ----

def test_goal_for_a_thing_stops_in_front_of_it(tmp_path):
    from apnvi_stage3.map_names import add_pin, table_path
    from apnvi_stage4.places import approach_point
    path = table_path(str(tmp_path), '')                # a map with no name yet
    table = empty_table('')
    add_pin(table, 'water bottle', 5.0, 0.0, 1.0, 0.0, sure=True)
    add_pin(table, 'plant', 9.0, 9.0, 8.0, 9.0)        # seen once: not a goal yet
    save_table(path, table)
    goal = find_goal('the bottle', str(tmp_path), '')
    assert (round(goal.x, 2), goal.y) == (4.2, 0.0)
    assert goal.thing == (5.0, 0.0) and goal.name == 'the water bottle'
    assert find_goal('plant', str(tmp_path), '') is None
    assert approach_point((1.0, 0.0), (0.5, 0.0)) == (0.5, 0.0)    # seen from very close


def test_the_nearest_of_two_things_with_one_name(tmp_path):
    from apnvi_stage3.map_names import add_pin
    path = map_paths(str(tmp_path), 'lab')[1]
    table = empty_table('lab')
    add_pin(table, 'chair', 1.0, 0.0, 0.0, 0.0, sure=True)
    add_pin(table, 'chair', 9.0, 0.0, 7.0, 0.0, sure=True)
    save_table(path, table)
    assert find_goal('chair', str(tmp_path), 'lab', near=(8.0, 0.0)).thing == (9.0, 0.0)
    assert find_goal('chair', str(tmp_path), 'lab', near=(0.0, 0.0)).thing == (1.0, 0.0)


def test_a_thing_with_no_seen_from_spot_is_approached_from_the_device():
    from apnvi_stage4.places import thing_goal
    goal = thing_goal('the door', 4.0, 0.0, near=(0.0, 0.0))
    assert (round(goal.x, 2), goal.y, goal.thing) == (3.2, 0.0, (4.0, 0.0))
    assert thing_goal('the door', 4.0, 0.0).thing is None


def test_rooms_and_signs_are_named_naturally(tmp_path):
    from apnvi_stage3.map_names import add_pin
    path = map_paths(str(tmp_path), 'lab')[1]
    table = empty_table('lab')
    add_pin(table, 'ESB117', 2.0, 0.0, 0.0, 0.0, kind='sign', sure=True)
    add_pin(table, 'Fire exit', 0.0, 5.0, 0.0, 3.0, kind='sign', sure=True)
    save_table(path, table)
    assert find_goal('esb 117', str(tmp_path), 'lab').name == 'ESB117'
    assert find_goal('fire exit', str(tmp_path), 'lab').name == 'the Fire exit sign'
