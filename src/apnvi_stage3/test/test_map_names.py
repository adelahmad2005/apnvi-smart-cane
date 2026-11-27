"""Tests for map_names.py: names, saved files and the table. No ROS needed."""

import os

import pytest

from apnvi_stage3.map_names import (
    add_place, copy_file, empty_table, find_place, load_table, map_paths, save_table,
    saved_maps, slug, work_path)


def test_spoken_names_become_the_same_file_name():
    assert slug('Engineering  building') == slug('engineering building') == 'engineering_building'
    assert slug('ESB117!') == 'esb117'


def test_a_map_needs_a_name(tmp_path):
    with pytest.raises(ValueError):
        map_paths(str(tmp_path), '  ')


def test_saved_map_has_a_database_and_a_table_file(tmp_path):
    database, table = map_paths(str(tmp_path), 'ESB117')
    assert database.endswith(os.path.join('named', 'esb117.db'))
    assert table.endswith(os.path.join('named', 'esb117.json'))
    assert work_path(str(tmp_path), 'ESB117').endswith(os.path.join('work', 'esb117_work.db'))


def test_copy_leaves_no_half_file_and_the_list_finds_it(tmp_path):
    source = tmp_path / 'live.db'
    source.write_bytes(b'x' * 1000)
    database, _ = map_paths(str(tmp_path), 'ESB117')
    copy_file(str(source), database)
    assert open(database, 'rb').read() == b'x' * 1000
    assert not os.path.exists(database + '.partial')
    assert saved_maps(str(tmp_path)) == ['esb117']


def test_table_round_trip_and_default(tmp_path):
    path = str(tmp_path / 'named' / 'esb117.json')
    assert load_table(path, 'ESB117') == empty_table('ESB117')
    table = empty_table('ESB117')
    add_place(table, 'My office', 1.5, -2.0, 0.5, when=100.0)
    save_table(path, table)
    assert load_table(path)['places']['my_office']['x'] == 1.5


def test_saving_the_same_place_again_moves_it():
    table = empty_table('m')
    add_place(table, 'My office', 1.0, 1.0, 0.0)
    add_place(table, 'my  office', 3.0, 4.0, 0.0)
    assert len(table['places']) == 1 and table['places']['my_office']['x'] == 3.0


def test_find_place_by_name_part_or_sign():
    table = empty_table('m')
    add_place(table, 'My office', 1.0, 1.0, 0.0)
    table['signs'].append({'text': 'Restroom', 'x': 5.0, 'y': 2.0})
    assert find_place(table, 'my office')['x'] == 1.0
    assert find_place(table, 'office')['x'] == 1.0
    assert find_place(table, 'the restroom')['x'] == 5.0
    assert find_place(table, 'kitchen') is None


def test_a_saved_place_wins_over_a_pinned_thing_of_the_same_name():
    from apnvi_stage3.map_names import add_pin
    table = empty_table('m')
    add_pin(table, 'desk', 1.5, 1.3, kind='thing')
    add_pin(table, 'desk', 1.5, 1.3, kind='thing')
    add_place(table, 'my desk', -4.4, -2.5, 0.0)
    assert find_place(table, 'desk')['x'] == -4.4
    assert find_place(table, 'my desk')['x'] == -4.4


def test_wall_picture_is_read_from_a_map_file(tmp_path):
    import sqlite3
    import zlib

    import numpy as np
    grid = np.full((30, 20), -1, np.int8)
    grid[5:25, 4:16] = 0
    grid[5, 4:16] = 100
    path = str(tmp_path / 'm.db')
    connection = sqlite3.connect(path)
    connection.execute('create table Admin (opt_map blob, opt_map_x_min float, '
                       'opt_map_y_min float, opt_map_resolution float)')
    connection.execute('insert into Admin values (?, ?, ?, ?)',
                       (zlib.compress(grid.tobytes()), -1.5, -2.5, 0.05))
    connection.commit()
    connection.close()
    from apnvi_stage3.map_names import saved_grid
    cells, resolution, origin = saved_grid(path)
    assert cells.shape == (30, 20) and resolution == 0.05 and origin == (-1.5, -2.5)
    assert saved_grid(str(tmp_path / 'missing.db')) is None


# ---- pins: things and signs the device labelled or the user asked for (16 Nov) ----

def test_a_labelled_thing_shows_only_after_two_looks():
    from apnvi_stage3.map_names import add_pin, all_pins, find_place
    table = empty_table('lab')
    add_pin(table, 'water bottle', 2.0, 1.0, 0.5, 1.0)
    assert find_place(table, 'water bottle') is None
    assert all_pins(table) == []
    pin = add_pin(table, 'bottle', 2.2, 1.0, 0.6, 1.0)
    assert pin['seen'] == 2 and len(table['objects']) == 1
    assert pin['x'] == pytest.approx(2.1)
    assert find_place(table, 'the water bottle'.replace('the ', ''))['x'] == pytest.approx(2.1)
    assert all_pins(table)[0]['name'] == 'water bottle'


def test_a_thing_the_user_asked_for_is_sure_at_once():
    from apnvi_stage3.map_names import add_pin, find_place
    table = empty_table('lab')
    add_pin(table, 'printer', 4.0, 0.0, 3.0, 0.0, sure=True)
    found = find_place(table, 'printer')
    assert found['x'] == 4.0 and found['from_x'] == 3.0


def test_the_same_name_far_away_is_another_pin_and_the_nearest_wins():
    from apnvi_stage3.map_names import add_pin, find_place
    table = empty_table('lab')
    add_pin(table, 'chair', 1.0, 0.0, sure=True)
    add_pin(table, 'chair', 6.0, 0.0, sure=True)
    assert len(table['objects']) == 2
    assert find_place(table, 'chair', near=(5.0, 0.0))['x'] == 6.0
    assert find_place(table, 'chair', near=(0.0, 0.0))['x'] == 1.0


def test_two_names_for_one_thing_are_one_pin():
    from apnvi_stage3.map_names import add_pin, same_name
    table = empty_table('lab')
    add_pin(table, 'bin', 1.0, 1.0)
    add_pin(table, 'trash can', 1.3, 1.1)
    assert len(table['objects']) == 1 and table['objects'][0]['seen'] == 2
    assert same_name('Water bottle', 'bottle') and not same_name('chair', 'door')


def test_signs_keep_their_words_and_rooms_are_marked():
    from apnvi_stage3.map_names import add_pin, find_place, is_room
    table = empty_table('lab')
    add_pin(table, 'ESB117', 0.0, 3.0, 0.0, 2.0, kind='sign', sure=True)
    add_pin(table, 'Fire exit', 5.0, 3.0, kind='sign', sure=True)
    assert table['signs'][0]['kind'] == 'room' and table['signs'][1]['kind'] == 'sign'
    assert find_place(table, 'esb 117')['y'] == 3.0
    assert is_room('Lab 2') and is_room('ESB 2090') and not is_room('Push')


def test_remove_rename_and_merge():
    from apnvi_stage3.map_names import (add_pin, find_place, merge_tables, remove_pin,
                                        rename_pin)
    table = empty_table('')
    add_place(table, 'my desk', 1.0, 1.0, 0.0)
    add_pin(table, 'bin', 2.0, 2.0, sure=True)
    add_pin(table, 'plant', 3.0, 3.0)
    add_pin(table, 'plant', 3.0, 3.0)
    assert rename_pin(table, 'bin', 'recycling bin') == 1
    assert find_place(table, 'recycling bin')['x'] == 2.0
    saved = merge_tables(empty_table('lab'), table)
    assert find_place(saved, 'my desk') and find_place(saved, 'plant')['seen'] == 2
    assert remove_pin(saved, 'plant') == 1 and find_place(saved, 'plant') is None
    assert remove_pin(saved, 'my desk') == 1


def test_a_clicked_pin_is_the_only_one_touched():
    # 23 Nov (Adel): one "Remove" on the console took all 60 tables off outside_2
    from apnvi_stage3.map_names import add_pin, all_pins, remove_pin, rename_pin
    table = empty_table('')
    for x in (0.0, 3.0, 6.0):
        add_pin(table, 'table', x, 0.0, sure=True)
    add_place(table, 'my desk', 9.0, 0.0, 0.0)
    assert remove_pin(table, 'table', near=(3.1, 0.1)) == 1
    assert sorted(p['x'] for p in all_pins(table) if p['name'] == 'table') == [0.0, 6.0]
    assert remove_pin(table, 'table', near=(3.0, 0.0)) == 0       # nothing that close now
    assert rename_pin(table, 'table', 'long table', near=(6.2, 0.0)) == 1
    names = sorted(p['name'] for p in all_pins(table))
    assert names == ['long table', 'my desk', 'table']
    assert remove_pin(table, 'my desk', near=(5.0, 0.0)) == 0      # too far from the desk
    assert remove_pin(table, 'my desk', near=(9.0, 0.3)) == 1
    assert remove_pin(table, 'table') == 1                         # by voice: all of that name


def test_table_path_for_a_map_with_no_name(tmp_path):
    from apnvi_stage3.map_names import table_path
    assert table_path(str(tmp_path), '').endswith(os.path.join('work', 'unsaved.json'))
    assert table_path(str(tmp_path), 'ESB 117').endswith(os.path.join('named', 'esb_117.json'))


def test_open_floor_boxes_are_wiped_only_in_the_route_picture():
    import numpy as np
    from apnvi_stage3.map_names import with_open_floor
    cells = np.full((20, 20), 100, np.int8)             # 1 m x 1 m, all wall
    grid = (cells, 0.05, (0.0, 0.0))
    table = {'open_floor': [[0.2, 0.5, 0.2, 0.5]]}
    route, resolution, origin = with_open_floor(grid, table)
    assert route[7, 7] == 0 and route[15, 15] == 100 and cells[7, 7] == 100
    assert with_open_floor(grid, {}) is grid


def test_movable_things_last_one_session(tmp_path):
    from apnvi_stage3.map_names import add_pin, empty_table, forget_session, load_table, save_table
    path = str(tmp_path / 'room.json')
    table = empty_table('room')
    add_pin(table, 'whiteboard', 1.0, 2.0)
    add_pin(table, 'chair', 3.0, 1.0, movable=True)
    save_table(path, table)
    names = [pin['name'] for pin in load_table(path, 'room')['objects']]
    assert names == ['whiteboard', 'chair']             # this session: both
    forget_session(path)                                # the map is loaded again
    assert [pin['name'] for pin in load_table(path, 'room')['objects']] == ['whiteboard']


def test_a_pin_moves_with_its_path_point():
    import math
    from apnvi_stage3.map_names import add_pin, empty_table, follow_path, tie_pin
    table = empty_table('outside')
    pin = add_pin(table, 'vending machine', 2.0, 1.0, from_x=1.0, from_y=1.0)
    tie_pin(pin, 7, (1.0, 1.0, 0.0))                    # seen from path point 7, 1 m ahead
    # RTAB-Map straightens the map: point 7 is now at (5, 5), turned a quarter left
    assert follow_path(table, {7: (5.0, 5.0, math.pi / 2)}) == 1
    assert abs(pin['x'] - 5.0) < 1e-6 and abs(pin['y'] - 6.0) < 1e-6
    assert abs(pin['from_x'] - 5.0) < 1e-6 and abs(pin['from_y'] - 5.0) < 1e-6
    assert follow_path(table, {7: (5.0, 5.0, math.pi / 2)}) == 0     # nothing new: no rewrite


def test_untied_pins_stay():
    from apnvi_stage3.map_names import add_pin, empty_table, follow_path
    table = empty_table('room')
    add_pin(table, 'door', 1.0, 1.0)
    assert follow_path(table, {1: (9.0, 9.0, 0.0)}) == 0
