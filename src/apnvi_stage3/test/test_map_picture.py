"""Tests for map_picture.py: the map picture and where things land in it. No ROS needed."""

import math

import numpy as np

from apnvi_stage3.map_picture import (
    DEVICE, draw_map, FREE, LEGEND_H, SIZE, to_map, to_pixel, UNKNOWN, view_box, WALL)

RES = 0.05


def room(width_m=4.0, height_m=2.0):
    """Make a map: free floor with a wall along one side, inside a bigger unknown area."""
    cols, rows = round(width_m / RES), round(height_m / RES)
    grid = np.full((rows + 40, cols + 40), -1, np.int8)
    grid[20:20 + rows, 20:20 + cols] = 0
    grid[20, 20:20 + cols] = 100                 # the wall at the lowest y
    return grid


def pixel(picture, x, y, box):
    column, row = to_pixel(x, y, box)
    return tuple(picture[int(row), int(column)])


def test_map_is_square_with_a_legend_under_it():
    grid, origin = room(), (-1.0, -1.0)           # the seen part starts at (0, 0)
    picture, box, _ = draw_map(grid, RES, origin)
    assert picture.shape == (SIZE + LEGEND_H, SIZE, 3)
    assert pixel(picture, 2.5, 1.5, box) == FREE
    assert pixel(picture, 2.5, 0.02, box) == WALL
    assert pixel(picture, 2.5, 2.5, box) == UNKNOWN   # beyond the seen floor


def test_legend_shows_each_colour_the_map_uses():
    legend = draw_map(room(), RES, (-1.0, -1.0))[0][SIZE:]
    colours = {tuple(pixel_) for row in legend for pixel_ in row}
    for colour in (FREE, WALL, UNKNOWN, DEVICE):
        assert colour in colours


def test_up_on_the_map_is_up_in_the_picture():
    _, box, _ = draw_map(room(), RES, (-1.0, -1.0))
    assert to_pixel(2.0, 2.0, box)[1] < to_pixel(2.0, 0.0, box)[1]      # larger y: higher up
    assert to_pixel(3.0, 1.0, box)[0] > to_pixel(1.0, 1.0, box)[0]      # larger x: further right


def test_pixel_and_map_point_convert_both_ways():
    _, box, _ = draw_map(room(), RES, (-1.0, -1.0))
    column, row = to_pixel(1.7, 0.4, box)
    x, y = to_map(column, row, box)
    assert math.isclose(x, 1.7) and math.isclose(y, 0.4)


def test_view_never_zooms_in_closer_than_six_metres():
    box = view_box(np.full((10, 10), -1, np.int8), RES, (0.0, 0.0), None)
    assert box[2] >= 6.0


def test_view_includes_the_device_even_outside_the_seen_map():
    grid, origin = room(), (-1.0, -1.0)
    box = view_box(grid, RES, origin, (10.0, 1.0, 0.0))
    column, row = to_pixel(10.0, 1.0, box)
    assert 0 <= column <= SIZE and 0 <= row <= SIZE


def test_device_is_an_orange_dot():
    grid, origin = room(), (-1.0, -1.0)
    picture, box, _ = draw_map(grid, RES, origin, (2.0, 1.0, 0.0))
    assert pixel(picture, 2.0, 1.0, box) == DEVICE


def test_arrow_points_the_way_the_device_faces():
    grid, origin = room(), (-1.0, -1.0)
    column, row = to_pixel(2.0, 1.0, view_box(grid, RES, origin, (2.0, 1.0, 0.0)))
    for yaw, dx, dy in ((0.0, 1, 0), (math.pi / 2, 0, -1)):      # right; up (smaller row)
        picture, _, _ = draw_map(grid, RES, origin, (2.0, 1.0, yaw))
        ahead = tuple(picture[round(row + 20 * dy), round(column + 20 * dx)])
        behind = tuple(picture[round(row - 20 * dy), round(column - 20 * dx)])
        assert ahead == DEVICE and behind != DEVICE


def test_empty_map_without_a_position_still_draws():
    picture, _, _ = draw_map(np.full((20, 20), -1, np.int8), RES, (0.0, 0.0))
    assert picture.shape == (SIZE + LEGEND_H, SIZE, 3) and tuple(picture[10, 10]) == UNKNOWN


def test_grid_line_every_metre_and_scale_bar():
    from apnvi_stage3.map_picture import GRID_LINE
    picture, box, _ = draw_map(room(), RES, (-1.0, -1.0))
    column, row = to_pixel(2.0, 1.5, box)            # on the x = 2 m line, in free floor
    assert tuple(picture[int(row), round(column)]) == GRID_LINE
    assert tuple(picture[int(row), round(column) + 5]) == FREE   # just off the line
    bottom = SIZE + LEGEND_H - 8                     # the white bar: in the legend (19 Nov)
    assert (picture[bottom - 6:bottom, 8:16] == 255).all()


def test_pins_are_drawn_with_their_colour():
    from apnvi_stage3.map_picture import PIN_COLOURS
    grid, origin = room(), (-1.0, -1.0)
    plain, box, _ = draw_map(grid, RES, origin)
    pins = [{'name': 'door', 'kind': 'thing', 'x': 1.0, 'y': 1.0},
            {'name': 'far away', 'kind': 'place', 'x': 500.0, 'y': 500.0}]
    picture, _, _ = draw_map(grid, RES, origin, pins=pins)
    column, row = (round(v) for v in to_pixel(1.0, 1.0, box))
    assert tuple(picture[row, column]) == PIN_COLOURS['thing']
    assert (picture[:SIZE] != plain[:SIZE]).any()


def test_pins_are_only_dots_with_no_words_and_no_list():
    # 17 Nov (Adel): no names or numbers on the map and no list under it; tap a dot instead
    from apnvi_stage3.map_picture import PIN_PX
    grid, origin = room(), (-1.0, -1.0)
    plain, box, _ = draw_map(grid, RES, origin)
    pins = [{'name': n, 'kind': 'thing', 'x': 1.0 + 0.3 * k, 'y': 1.0}
            for k, n in enumerate(('desk', 'screen', 'chair', 'bin'))]
    picture, _, shown = draw_map(grid, RES, origin, pins=pins)
    assert picture.shape == (SIZE + LEGEND_H, SIZE, 3) and len(shown) == 4
    rows, columns = np.nonzero((picture != plain).any(axis=2))
    assert rows.size
    for row, column in zip(rows, columns):          # every changed pixel is part of a dot
        assert any(abs(row - p['row']) <= PIN_PX + 2 and abs(column - p['col']) <= PIN_PX + 2
                   for p in shown)


def test_the_same_name_close_together_is_drawn_once():
    grid, origin = room(), (-1.0, -1.0)
    chair = {'name': 'chair', 'kind': 'thing', 'x': 1.0, 'y': 1.0}
    one, _, _ = draw_map(grid, RES, origin, pins=[chair])
    two, _, _ = draw_map(grid, RES, origin, pins=[chair, dict(chair, x=1.02)])
    assert (one == two).all()


def test_pins_come_back_with_their_place():
    # 17 Nov (Adel): the page needs each dot's place to tell a tap on it
    grid, origin = room(), (-1.0, -1.0)
    pins = [{'name': 'desk', 'kind': 'thing', 'x': 1.0, 'y': 1.0},
            {'name': 'far away', 'kind': 'place', 'x': 500.0, 'y': 500.0},
            {'name': 'ESB117', 'kind': 'room', 'x': 2.0, 'y': 0.5}]
    _, box, shown = draw_map(grid, RES, origin, pins=pins)
    assert [(p['name'], p['kind']) for p in shown] == [('desk', 'thing'), ('ESB117', 'room')]
    column, row = to_pixel(1.0, 1.0, box)
    assert (shown[0]['col'], shown[0]['row']) == (round(column), round(row))
    assert (shown[0]['x'], shown[0]['y']) == (1.0, 1.0)


def test_zoom_box_is_closer_and_stays_inside_the_whole_map():
    from apnvi_stage3.map_picture import MIN_ZOOM_SPAN_M, zoom_box
    whole = (0.0, 10.0, 10.0)
    assert zoom_box(whole) == whole and zoom_box(whole, 0.5) == whole
    assert zoom_box(whole, 2, (5.0, 5.0)) == (2.5, 7.5, 5.0)
    assert zoom_box(whole, 2) == (2.5, 7.5, 5.0)                 # no centre: the middle
    assert zoom_box(whole, 2, (-5.0, 20.0)) == (-2.5, 12.5, 5.0)  # far out: corner in middle
    assert zoom_box(whole, 1, (7.0, 5.0)) == (2.0, 10.0, 10.0)    # dragged at the whole map
    assert zoom_box(whole, 100, (5.0, 5.0))[2] == MIN_ZOOM_SPAN_M
    assert zoom_box((0.0, 1.0, 1.5), 4) == (0.0, 1.0, 1.5)       # already small: no change


def test_zoomed_map_spreads_close_pins_apart():
    grid, origin = room(), (-1.0, -1.0)
    pins = [{'name': 'desk', 'kind': 'thing', 'x': 1.0, 'y': 1.0},
            {'name': 'screen', 'kind': 'thing', 'x': 1.3, 'y': 1.0}]
    _, whole, far = draw_map(grid, RES, origin, pins=pins)
    picture, box, near = draw_map(grid, RES, origin, pins=pins, zoom=2, centre=(1.15, 1.0))
    assert picture.shape[1] == SIZE and box[2] < whole[2]
    assert near[1]['col'] - near[0]['col'] > far[1]['col'] - far[0]['col']
