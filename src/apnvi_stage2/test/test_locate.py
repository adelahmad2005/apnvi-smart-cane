"""Tests for locate.py: a box, a depth picture and a position give a map point."""

import math

from apnvi_stage2.locate import (
    box_distance, box_on_depth, in_front, lens_from_fov, lens_from_info, locate, NO_DEPTH,
    NO_POSITION, on_map, same_spot, TOO_FAR)
import numpy as np
import pytest

COLOUR = lens_from_fov(1280, 720, 69.0)
DEPTH = lens_from_fov(848, 480, 87.0)


def depth_picture(wall_mm=4000):
    """Make a depth picture filled by a wall 4 m away."""
    return np.full((480, 848), wall_mm, dtype=np.uint16)


def put_thing(depth_mm, angle_deg, distance_mm, half_px=14):
    """Put a small thing at this angle (+ = left of centre) into the depth picture."""
    column = int(DEPTH.cx - DEPTH.fx * math.tan(math.radians(angle_deg)))
    depth_mm[240 - half_px:240 + half_px, column - half_px:column + half_px] = distance_mm
    return depth_mm


def colour_box(angle_deg, half_px=20):
    """Give the box Qwen would give for a thing at this angle, in colour pixels."""
    column = COLOUR.cx - COLOUR.fx * math.tan(math.radians(angle_deg))
    return (column - half_px, 360 - half_px, column + half_px, 360 + half_px)


def test_straight_ahead_thing_is_put_in_front_of_the_device():
    depth_mm = put_thing(depth_picture(), 0.0, 2000)
    found = locate(colour_box(0.0), (1280, 720), depth_mm, (1.0, 1.0, 0.0), COLOUR, DEPTH)
    assert found.found and found.distance_m == pytest.approx(2.0)
    assert found.x == pytest.approx(1.0 + 2.0 - 0.70) and found.y == pytest.approx(1.0, abs=0.02)
    assert (found.from_x, found.from_y) == (1.0, 1.0)


def test_thing_to_the_left_lands_left_on_the_map_whatever_way_the_device_faces():
    depth_mm = put_thing(depth_picture(), 20.0, 3000)
    box = colour_box(20.0)
    facing_x = locate(box, (1280, 720), depth_mm, (0.0, 0.0, 0.0), COLOUR, DEPTH)
    left = 3.0 * math.tan(math.radians(20.0))
    assert facing_x.y == pytest.approx(left, abs=0.05) and facing_x.x == pytest.approx(2.3)
    facing_y = locate(box, (1280, 720), depth_mm, (0.0, 0.0, math.pi / 2), COLOUR, DEPTH)
    assert facing_y.x == pytest.approx(-left, abs=0.05) and facing_y.y == pytest.approx(2.3)


def test_the_nearer_readings_in_the_box_win_over_the_wall_behind():
    depth_mm = depth_picture()
    depth_mm[230:250, 400:448] = 1500               # a thin thing in a much bigger box
    assert box_distance(depth_mm, (380, 215, 468, 265)) == pytest.approx(1.5)


def test_no_readings_or_too_far_or_no_position_give_a_reason():
    empty = np.zeros((480, 848), dtype=np.uint16)
    assert locate(colour_box(0.0), (1280, 720), empty, (0, 0, 0)).problem == NO_DEPTH
    assert locate(colour_box(0.0), (1280, 720), None, (0, 0, 0)).problem == NO_DEPTH
    far = locate(colour_box(0.0), (1280, 720), depth_picture(7000), (0, 0, 0))
    assert not far.found and far.problem == TOO_FAR and far.distance_m == pytest.approx(7.0)
    lost = locate(colour_box(0.0), (1280, 720), put_thing(depth_picture(), 0, 2000), None)
    assert not lost.found and lost.problem == NO_POSITION and lost.distance_m > 0


def test_box_moves_to_the_depth_picture_by_angle():
    left, _top, right, _bottom = box_on_depth((0, 0, 1280, 720), COLOUR, DEPTH)
    # the colour picture's 69 degrees sit inside the depth picture's 87 degrees
    assert 0 < left < right < 848
    assert (left + right) / 2 == pytest.approx(424.0)


def test_works_without_camera_info_and_with_a_resized_picture():
    depth_mm = put_thing(depth_picture(), 0.0, 2000)
    small_box = tuple(v / 2 for v in colour_box(0.0))
    found = locate(small_box, (640, 360), depth_mm, (0.0, 0.0, 0.0), COLOUR, None)
    assert found.found and found.x == pytest.approx(1.3, abs=0.03)
    assert locate(small_box, (640, 360), depth_mm, (0.0, 0.0, 0.0)).found


def test_lens_from_camera_info():
    lens = lens_from_info([900.0, 0, 640.0, 0, 905.0, 360.0, 0, 0, 1], 1280, 720)
    assert (lens.fx, lens.fy, lens.cx, lens.cy) == (900.0, 905.0, 640.0, 360.0)
    assert lens_from_info([0.0] * 9, 1280, 720) is None and lens_from_info([], 0, 0) is None


def test_in_front_and_on_map_and_same_spot():
    assert in_front((0, 0, 1280, 720), 2.0, COLOUR) == (2.0, pytest.approx(0.0))
    assert on_map(1.7, 0.0, (0.0, 0.0, math.pi)) == (pytest.approx(-1.0), pytest.approx(0.0))
    assert same_spot((1, 1, 0.0), (1.05, 1.0, 0.02)) and not same_spot((1, 1, 0), (1.5, 1, 0))
    assert not same_spot((1, 1, 0.0), (1, 1, 0.5)) and not same_spot(None, (0, 0, 0))
