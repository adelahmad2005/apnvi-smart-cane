"""Free mode steering: the device only turns, and only while the user pushes it."""

import math

from apnvi_stage4.free_mode import (
    clear_heading, FreeSettings, lane_nearest, PushMeter, quaternion_matrix, side_room,
    Steering, sweep_points)
import numpy as np
import pytest

S = FreeSettings()


def box(x, y, size=0.3, step=0.05):
    """Points on a small box centred at (x, y) in the device's frame."""
    line = np.arange(-size / 2, size / 2 + 1e-9, step)
    return np.array([(x + a, y + b) for a in line for b in line])


def wall(y, x_from=-1.0, x_to=3.0, step=0.05):
    """Points along a wall beside the device, at this y."""
    return np.array([(x, y) for x in np.arange(x_from, x_to, step)])


def pushed(steering, points, speed=0.4, yaw=0.0, start=0.0, seconds=1.0):
    """Push for a while and give the last turn rate."""
    turn, t = 0.0, start
    while t <= start + seconds + 1e-9:
        turn = steering.step(t, speed, yaw, points)
        t += 0.1
    return turn


# ---- what is seen ----

def test_lane_sees_only_what_is_in_front_and_within_its_width():
    assert lane_nearest(box(1.0, 0.0), 0.0, S) == pytest.approx(1.0 - 0.15 - S.front_m)
    assert lane_nearest(box(1.0, 1.0), 0.0, S) is None          # beside the lane
    assert lane_nearest(box(-1.0, 0.0), 0.0, S) is None         # the user, behind
    assert lane_nearest(box(2.5, 0.0), 0.0, S) is None          # beyond the lane
    assert lane_nearest(np.zeros((0, 2)), 0.0, S) is None
    # a box ahead and to the left is in the lane of a heading turned left
    assert lane_nearest(box(1.0, 1.0), math.radians(45), S) is not None


def test_sweep_from_a_sensor_facing_forward_and_from_the_lidar_upside_down_facing_back():
    ahead = sweep_points([2.0, float('inf'), float('nan'), 9.0], -0.1, 0.1, np.eye(3),
                         (-0.7, 0.0, 0.86))
    assert len(ahead) == 1 and ahead[0] == pytest.approx((2.0 * math.cos(-0.1) - 0.7,
                                                          2.0 * math.sin(-0.1)))
    # the LiDAR: turned half round (facing back) and rolled over (upside down), 0.7 m behind
    yaw_pi = quaternion_matrix(0, 0, 1, 0)
    roll_pi = quaternion_matrix(1, 0, 0, 0)
    turn = yaw_pi @ roll_pi
    # a beam at its own +90 deg: upside down that points to its right, and facing back its
    # right is the device's left... so it lands on the device's left (+y)
    left = sweep_points([1.0], math.pi / 2, 0.0, turn, (-0.7, 0.0, 0.77))
    assert left[0] == pytest.approx((-0.7, 1.0), abs=1e-6)
    back = sweep_points([1.0], 0.0, 0.0, turn, (-0.7, 0.0, 0.77))
    assert back[0] == pytest.approx((-1.7, 0.0), abs=1e-6)


def test_clear_heading_picks_the_smallest_turn_and_the_roomier_side():
    assert clear_heading(box(1.0, 0.2), S) < 0                  # box a bit left: go right
    assert clear_heading(box(1.0, -0.2), S) > 0
    both = np.vstack([box(1.0, 0.0), wall(-0.9)])               # wall on the right: go left
    assert clear_heading(both, S) > 0
    assert clear_heading(box(1.0, 0.2), S, keep_side=1) > 0     # told to stay on the left
    boxed_in = np.array([(1.0 * math.cos(a), 1.0 * math.sin(a))
                         for a in np.arange(-1.6, 1.6, 0.03)])
    assert clear_heading(boxed_in, S) is None


def test_side_room():
    left, right = side_room(np.vstack([wall(0.7), wall(-1.0)]), S)
    assert left == pytest.approx(0.7) and right == pytest.approx(1.0)
    assert side_room(wall(0.7), S) == (pytest.approx(0.7), None)


# ---- the decision ----

def test_never_turns_when_nobody_pushes():
    steering = Steering()
    for t in range(50):
        assert steering.step(t * 0.1, 0.0, 0.0, box(1.0, 0.0)) == 0.0
    assert steering.why == 'not_pushed'


def test_pushing_at_a_box_turns_away_from_it_and_keeps_to_that_side():
    steering = Steering()
    assert pushed(steering, box(1.0, 0.2)) == -S.turn_rps and steering.why == 'round'
    # the box now looks as if it were on the other side: it still goes round on the right
    assert steering.step(1.2, 0.4, 0.0, box(1.0, -0.2)) == -S.turn_rps


def test_no_turn_until_pushed_for_a_moment():
    steering = Steering()
    assert steering.step(0.0, 0.4, 0.0, box(1.0, 0.2)) == 0.0
    assert steering.step(0.2, 0.4, 0.0, box(1.0, 0.2)) == 0.0
    assert steering.step(0.35, 0.4, 0.0, box(1.0, 0.2)) != 0.0


def test_letting_go_stops_the_turn_at_once_and_needs_a_new_push():
    steering = Steering()
    assert pushed(steering, box(1.0, 0.2)) != 0.0
    assert steering.step(1.2, 0.05, 0.0, box(1.0, 0.2)) == 0.0
    assert steering.step(1.3, 0.4, 0.0, box(1.0, 0.2)) == 0.0   # pushed again: not yet
    assert steering.step(1.7, 0.4, 0.0, box(1.0, 0.2)) != 0.0


def test_slower_push_keeps_a_turn_going_but_does_not_start_one():
    steering = Steering()
    assert pushed(steering, box(1.0, 0.2)) != 0.0
    assert steering.step(1.2, 0.12, 0.0, box(1.0, 0.2)) != 0.0  # above keep_mps
    assert pushed(Steering(), box(1.0, 0.2), speed=0.12) == 0.0  # below push_mps


def test_open_floor_and_a_wall_all_round():
    assert pushed(Steering(), np.zeros((0, 2))) == 0.0
    boxed_in = np.array([(1.0 * math.cos(a), 1.0 * math.sin(a))
                         for a in np.arange(-1.6, 1.6, 0.03)])
    steering = Steering()
    assert pushed(steering, boxed_in) == 0.0 and steering.why == 'no_way'


def test_a_turn_ends_after_its_time_or_its_angle_then_rests():
    steering = Steering()
    pushed(steering, box(1.0, 0.2), seconds=0.4)
    turn = pushed(steering, box(1.0, 0.2), start=0.5, seconds=S.max_turn_s)
    assert turn == 0.0 and steering.why == 'rest'
    steering = Steering()
    pushed(steering, box(1.0, 0.2), seconds=0.4)                # turn began facing 0
    assert steering.step(0.6, 0.4, -math.radians(61), box(1.0, 0.2)) == 0.0
    assert steering.why == 'rest'
    assert steering.step(0.7 + S.rest_s, 0.4, -math.radians(61), box(1.0, 0.2)) != 0.0


def test_centres_gently_in_a_narrow_gap_only():
    gap = np.vstack([wall(0.6), wall(-1.0)])                    # nearer on the left
    steering = Steering()
    turn = pushed(steering, gap)
    assert -S.centre_rps <= turn < 0 and steering.why == 'centre'
    assert pushed(Steering(), wall(0.6)) == 0.0                 # one wall is not a gap
    even = np.vstack([wall(0.8), wall(-0.8)])
    assert pushed(Steering(), even) == 0.0
    assert pushed(Steering(FreeSettings(centre=False)), gap) == 0.0


# ---- is the user pushing ----

def test_push_meter_reads_forward_speed_and_not_turning_on_the_spot():
    meter = PushMeter()
    for i in range(6):
        meter.add(i * 0.1, 0.04 * i, 0.0, 0.0)                  # 0.4 m/s along x, facing x
    assert meter.speed(0.5) == pytest.approx(0.4)
    spin = PushMeter()
    for i in range(6):
        spin.add(i * 0.1, 1.0, 1.0, 0.04 * i)                   # turning, not moving
    assert spin.speed(0.5) == pytest.approx(0.0)
    sideways = PushMeter()
    for i in range(6):
        sideways.add(i * 0.1, 0.0, 0.04 * i, 0.0)               # dragged sideways
    assert sideways.speed(0.5) == pytest.approx(0.0)
    assert meter.speed(2.0) == 0.0 and PushMeter().speed(0.0) == 0.0     # old, or no data
