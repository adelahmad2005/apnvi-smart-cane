"""Moving a stop point out of a spot the device cannot reach (18 Nov, the bin in a corner)."""

import pytest

from apnvi_stage4.reach import reachable_point


def room(width=40, height=40, cost=0):
    return [cost] * (width * height)


def test_a_free_stop_point_stays():
    assert reachable_point(room(), 40, 40, 0.1, 0.0, 0.0, 2.0, 2.0) == (2.0, 2.0)


def test_a_squeezed_stop_point_moves_to_the_nearest_free_spot():
    costs = room()
    for row in range(40):
        for column in range(25, 40):            # x from 2.5 m on: too close to things
            costs[row * 40 + column] = 99
    x, y = reachable_point(costs, 40, 40, 0.1, 0.0, 0.0, 3.0, 2.0)
    assert x == pytest.approx(2.45) and y == pytest.approx(2.05)


def test_no_free_spot_near_gives_none():
    assert reachable_point(room(cost=99), 40, 40, 0.1, 0.0, 0.0, 2.0, 2.0) is None


def test_unseen_cells_are_not_used():
    costs = room(cost=-1)
    costs[20 * 40 + 20] = 99
    assert reachable_point(costs, 40, 40, 0.1, 0.0, 0.0, 2.0, 2.0) is None


def test_off_the_map_stays():
    assert reachable_point(room(), 40, 40, 0.1, 0.0, 0.0, 9.0, 9.0) == (9.0, 9.0)


def test_stop_point_moves_are_measured_from_the_point_asked_for():
    # 19 Nov demo: each second the stop point moved again from the last moved point and crept
    # 1.9 m away. The finder here always moves a point 0.3 m right, so creeping would show.
    from apnvi_stage4.guide import Goal, move_stop_point
    goal = Goal(10.0, 5.0, 'the sign')

    def finder(x, y):
        return (x + 0.3, y)

    assert move_stop_point(goal, finder) == (10.0, 5.0)
    for _ in range(5):
        assert move_stop_point(goal, finder) is None
    assert (goal.x, goal.y) == pytest.approx((10.3, 5.0))
    assert goal.wanted == (10.0, 5.0)


def test_stop_point_goes_back_when_the_asked_point_fits_again():
    from apnvi_stage4.guide import Goal, move_stop_point
    goal = Goal(1.0, 1.0)
    move_stop_point(goal, lambda x, y: (x, y + 0.5))
    assert move_stop_point(goal, lambda x, y: (x, y)) == pytest.approx((1.0, 1.5))
    assert (goal.x, goal.y) == (1.0, 1.0)
    assert move_stop_point(goal, lambda x, y: None) is None   # nothing fits: the asked point


def walled_room():
    """Make a 4 x 4 m room at 0.1 m cells with a wall two cells thick at x 2.0 to 2.2 m."""
    costs = room()
    for row in range(40):
        for column in (20, 21):
            costs[row * 40 + column] = 100
    return costs


def test_a_stop_point_on_a_wall_moves_to_the_device_side():
    # 23 Nov: the stop point by the door sign once moved behind the door, where no route went.
    # Without the device's position the nearest free cell is behind the wall (x 2.25).
    x, _ = reachable_point(walled_room(), 40, 40, 0.1, 0.0, 0.0, 2.15, 2.05)
    assert x == pytest.approx(2.25)
    x, _ = reachable_point(walled_room(), 40, 40, 0.1, 0.0, 0.0, 2.15, 2.05, near=(0.5, 2.0))
    assert x == pytest.approx(1.95)


def test_a_device_behind_the_wall_gets_the_point_on_its_side():
    x, _ = reachable_point(walled_room(), 40, 40, 0.1, 0.0, 0.0, 2.05, 2.05, near=(3.5, 2.0))
    assert x == pytest.approx(2.25)


def test_with_a_wall_on_every_side_the_nearest_is_used():
    costs = walled_room()
    for row in range(40):
        for column in range(0, 20):
            costs[row * 40 + column] = 99 if column != 19 else 0
    for row in range(40):
        costs[row * 40 + 10] = 100              # a second wall between x 1.9 and the device
    x, _ = reachable_point(costs, 40, 40, 0.1, 0.0, 0.0, 2.05, 2.05, near=(0.3, 2.0))
    assert x in (pytest.approx(1.95), pytest.approx(2.25))


def test_fits_point():
    from apnvi_stage4.reach import fits_point
    costs = walled_room()
    assert fits_point(costs, 40, 40, 0.1, 0.0, 0.0, 1.0, 1.0)
    assert not fits_point(costs, 40, 40, 0.1, 0.0, 0.0, 2.05, 1.0)
    assert fits_point(costs, 40, 40, 0.1, 0.0, 0.0, 9.0, 9.0)       # off the map


def test_a_held_stop_point_stays_while_it_fits():
    # 23 Nov: the point was picked again every second and jumped 40 times in 80 s.
    from apnvi_stage4.guide import Goal, move_stop_point
    goal = Goal(5.0, -3.9, 'the door')
    assert move_stop_point(goal, lambda x, y: (4.7, -3.4), lambda x, y: True) is not None
    goal.held = True                            # the planner found a route to (4.7, -3.4)
    for other in ((4.9, -3.1), (5.3, -4.6), (4.6, -3.2)):
        assert move_stop_point(goal, lambda x, y, p=other: p, lambda x, y: True) is None
    assert (goal.x, goal.y) == (4.7, -3.4)


def test_a_stop_point_with_no_route_yet_is_picked_again():
    from apnvi_stage4.guide import Goal, move_stop_point
    goal = Goal(5.0, -3.9)
    move_stop_point(goal, lambda x, y: (5.3, -4.6), lambda x, y: True)
    assert goal.held is False                   # no route reached it (behind the door)
    assert move_stop_point(goal, lambda x, y: (4.7, -3.4), lambda x, y: True) is not None
    assert (goal.x, goal.y) == (4.7, -3.4)


def test_a_held_stop_point_that_no_longer_fits_moves():
    from apnvi_stage4.guide import Goal, move_stop_point
    goal = Goal(5.0, -3.9)
    move_stop_point(goal, lambda x, y: (4.7, -3.4), lambda x, y: True)
    goal.held = True
    assert move_stop_point(goal, lambda x, y: (4.6, -3.2), lambda x, y: False) == (4.7, -3.4)
    assert (goal.x, goal.y) == (4.6, -3.2) and goal.held is False
