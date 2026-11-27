"""Getting unstuck: stopped by the camera, the device turns to clear floor and drives on."""

import math

from apnvi_stage4.free_mode import FreeSettings
from apnvi_stage4.guide import BLOCKED, STILL_BLOCKED
from apnvi_stage4.unstuck import (
    angle_between, person_ahead, pick_side, Seen, TURNING, Unstuck, UnstuckSettings, way_clear)
import numpy as np

LANES = FreeSettings()
NOTHING = np.zeros((0, 2))
# The numbers these tests were written with (17 Nov). 23 Nov made the defaults slower and quicker
# to start (test_the_23_nov_numbers); the behaviour checked here is the same with either.
OLD = UnstuckSettings(wait_s=4.0, still_s=15.0, person_m=1.0, turn_rps=0.6, confirm_s=0.3,
                      person_wait_s=1000.0)


def box(x, y, size=0.3, step=0.05):
    """Points on a small box centred at (x, y) in the device's frame."""
    line = np.arange(-size / 2, size / 2 + 1e-9, step)
    return np.array([(x + a, y + b) for a in line for b in line])


def desk_ahead(**changes):
    """Give a trip stopped by a desk the camera sees 0.3 m ahead; the LiDAR sees nothing."""
    seen = Seen(guiding=True, blocked=True, stage1_ok=True, camera_m=0.3, points=NOTHING,
                lidar=NOTHING, yaw=0.0, x=0.0, y=0.0)
    for name, value in changes.items():
        setattr(seen, name, value)
    return seen


def run(unstuck, seen, start, end, step=0.1):
    """Step from start to end with the same view; give [(t, command, actions)]."""
    out, t = [], start
    while t <= end + 1e-9:
        command, actions = unstuck.step(round(t, 3), seen)
        out.append((round(t, 3), command, actions))
        t += step
    return out


def said(out):
    return [a[1] for _t, _c, actions in out for a in actions if a[0] == 'say']


def started(unstuck, seen=None):
    """Bring a fresh Unstuck to the start of its turn (4 s stopped, 1.5 s announced)."""
    out = run(unstuck, seen or desk_ahead(), 0.0, 5.6)
    assert unstuck.phase == 'turn'
    return out


# ---- the parts ----

def test_angle_between_wraps_round():
    assert math.isclose(angle_between(3.0, -3.0), 2 * math.pi - 6.0)
    assert math.isclose(angle_between(0.0, -0.5), 0.5)


def test_person_is_what_the_lidar_sees_straight_ahead():
    assert person_ahead(box(0.8, 0.0), LANES, 1.0)
    assert not person_ahead(box(2.0, 0.0), LANES, 1.0)          # further than person_m
    assert not person_ahead(box(0.8, 1.2), LANES, 1.0)          # beside the lane
    assert not person_ahead(NOTHING, LANES, 1.0)
    assert not person_ahead(None, LANES, 1.0)


def test_side_with_room_wins():
    wall_left = np.array([(x, 0.6) for x in np.arange(0.0, 2.0, 0.05)])
    assert pick_side(wall_left, LANES) == -1
    wall_right = wall_left * np.array([1, -1])
    assert pick_side(wall_right, LANES) == 1


def test_side_with_nothing_seen_goes_toward_the_goal():
    assert pick_side(NOTHING, LANES, goal_side=-1) == -1
    assert pick_side(NOTHING, LANES, goal_side=1) == 1
    assert pick_side(NOTHING, LANES) == 1


def test_way_clear_needs_both_camera_and_lidar():
    assert way_clear(desk_ahead(camera_m=None), LANES, 1.2)
    assert way_clear(desk_ahead(camera_m=1.5), LANES, 1.2)
    assert not way_clear(desk_ahead(camera_m=0.9), LANES, 1.2)
    assert not way_clear(desk_ahead(camera_m=None, camera_stop=True), LANES, 1.2)
    assert not way_clear(desk_ahead(camera_m=None, points=box(0.9, 0.0)), LANES, 1.2)


# ---- a whole try ----

def test_waits_a_few_seconds_then_says_which_way_and_turns():
    unstuck = Unstuck(OLD)
    out = run(unstuck, desk_ahead(goal_side=-1), 0.0, 3.9)
    assert all(command is None and not actions for _t, command, actions in out)
    command, actions = unstuck.step(4.0, desk_ahead(goal_side=-1))
    assert command == (0.0, 0.0)
    assert actions == [('hold',), ('say', TURNING % 'right')]
    out = run(unstuck, desk_ahead(goal_side=-1), 4.1, 5.4)
    assert all(command == (0.0, 0.0) for _t, command, _a in out)    # still while it speaks
    command, _ = unstuck.step(5.5, desk_ahead(goal_side=-1))
    assert command == (0.0, -0.6)


def test_turns_until_clear_then_drives_half_a_metre_and_hands_back():
    unstuck = Unstuck(OLD)
    started(unstuck)
    run(unstuck, desk_ahead(yaw=0.4), 5.7, 6.0)
    assert unstuck.phase == 'turn'
    clear = desk_ahead(yaw=0.7, camera_m=None)
    run(unstuck, clear, 6.1, 6.5)
    assert unstuck.phase == 'settle'                    # clear for 0.3 s: stop turning
    command, _ = unstuck.step(6.6, clear)
    assert command == (0.0, 0.0)                        # the hazard gate still holds it
    free = desk_ahead(yaw=0.7, camera_m=None, blocked=False)
    command, _ = unstuck.step(7.0, free)
    assert command == (0.15, 0.0) and unstuck.phase == 'drive'
    command, _ = unstuck.step(8.0, desk_ahead(yaw=0.7, camera_m=None, blocked=False, x=0.3))
    assert command == (0.15, 0.0)
    command, actions = unstuck.step(9.0, desk_ahead(yaw=0.7, camera_m=None, blocked=False,
                                                    x=0.4, y=0.3))
    assert command == (0.0, 0.0) and actions == [('resume',)]
    assert not unstuck.active and unstuck.why == 'done'
    assert unstuck.step(9.1, free) == (None, [])        # the route drives again


def test_stopped_again_while_driving_hands_back_at_once():
    unstuck = Unstuck(OLD)
    started(unstuck)
    run(unstuck, desk_ahead(camera_m=None), 5.7, 6.1)
    unstuck.step(6.2, desk_ahead(camera_m=None, blocked=False))
    assert unstuck.phase == 'drive'
    command, actions = unstuck.step(6.3, desk_ahead(x=0.1))
    assert command == (0.0, 0.0) and actions == [('resume',)]


def test_gate_that_never_lets_go_ends_the_try():
    unstuck = Unstuck(OLD)
    started(unstuck)
    out = run(unstuck, desk_ahead(camera_m=None), 5.7, 9.5)
    assert ('resume',) in [a for _t, _c, actions in out for a in actions]
    assert unstuck.why == 'gate kept it stopped'


def test_never_drives_backwards():
    unstuck = Unstuck(OLD)
    out = started(unstuck)
    out += run(unstuck, desk_ahead(), 5.7, 20.0)
    assert all(command is None or command[0] >= 0.0 for _t, command, _a in out)


# ---- when it does not try ----

def test_person_ahead_just_waits_and_talks_as_before():
    unstuck = Unstuck(OLD)
    out = run(unstuck, desk_ahead(lidar=box(0.8, 0.0), points=box(0.8, 0.0)), 0.0, 20.0)
    assert all(command is None for _t, command, _a in out)
    assert said(out) == [BLOCKED, STILL_BLOCKED]
    assert [t for t, _c, actions in out if actions] == [4.0, 15.0]


def test_person_leaves_but_a_desk_stays_then_it_tries():
    unstuck = Unstuck(OLD)
    run(unstuck, desk_ahead(lidar=box(0.8, 0.0)), 0.0, 6.0)
    _command, actions = unstuck.step(6.1, desk_ahead())
    assert ('hold',) in actions


def test_no_try_while_stage1_is_quiet_or_the_master_stops_or_eyes_are_old():
    for changes in ({'stage1_ok': False}, {'master_stop': True}, {'points': None},
                    {'yaw': None}):
        unstuck = Unstuck(OLD)
        out = run(unstuck, desk_ahead(**changes), 0.0, 6.0)
        assert all(command is None for _t, command, _a in out), changes
        assert said(out) == [BLOCKED]


def test_not_blocked_does_nothing():
    unstuck = Unstuck(OLD)
    out = run(unstuck, desk_ahead(blocked=False), 0.0, 30.0)
    assert all(command is None and not actions for _t, command, actions in out)


# ---- safety: it stops at once ----

def test_stage1_quiet_or_a_master_stop_ends_the_turn_at_once():
    for changes in ({'stage1_ok': False}, {'master_stop': True}, {'points': None},
                    {'x': None}):
        unstuck = Unstuck(OLD)
        started(unstuck)
        command, actions = unstuck.step(5.7, desk_ahead(**changes))
        assert command == (0.0, 0.0) and actions == [('resume',)], changes
        assert not unstuck.active


def test_paused_trip_stops_at_once_and_forgets_the_tries():
    unstuck = Unstuck(OLD)
    started(unstuck)
    paused = desk_ahead(guiding=False)
    assert unstuck.step(5.7, paused) == ((0.0, 0.0), [])
    assert unstuck.step(5.8, paused) == (None, [])
    assert unstuck.tries == 0 and not unstuck.active


# ---- tries ----

def turning(unstuck, steps):
    """Step through (time, yaw) pairs with the desk still ahead; give the last command."""
    command = None
    for t, yaw in steps:
        command, _actions = unstuck.step(t, desk_ahead(yaw=yaw))
    return command


def test_turn_that_finds_nothing_tries_the_other_way():
    unstuck = Unstuck(OLD)
    started(unstuck, desk_ahead(goal_side=1))
    assert unstuck.side == 1
    assert turning(unstuck, ((5.7, 0.5), (5.8, 1.0))) == (0.0, 0.6)
    assert turning(unstuck, ((5.9, 1.6),)) == (0.0, 0.0)    # 92 degrees, nothing clear
    assert unstuck.phase == 'gap'
    out = run(unstuck, desk_ahead(yaw=1.6), 6.0, 7.0)
    assert unstuck.side == -1 and unstuck.phase == 'announce' and unstuck.tries == 2
    assert said(out) == [TURNING % 'right']
    assert ('hold',) not in [a for _t, _c, actions in out for a in actions]
    assert math.isclose(unstuck.budget, math.radians(90) + 1.6)    # back, then 90 more


def test_after_two_tries_asks_to_be_turned():
    unstuck = Unstuck(OLD)
    started(unstuck)
    turning(unstuck, ((5.7, 1.6),))                     # try 1: nothing clear within 90 deg
    run(unstuck, desk_ahead(yaw=1.6), 5.8, 8.5)         # try 2 (right) turns from 8.2
    assert unstuck.tries == 2 and unstuck.phase == 'turn' and unstuck.side == -1
    turning(unstuck, ((8.6, 0.5), (8.7, -0.5), (8.8, -1.6)))
    assert unstuck.phase == 'gap'                       # 183 degrees back: nothing clear
    out = run(unstuck, desk_ahead(yaw=-1.6), 8.9, 30.0)
    assert ('resume',) in [a for _t, _c, actions in out for a in actions]
    assert not unstuck.active
    assert said(out) == [STILL_BLOCKED]
    late = [command for t, command, _a in out if t > 9.9]
    assert all(command is None for command in late)


def test_driving_free_for_a_while_forgets_the_tries():
    unstuck = Unstuck(OLD)
    unstuck.tries = 2
    run(unstuck, desk_ahead(blocked=False), 0.0, 8.0)
    assert unstuck.tries == 0


def test_the_23_nov_numbers():
    # Adel, 23 Nov: turns on the spot too quick to follow; a door frame within 1 m waited for 17 s.
    from apnvi_stage4.guide import BLOCKED_SAY_S, STILL_BLOCKED_S
    from apnvi_stage4.rules import PIVOT_RPS
    settings = UnstuckSettings()
    assert settings.turn_rps == PIVOT_RPS == 0.4
    assert settings.person_m == 0.6 and settings.confirm_s == 0.2
    assert settings.wait_s == BLOCKED_SAY_S == 3.0 and settings.still_s == STILL_BLOCKED_S == 8.0


def test_a_wall_close_ahead_is_waited_for_only_a_few_seconds():
    # 23 Nov (Adel): stuck at a corner, "Turn me a little..." again and again. A wall 0.5 m
    # straight ahead looked like a person, and a person is waited for; a wall never leaves.
    unstuck = Unstuck()
    corner = desk_ahead(lidar=box(0.5, 0.0), points=box(0.5, 0.0))
    out = run(unstuck, corner, 0.0, 5.9)
    assert all(command is None for _t, command, _a in out)
    assert said(out) == [BLOCKED]
    _command, actions = unstuck.step(6.0, corner)
    assert ('hold',) in actions


def test_no_route_is_a_hold_up_too():
    # 23 Nov (Adel): by a desk corner the planner found no route, the camera saw 3.7 m clear,
    # nothing blocked forward driving, and no try came. Now no route starts a try after wait_s;
    # the guide does the talking ("I can't find a way there right now...").
    unstuck = Unstuck()
    open_ahead = desk_ahead(blocked=False, no_route=True, camera_m=3.7)
    out = run(unstuck, open_ahead, 0.0, 2.9)
    assert all(command is None and not actions for _t, command, actions in out)
    _command, actions = unstuck.step(3.0, open_ahead)
    assert ('hold',) in actions
