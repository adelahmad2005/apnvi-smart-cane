"""The hazard gate and the vibration strength."""

from apnvi_stage4.rules import (CAREFUL_STRENGTH, from_front, gate, STOPPED_STRENGTH,
                                vibration_pattern, vibration_strength)
import pytest


@pytest.mark.parametrize('level', ['clear', 'notice', 'warning'])
def test_no_hazard_passes_the_speed(level):
    assert gate(0.4, 0.2, level, 0.1) == (0.4, 0.2, False)


def test_stop_level_zeroes_forward_and_keeps_the_turn():
    assert gate(0.4, 0.2, 'stop', 0.1) == (0.0, 0.2, True)


def test_stage1_quiet_or_never_heard_blocks_forward():
    assert gate(0.4, 0.2, 'clear', 0.8) == (0.0, 0.2, True)
    assert gate(0.4, 0.2, None, 0.0) == (0.0, 0.2, True)


def test_zero_or_backward_is_not_changed():
    assert gate(0.0, 0.5, 'stop', 0.1) == (0.0, 0.5, True)
    assert gate(-0.1, 0.0, 'stop', 0.1) == (-0.1, 0.0, True)


def test_distance_from_the_front_takes_off_the_camera_offset():
    # the camera is about 0.9 m behind the device's front
    assert from_front(1.4, 0.9) == pytest.approx(0.5)
    assert from_front(0.3, 0.9) == 0.0                 # never below 0
    assert from_front(-1, 0.9) is None                 # nothing in range
    assert from_front(None, 0.9) is None


def test_something_close_to_the_front_stops_even_before_stage1_says_stop():
    # 1.3 m from the camera is only 0.4 m from the front: Stage 1 still says "warning"
    assert gate(0.4, 0.2, 'warning', 0.1, from_front(1.3, 0.9)) == (0.0, 0.2, True)
    # 1.5 m from the camera is 0.6 m from the front: drive on
    assert gate(0.4, 0.2, 'warning', 0.1, from_front(1.5, 0.9)) == (0.4, 0.2, False)
    # nothing in range: drive on
    assert gate(0.4, 0.2, 'clear', 0.1, from_front(-1, 0.9)) == (0.4, 0.2, False)


def test_stays_blocked_until_clear_for_a_second():
    from apnvi_stage4.rules import hold_blocked
    assert hold_blocked(False, False, None, 0.0) == (False, None)    # was open, still open
    assert hold_blocked(True, False, None, 0.0) == (True, None)      # shuts at once
    assert hold_blocked(False, True, None, 1.0) == (True, 1.0)       # looks clear: hold on
    assert hold_blocked(False, True, 1.0, 1.5) == (True, 1.0)
    assert hold_blocked(True, True, 1.0, 1.7) == (True, None)        # flicker: start again
    assert hold_blocked(False, True, None, 2.0) == (True, 2.0)
    assert hold_blocked(False, True, 2.0, 3.0) == (False, None)      # clear for 1 s: open


def test_blocked_at_a_corner_turns_on_the_spot_the_way_the_route_bends():
    from apnvi_stage4.rules import pivot_when_blocked, PIVOT_RPS
    assert pivot_when_blocked(0.3, 0.2, True) == PIVOT_RPS            # bends left: turn left
    assert pivot_when_blocked(0.3, -0.1, True) == -PIVOT_RPS          # bends right
    assert pivot_when_blocked(0.3, 0.9, True) == 0.9                  # already fast: kept
    assert pivot_when_blocked(0.3, 0.01, True) == 0.01                # straight ahead: wait
    assert pivot_when_blocked(0.3, 0.2, False) == 0.2                 # not blocked: unchanged
    assert pivot_when_blocked(0.0, 0.2, True) == 0.2                  # no forward asked


def test_vibration_measured_from_the_front():
    assert vibration_strength(from_front(2.5, 0.9)) == 0      # 1.6 m from the front
    assert vibration_strength(from_front(2.4, 0.9)) == 30     # 1.5 m from the front
    assert vibration_strength(from_front(1.4, 0.9)) == 100    # 0.5 m from the front


def test_vibration_off_when_nothing_is_near():
    assert vibration_strength(-1) == 0
    assert vibration_strength(None) == 0
    assert vibration_strength(3.0) == 0
    assert vibration_strength(1.51) == 0


def test_vibration_stronger_when_closer():
    far, middle, near = vibration_strength(1.4), vibration_strength(1.0), vibration_strength(0.6)
    assert 0 < far < middle < near < 100


def test_vibration_full_at_the_stop_distance_and_closer():
    assert vibration_strength(0.5) == 100
    assert vibration_strength(0.1) == 100


def test_vibration_starts_at_the_weakest_felt_strength():
    assert vibration_strength(1.5) == 30


# ---- vibration only when it matters (Adel 15 Nov) ---------------------------------

def pattern_over(seconds, **state):
    """Give the strengths over a stretch of time at 10 a second."""
    return [vibration_pattern(t_s=k / 10.0, **state) for k in range(int(seconds * 10))]


def test_silent_when_not_guiding_whatever_is_near():
    assert set(pattern_over(3, guiding=False, blocked=True, front_m=0.1, blocked_for_s=1)) == {0}


def test_silent_on_open_floor_while_guiding():
    assert set(pattern_over(3, guiding=True, blocked=False, front_m=None, blocked_for_s=0)) == {0}
    assert set(pattern_over(3, guiding=True, blocked=False, front_m=1.5, blocked_for_s=0)) == {0}


def test_careful_is_two_short_pulses_then_quiet():
    one = pattern_over(1.2, guiding=True, blocked=False, front_m=0.8, blocked_for_s=0)
    on = [k for k, s in enumerate(one) if s]
    assert on == [0, 1, 4, 5] and max(one) == CAREFUL_STRENGTH


def test_stopped_is_steady_then_a_pulse_a_second():
    steady = pattern_over(2, guiding=True, blocked=True, front_m=0.3, blocked_for_s=1.0)
    assert set(steady) == {STOPPED_STRENGTH}
    later = pattern_over(2, guiding=True, blocked=True, front_m=0.3, blocked_for_s=5.0)
    assert later.count(STOPPED_STRENGTH) == 4 and later.count(0) == 16


def test_stopped_wins_over_careful():
    assert vibration_pattern(True, True, 0.8, 0.5, 0.7) == STOPPED_STRENGTH


# ---- the vibration signals (17 Nov redesign: near, stopped, left, right) ----

def _sig(now, t=0.0, **kw):
    from apnvi_stage4.rules import vibration_signal
    return vibration_signal(now, t, **kw)


def test_silent_when_all_is_fine_or_standing_still():
    from apnvi_stage4.rules import BOTH_MOTORS, VibeInput
    assert _sig(VibeInput(guiding=True)) == (0, BOTH_MOTORS, '')
    assert _sig(VibeInput(blocked=True, front_m=0.2)) == (0, BOTH_MOTORS, '')   # no trip


def test_near_is_soft_on_both_motors_and_faster_when_closer():
    from apnvi_stage4.rules import BOTH_MOTORS, NEAR_STRENGTH, VibeInput
    for mode in ({'guiding': True}, {'moving': True}):
        def pulses(front_m):
            return sum(1 for k in range(40)
                       if _sig(VibeInput(front_m=front_m, **mode), k / 10.0)[0])
        assert pulses(1.4) < pulses(1.0) < pulses(0.6)        # closer: more pulses
        assert _sig(VibeInput(front_m=1.0, **mode)) == (NEAR_STRENGTH, BOTH_MOTORS, 'near')
        assert _sig(VibeInput(front_m=1.6, **mode))[2] == ''  # beyond 1.5 m: quiet


def test_stopped_is_strong_on_both_motors():
    from apnvi_stage4.rules import BOTH_MOTORS, VibeInput
    blocked = VibeInput(guiding=True, blocked=True, blocked_for_s=1.0, front_m=0.3)
    assert _sig(blocked, 0.5) == (100, BOTH_MOTORS, 'stopped')
    later = VibeInput(guiding=True, blocked=True, blocked_for_s=4.0)
    assert _sig(later, 0.5)[0] == 0 and _sig(later, 1.0)[0] == 100


def test_left_and_right_on_that_sides_motor_only():
    from apnvi_stage4.rules import BOTH_MOTORS, VibeInput
    left = VibeInput(guiding=True, side='left', front_m=1.0)
    right = VibeInput(guiding=True, side='right')
    assert _sig(left, 0.0) == (80, 1, 'left') and _sig(left, 0.2)[0] == 0
    assert _sig(right, 0.0) == (80, 2, 'right')
    assert _sig(left, 0.0, left_motor=2)[1] == 2 and _sig(right, 0.0, left_motor=2)[1] == 1
    assert _sig(left, 0.0, two_sides=False)[1] == BOTH_MOTORS


def test_order_stopped_then_side_then_near():
    from apnvi_stage4.rules import VibeInput
    everything = VibeInput(guiding=True, blocked=True, blocked_for_s=1.0, front_m=0.4,
                           side='left', turn_rps=0.6, turning_for_s=1.0, need_help=True)
    assert _sig(everything)[2] == 'stopped'
    everything.blocked = False
    assert _sig(everything)[2] == 'left'
    everything.side = None
    assert _sig(everything)[2] == 'near'


def test_no_signals_for_what_is_spoken():
    from apnvi_stage4.rules import VibeInput
    # arrived, heard, help and being turned are spoken or felt through the handle (Adel)
    for now in (VibeInput(heard_ago_s=0.05), VibeInput(arrived_ago_s=0.2),
                VibeInput(guiding=True, need_help=True),
                VibeInput(guiding=True, turn_rps=0.6, turning_for_s=1.0)):
        assert _sig(now)[0] == 0


def test_every_named_test_signal_plays_that_signal():
    from apnvi_stage4.rules import BOTH_MOTORS, signal_input, SIGNAL_NAMES, vibration_signal
    assert set(SIGNAL_NAMES) == {'near', 'stopped', 'left', 'right'}
    for name in SIGNAL_NAMES:
        played = {vibration_signal(signal_input(name, t / 10.0), t / 10.0)[2]
                  for t in range(40)}
        assert name in played, name
    assert vibration_signal(signal_input('nonsense', 0.0), 0.0) == (0, BOTH_MOTORS, '')


def test_a_turn_on_the_spot_keeps_its_way_until_the_device_drives_on():
    # 18 Nov: the goal straight behind, the follower asked left, right, left...
    from apnvi_stage4.rules import TurnLatch
    latch = TurnLatch()
    assert latch.apply(0.0, 0.0, 0.6) == 0.6
    assert latch.apply(0.5, 0.0, -0.6) == 0.0           # a blip the other way: wait, don't wiggle
    assert latch.apply(0.6, 0.0, 0.4) == 0.4            # back the kept way
    assert latch.apply(0.7, 0.0, -0.1) == 0.0
    assert latch.apply(1.0, 0.0, 0.0) == 0.0
    assert latch.apply(1.5, 0.2, -0.3) == -0.3          # driving on: free again
    assert latch.apply(1.6, 0.0, -0.6) == -0.6          # a new turn picks its way
    assert latch.apply(5.0, 0.0, 0.6) == 0.6            # long after: picks again


def test_turn_latch_follows_the_other_way_when_it_is_asked_for_a_second():
    from apnvi_stage4.rules import TurnLatch
    latch = TurnLatch()
    assert latch.apply(0.0, 0.0, 0.3) == 0.3            # left once (by the desk, 19:45:59)
    assert latch.apply(0.2, 0.0, -0.28) == 0.0          # then right, away from the desk
    assert latch.apply(0.6, 0.0, 0.0) == 0.0
    assert latch.apply(0.9, 0.0, -0.25) == 0.0
    assert latch.apply(1.2, 0.0, -0.27) == -0.27        # right for a second: it means it
    assert latch.apply(1.4, 0.0, 0.3) == 0.0            # now right is kept


def test_turn_latch_small_turn_before_a_way_is_picked_passes():
    from apnvi_stage4.rules import TurnLatch
    latch = TurnLatch()
    assert latch.apply(0.0, 0.0, -0.1) == -0.1          # no way picked yet: as it is


def test_turn_latch_release_lets_the_guide_turn_its_own_way():
    from apnvi_stage4.rules import TurnLatch
    latch = TurnLatch()
    assert latch.apply(0.0, 0.0, 0.6) == 0.6
    latch.release()                                     # getting unstuck: "turning right"
    assert latch.apply(0.1, 0.0, -0.6) == -0.6


def test_lidar_quiet():
    # 23 Nov: the LiDAR stopped mid-trip and the device kept turning blind.
    from apnvi_stage4.rules import lidar_quiet
    assert lidar_quiet(10.0, None)                  # no sweep yet
    assert not lidar_quiet(10.0, 9.7)               # 0.3 s ago
    assert lidar_quiet(10.0, 9.4)                   # 0.6 s ago
    assert not lidar_quiet(10.0, 9.4, quiet_s=1.0)
