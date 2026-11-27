"""Wheel speeds, caps, the master's reports, and what the drive link sends."""

import json

from apnvi_stage4.drive import (DriveLink, DriveSettings, MasterState, wheel_power,
                                wheel_rpm, wheel_speeds)
import pytest

SETTINGS = DriveSettings(wheel_spacing_m=0.4, wheel_diameter_m=0.15, speed_at_cap_mps=0.5,
                         min_power_pct=0, power_table=(), turn_gain=1.0,
                         max_speed_mps=0.5, max_turn_rps=1.0, pivot_power_pct=0)


def test_straight_means_both_wheels_the_same():
    assert wheel_speeds(0.3, 0.0, 0.4) == (0.3, 0.3)


def test_positive_turn_makes_the_right_wheel_faster():
    left, right = wheel_speeds(0.3, 0.5, 0.4)
    assert left == pytest.approx(0.2) and right == pytest.approx(0.4)


def test_turn_on_the_spot():
    left, right = wheel_speeds(0.0, 1.0, 0.4)
    assert left == pytest.approx(-0.2) and right == pytest.approx(0.2)


def test_power_is_percent_of_the_speed_at_the_cap():
    assert wheel_power(0.25, 0.0, SETTINGS) == (50, 50)
    assert wheel_power(0.5, 0.0, SETTINGS) == (100, 100)


def test_forward_speed_is_capped_at_the_top_speed():
    assert wheel_power(2.0, 0.0, SETTINGS) == (100, 100)
    slow = DriveSettings(speed_at_cap_mps=1.0, max_speed_mps=0.5, power_table=(), min_power_pct=0,
                         turn_gain=1.0)
    assert wheel_power(2.0, 0.0, slow) == (50, 50)


def test_no_reversing_unless_allowed():
    assert wheel_power(-0.3, 0.0, SETTINGS) == (0, 0)
    reverse = DriveSettings(speed_at_cap_mps=0.5, allow_reverse=True, power_table=())
    assert wheel_power(-0.25, 0.0, reverse) == (-50, -50)


def test_a_wheel_over_the_cap_scales_both_and_keeps_the_curve():
    left, right = wheel_power(0.5, 1.0, SETTINGS)     # right would need 0.7 m/s
    assert right == 100
    assert left == round(100 * 0.3 / 0.7)


def test_turn_rate_is_capped():
    assert wheel_power(0.0, 5.0, SETTINGS) == wheel_power(0.0, 1.0, SETTINGS)


def test_straight_line_mode_raises_small_power_to_the_lowest_that_moves():
    line = DriveSettings(speed_at_cap_mps=0.64, min_power_pct=45, power_table=())
    assert wheel_power(0.2, 0.0, line) == (45, 45)             # 31% would only hum (15 Nov)
    assert wheel_power(0.4, 0.0, line) == (62, 62)             # above it: unchanged
    assert wheel_power(0.0, 0.0, line) == (0, 0)


def test_floor_table_gives_the_measured_powers():
    floor = DriveSettings()                                     # the 15 Nov floor table
    assert wheel_power(0.142, 0.0, floor) == (62, 62)
    assert wheel_power(0.31, 0.0, floor) == (78, 78)
    assert wheel_power(0.226, 0.0, floor) == (70, 70)           # halfway: halfway
    assert wheel_power(0.04, 0.0, floor) == (45, 45)            # slow: the lowest that moves
    assert wheel_power(0.02, 0.0, floor) == (0, 0)              # too slow to bother
    assert wheel_power(0.5, 0.0, floor) == (96, 96)


def test_floor_table_caps_at_full_power_and_keeps_the_curve():
    floor = DriveSettings(max_speed_mps=1.0)
    assert wheel_power(1.0, 0.0, floor) == (100, 100)
    left, right = wheel_power(0.54, 1.0, floor)                 # right wheel would need too much
    assert right == 100 and 0 < left < right


def test_turns_get_the_turn_gain():
    plain = DriveSettings(turn_gain=1.0, pivot_power_pct=0)
    boosted = DriveSettings(turn_gain=1.8, pivot_power_pct=0)
    assert abs(wheel_power(0.0, 0.5, boosted)[1]) > abs(wheel_power(0.0, 0.5, plain)[1])
    assert wheel_power(0.3, 0.0, boosted) == wheel_power(0.3, 0.0, plain)   # straight: same


def test_floor_table_spin_and_pivot():
    floor = DriveSettings(turn_gain=1.0, pivot_power_pct=0)
    left, right = wheel_power(0.0, 1.0, floor)                  # spin: 0.18 m/s each way
    assert left == -right and right > 62
    left, right = wheel_power(0.2, 0.9, floor)                  # 0.035 m/s and 0.365 m/s
    assert left == 45 and right > 78


def test_every_turn_on_the_spot_gets_the_pivot_power():
    # 23 Nov: 45% did not move, 50% 19 deg/s held, 52% 31-37 on a trip, 80% about 100 deg/s.
    settings = DriveSettings()
    for asked in (0.04, 0.2, 0.4, 1.0):     # the follower's first small ask, up to the cap
        assert wheel_power(0.0, asked, settings) == (-50, 50)
        assert wheel_power(0.0, -asked, settings) == (50, -50)
    assert wheel_power(0.01, 0.4, settings) == (-50, 50)   # creeping counts as on the spot
    assert wheel_power(0.0, 0.01, settings) == (0, 0)      # a tiny ask is no turn
    assert wheel_power(0.0, 0.0, settings) == (0, 0)


def test_turns_while_driving_are_unchanged_by_the_pivot_power():
    on, off = DriveSettings(), DriveSettings(pivot_power_pct=0)
    for forward, turn in ((0.3, 0.0), (0.3, 0.5), (0.2, -0.9), (0.05, 0.3)):
        assert wheel_power(forward, turn, on) == wheel_power(forward, turn, off)


def test_a_turn_on_the_spot_starts_with_a_kick():
    # 23 Nov, EB2: 50% turned only 11 deg/s and stalled at the start of a turn.
    link = DriveLink(DriveSettings())
    link.master.feed('ready')
    link.set_speed(0.0, 0.4, 10.0)
    assert link.wanted_power(10.0) == (-58, 58)          # kick
    link.set_speed(0.0, 0.4, 10.3)
    assert link.wanted_power(10.3) == (-58, 58)
    link.set_speed(0.0, 0.4, 10.6)
    assert link.wanted_power(10.6) == (-50, 50)          # then the normal pivot power
    link.set_speed(0.0, -0.4, 10.7)
    assert link.wanted_power(10.7) == (58, -58)          # the other way: a new kick
    link.set_speed(0.2, 0.0, 11.0)
    assert link.wanted_power(11.0)[0] == link.wanted_power(11.0)[1] > 0     # driving: no kick
    link.set_speed(0.0, -0.4, 11.1)
    assert link.wanted_power(11.1) == (58, -58)          # a new turn on the spot: kicked again
    link = DriveLink(DriveSettings(pivot_kick_pct=0))
    link.set_speed(0.0, 0.4, 10.0)
    assert link.wanted_power(10.0) == (-50, 50)          # kick off


def test_rpm_for_checking_by_eye():
    # 0.5 m/s on a 0.15 m wheel is about 63.7 turns a minute
    assert wheel_rpm(100, SETTINGS) == pytest.approx(63.66, abs=0.05)
    assert wheel_rpm(0, SETTINGS) == 0


def test_master_reports_are_read():
    master = MasterState()
    assert master.feed('who master') == [('master',)]
    assert master.feed('btn red down') == [('red', True)]
    assert master.feed('stop red') == [('stop', 'red')]
    assert master.feed('btn green on') == [('green', True)]
    assert master.feed('slave lost') == [('handle', False)]
    assert master.feed('clear jetson') == [('clear', 'jetson')]
    assert master.feed('alive 3 s, slave up stops red l 0 r 0 rear -1.00')[-1][0] == 'alive'
    assert master.feed('motors: something new')[0][0] == 'other'
    assert master.feed('') == []
    assert master.red and master.green and 'red' in master.stops


def test_master_boot_banner_and_restart():
    master = MasterState()
    master.feed('who master. Power cap PWM 100 of 255. Rear check off.')
    assert master.is_master
    for line in ('clear handle', 'clear jetson', 'ready'):
        master.feed(line)
    assert master.may_drive()
    master.feed('=== ESP32 boot ===')                  # it restarted
    assert not master.may_drive() and master.stops == {'handle', 'jetson'}


def test_alive_line_catches_up_on_a_master_that_was_already_running():
    # 13 Nov: the drive link joined a master that had sent "clear handle" long before, so it
    # kept "handle" as a stop forever and sent only 0. The alive line puts that right.
    master = MasterState()
    master.feed('who master')
    events = master.feed('alive 118 s, slave up stops jetson l 0 r 0 rear -1.00')
    assert ('clear', 'handle') in events and ('handle', True) in events
    assert master.stops == {'jetson'} and master.handle_up
    master.feed('clear jetson')
    master.feed('ready')
    master.feed('alive 121 s, slave up stops none l 0 r 0 rear -1.00')
    assert master.may_drive()


def test_alive_line_adds_a_stop_it_missed():
    master = MasterState()
    for line in ('who master', 'slave connected', 'clear handle', 'clear jetson', 'ready'):
        master.feed(line)
    events = master.feed('alive 9 s, slave down stops handle,red l 0 r 0 rear -1.00')
    assert ('stop', 'handle') in events and ('stop', 'red') in events
    assert ('handle', False) in events
    assert not master.may_drive() and not master.ready


def test_alive_line_with_unknown_words_changes_nothing():
    master = MasterState()
    master.feed('alive 5 s')                           # cut short
    assert master.stops == {'handle', 'jetson'}


def ready_link(settings=SETTINGS):
    link = DriveLink(settings)
    for line in ('who master', 'clear handle', 'clear jetson', 'ready'):
        link.on_line(line)
    return link


def test_sends_zero_until_the_master_is_ready():
    link = DriveLink(SETTINGS)
    link.set_speed(0.25, 0.0, now=10.0)
    assert link.lines(10.0) == ['d 0 0', 'vibe 0']
    link.on_line('who master')
    link.on_line('clear handle')
    link.on_line('clear jetson')
    assert link.lines(10.0)[0] == 'd 0 0'             # stops gone, but no "ready" yet
    link.on_line('ready')
    assert link.lines(10.0)[0] == 'd 50 50'


@pytest.mark.parametrize('reason', ['red', 'green', 'handle', 'jetson', 'rear'])
def test_every_stop_reason_sends_zero(reason):
    link = ready_link()
    link.set_speed(0.25, 0.0, now=1.0)
    link.on_line('stop ' + reason)
    assert link.lines(1.0)[0] == 'd 0 0'
    link.on_line('clear ' + reason)
    assert link.lines(1.0)[0] == 'd 0 0'              # still waits for "ready"
    link.on_line('ready')
    if reason == 'green':                             # green also needs a 0 speed first
        assert link.lines(1.0)[0] == 'd 0 0'
        link.set_speed(0.0, 0.0, now=1.0)
        link.lines(1.0)
        link.set_speed(0.25, 0.0, now=1.0)
    assert link.lines(1.0)[0] == 'd 50 50'


def test_after_green_a_speed_still_being_sent_does_not_restart():
    link = ready_link()
    link.set_speed(0.25, 0.0, now=1.0)
    link.on_line('btn green on')
    link.on_line('stop green')
    assert link.lines(1.0)[0] == 'd 0 0'
    link.on_line('btn green off')
    link.on_line('clear green')
    link.on_line('ready')
    for t in (1.1, 1.2, 1.3):                         # the same speed keeps coming
        link.set_speed(0.25, 0.0, now=t)
        assert link.lines(t)[0] == 'd 0 0'
    assert json.loads(link.status(True))['green_hold'] is True
    link.set_speed(0.0, 0.0, now=1.4)                 # asked to stop once
    assert link.lines(1.4)[0] == 'd 0 0'
    link.set_speed(0.25, 0.0, now=1.5)                # then a fresh request
    assert link.lines(1.5)[0] == 'd 50 50'


def test_after_green_no_speed_at_all_also_releases_the_hold():
    link = ready_link()
    link.set_test_power(40, 40, now=1.0)
    link.on_line('stop green')
    link.lines(1.0)
    link.on_line('clear green')
    link.on_line('ready')
    assert link.lines(1.1)[0] == 'd 0 0'              # test power still fresh: held
    assert link.lines(2.0)[0] == 'd 0 0'              # nothing asked for: hold released
    link.set_test_power(40, 40, now=2.1)
    assert link.lines(2.1)[0] == 'd 40 40'


def test_red_does_not_hold():
    link = ready_link()
    link.set_speed(0.25, 0.0, now=1.0)
    link.on_line('stop red')
    link.lines(1.0)
    link.on_line('clear red')
    link.on_line('ready')
    assert link.lines(1.0)[0] == 'd 50 50'           # red: the guide decides, not the link


def test_red_or_green_alone_also_send_zero():
    link = ready_link()
    link.set_speed(0.25, 0.0, now=1.0)
    link.on_line('btn red down')
    assert link.lines(1.0)[0] == 'd 0 0'
    link.on_line('btn red up')
    link.on_line('btn green on')
    assert link.lines(1.0)[0] == 'd 0 0'


def test_old_speed_goes_to_zero():
    link = ready_link()
    link.set_speed(0.25, 0.0, now=1.0)
    assert link.lines(1.2)[0] == 'd 50 50'
    assert link.lines(1.4)[0] == 'd 0 0'              # 0.4 s old: more than 0.3 s


def test_vibration_is_sent_and_times_out():
    link = ready_link()
    link.set_vibration(70, now=5.0)
    assert link.lines(5.1)[1] == 'vibe 70'
    assert link.lines(5.7)[1] == 'vibe 0'
    link.set_vibration(500, now=6.0)
    assert link.lines(6.0)[1] == 'vibe 100'


def test_test_power_still_obeys_stops():
    link = ready_link()
    link.set_test_power(100, 100, now=1.0)
    assert link.lines(1.0)[0] == 'd 100 100'
    link.on_line('stop handle')
    assert link.lines(1.0)[0] == 'd 0 0'


def test_status_is_json_for_the_page():
    link = ready_link()
    link.set_speed(0.5, 0.0, now=1.0)
    link.lines(1.0)
    status = json.loads(link.status(connected=True))
    assert status['left_pct'] == 100 and status['ready'] and status['stops'] == []
    assert status['left_rpm'] == pytest.approx(63.7, abs=0.1)


def test_vibration_motor_is_named_only_for_the_second_motor():
    from apnvi_stage4.drive import DriveLink, vibe_line
    assert vibe_line(70) == 'vibe 70' and vibe_line(70, 1) == 'vibe 70'
    assert vibe_line(70, 2) == 'vibe 70 2' and vibe_line(0, 2) == 'vibe 0 2'
    assert vibe_line(70, 3) == 'vibe 70 3'                      # both motors (17 Nov)
    link = DriveLink()
    link.set_vibration(60, 1.0)
    link.set_vibration_motor(2)
    assert link.lines(1.1)[1] == 'vibe 60 2'
    link.set_vibration_motor(7)
    assert link.lines(1.2)[1] == 'vibe 60'
