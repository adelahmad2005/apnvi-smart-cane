"""The guide's decisions: goals, red and green presses, follow-ups, arrival."""

from apnvi_stage4.guide import (ARRIVED, CLOSE, CONTINUING, Goal, Guide, NOTHING_TO_CONTINUE,
                                STOPPING)

RESTROOM = Goal(10.0, 0.0, 'restroom')
DOOR = Goal(0.0, 5.0, 'door')


def guiding():
    guide = Guide()
    guide.moved(0.0, 0.0)
    guide.new_goal(RESTROOM)
    return guide


def test_a_goal_starts_guiding_and_says_so():
    guide = Guide()
    guide.moved(0.0, 0.0)
    assert guide.new_goal(RESTROOM) == [('follow', RESTROOM), ('say', 'Taking you to restroom')]
    assert guide.state == 'guiding'


def test_replans_every_tick_only_while_guiding():
    guide = guiding()
    assert guide.tick() == [('follow', RESTROOM)]
    guide.talk_pressed()
    assert guide.tick() == []


def test_arrival_within_half_a_metre():
    guide = guiding()
    assert guide.moved(9.0, 0.0) == []
    assert guide.moved(9.6, 0.0) == [('cancel',), ('say', ARRIVED)]
    assert guide.state == 'idle' and guide.goal is None
    assert guide.moved(9.8, 0.0) == []                     # said once only


def test_follower_success_is_arrival():
    guide = guiding()
    guide.moved(9.4, 0.0)                                   # 0.6 m: inside Nav2's check
    assert guide.follower_finished(True) == [('cancel',), ('say', ARRIVED)]
    other = guiding()
    assert other.follower_finished(False) == [('say', CLOSE)] and other.state == 'guiding'


def test_follower_giving_up_is_said_once_until_the_device_moves():
    # 19 Nov demo: 490 silent "collision ahead" stops
    guide = guiding()
    assert guide.follower_finished(False) == [('say', CLOSE)]
    assert guide.follower_finished(False) == []             # same stop: said once
    guide.moved(0.3, 0.0)
    assert guide.follower_finished(False) == []             # moved only 0.3 m
    guide.moved(0.6, 0.0)
    assert guide.follower_finished(False) == [('say', CLOSE)]


def test_follower_giving_up_while_the_hazard_gate_stops_is_not_said_twice():
    guide = guiding()
    assert guide.follower_finished(False, blocked=True) == []
    assert guide.follower_finished(False) == [('say', CLOSE)]


def test_a_new_trip_may_say_close_again():
    guide = guiding()
    guide.follower_finished(False)
    guide.new_goal(DOOR)
    assert guide.follower_finished(False) == [('say', CLOSE)]


def test_follower_giving_up_when_not_guiding_says_nothing():
    guide = guiding()
    guide.talk_pressed()
    assert guide.follower_finished(False) == []


def test_follower_success_far_from_the_goal_is_not_arrival():
    # 15 Nov: the follower once said "reached" the moment it got a 1.5 m route
    guide = guiding()                                       # at 0 0, goal 10 m away
    assert guide.follower_finished(True) == []
    assert guide.state == 'guiding' and guide.tick() == [('follow', RESTROOM)]


def test_follower_success_with_no_position_is_still_arrival():
    guide = Guide()
    guide.new_goal(RESTROOM)
    assert guide.follower_finished(True) == [('cancel',), ('say', ARRIVED)]


def test_goal_already_reached():
    guide = Guide()
    guide.moved(10.2, 0.0)
    assert guide.new_goal(RESTROOM) == [('cancel',), ('say', ARRIVED)]


def test_red_while_guiding_stops_at_once_and_pauses():
    guide = guiding()
    assert guide.talk_pressed() == [('cancel',)]       # Stage 2 says "Stopping. Listening"
    assert guide.state == 'paused' and guide.goal == RESTROOM


def test_red_while_idle_does_nothing():
    assert Guide().talk_pressed() == []


def test_green_while_guiding_stops_and_says_stopping():
    guide = guiding()
    assert guide.emergency_on() == [('cancel',), ('say', STOPPING)]
    assert guide.state == 'paused'


def test_follow_up_after_red_carries_on():
    guide = guiding()
    guide.moved(4.0, 0.0)
    guide.talk_pressed()
    actions = guide.follow_up()
    assert actions[0] == ('follow', RESTROOM)
    assert actions[1] == ('say', CONTINUING + ', about 6 metres to go')
    assert guide.state == 'guiding'


def test_follow_up_waits_while_green_is_on():
    guide = guiding()
    guide.emergency_on()
    actions = guide.follow_up(green_on=True)
    assert actions[0][0] == 'say' and guide.state == 'paused'


def test_new_goal_after_red_wins():
    guide = guiding()
    guide.talk_pressed()
    assert guide.new_goal(DOOR)[0] == ('follow', DOOR)
    assert guide.goal == DOOR and guide.state == 'guiding'


def test_other_question_after_red_drops_the_route():
    guide = guiding()
    guide.talk_pressed()
    assert guide.other_question() == [('cancel',)]
    assert guide.state == 'idle' and guide.goal is None


def test_other_question_while_idle_changes_nothing():
    assert Guide().other_question() == []


def test_unknown_place_after_red_drops_the_route_and_says_so():
    guide = guiding()
    guide.talk_pressed()
    actions = guide.unknown_place('kitchen')
    assert actions == [('cancel',), ('say', "I don't know where kitchen is")]
    assert guide.state == 'idle'


def test_follow_up_with_nothing_to_continue():
    assert Guide().follow_up() == [('say', NOTHING_TO_CONTINUE)]


def test_how_far_while_guiding():
    guide = guiding()
    guide.moved(9.0, 0.0)
    assert guide.follow_up() == [('say', 'About 1 metre to go')]


def test_summary_for_the_page():
    guide = guiding()
    summary = guide.summary()
    assert summary['state'] == 'guiding' and summary['goal']['name'] == 'restroom'
    assert summary['distance_m'] == 10.0


# ---- goals that are things (16 Nov) ----

def test_arriving_at_a_thing_says_where_it_is():
    import math
    guide = Guide()
    guide.moved(0.0, 0.0, 0.0)
    guide.new_goal(Goal(4.2, 0.0, 'the water bottle', thing=(5.0, 0.0)))
    assert guide.moved(2.0, 0.0, 0.0) == []
    actions = guide.moved(4.0, 0.0, 0.0)
    words = ARRIVED + '. The water bottle is ahead of you, about 1 metre'
    assert actions == [('cancel',), ('say', words)]
    assert guide.state == 'idle'
    guide.moved(0.0, 0.0, 0.0)
    guide.new_goal(Goal(4.2, 0.0, 'the door', thing=(5.0, 0.0)))
    said = guide.moved(4.2, 0.0, -math.pi / 2)[1][1]
    assert said.endswith('The door is on your left, about 1 metre')


def test_where_words():
    import math
    from apnvi_stage4.guide import where_words
    assert where_words('the bin', (0, 0), 0.0, (0.4, 0.0)) == \
        'The bin is ahead of you, less than a metre'
    assert where_words('the bin', (0, 0), 0.0, (2.0, -2.0)) == \
        'The bin is ahead on your right, about 3 metres'
    assert where_words('the bin', (0, 0), math.pi, (2.0, 0.0)) == \
        'The bin is behind you, about 2 metres'
    assert where_words('the bin', (0, 0), None, (2.0, 0.0)) == 'The bin is about 2 metres away'


def test_looking_for_a_thing_and_the_reasons_it_was_not_placed():
    from apnvi_stage4.guide import LOOKING, NOT_LOCATED
    guide = Guide()
    assert guide.looking_for('water bottle') == [('say', LOOKING % 'water bottle')]
    assert guide.not_located('water bottle', 'not_seen') == [
        ('say', "I can't see the water bottle and it isn't on the map. "
                'Turn slowly and ask again')]
    assert guide.not_located('door', 'no_helper') == [('say', NOT_LOCATED['no_helper'])]
    assert guide.not_located('door', 'something new')[0][1].startswith("I can't see the door")
    guide.moved(0.0, 0.0)
    guide.new_goal(Goal(5.0, 0.0, 'the lab'))
    guide.talk_pressed()
    assert guide.looking_for('door')[0] == ('cancel',)      # a new question after red


def test_says_why_it_is_waiting_once_per_hold_up():
    from apnvi_stage4.guide import BLOCKED, NO_WAY
    guide = Guide()
    assert guide.waiting(0.0, True, False) == []            # idle: nothing to say
    guide.moved(0.0, 0.0)
    guide.new_goal(Goal(5.0, 0.0, 'the lab'))
    assert guide.waiting(1.0, True, True) == []
    assert guide.waiting(3.5, True, True) == []             # BLOCKED_SAY_S is 3 s (23 Nov)
    assert guide.waiting(4.5, True, True) == [('say', BLOCKED)]
    assert guide.waiting(8.5, True, True) == []             # said once
    assert guide.waiting(10.0, False, True) == []           # it moved on
    assert guide.waiting(11.0, True, True) == []
    assert guide.waiting(16.0, True, None) == [('say', BLOCKED)]    # a new hold-up
    assert guide.waiting(20.0, False, False) == []
    assert guide.waiting(27.0, False, False) == [('say', NO_WAY)]
    assert guide.waiting(30.0, False, False) == []


def test_asks_to_be_turned_when_still_blocked():
    from apnvi_stage4.guide import BLOCKED, STILL_BLOCKED
    guide = Guide()
    guide.moved(0.0, 0.0)
    guide.new_goal(Goal(5.0, 0.0, 'the lab'))
    assert guide.waiting(0.0, True, True) == []
    assert guide.waiting(5.0, True, True) == [('say', BLOCKED)]
    assert guide.waiting(7.0, True, True) == []             # STILL_BLOCKED_S is 8 s (23 Nov)
    assert guide.waiting(9.0, True, True) == [('say', STILL_BLOCKED)]
    assert guide.waiting(40.0, True, True) == []            # said once
    assert guide.waiting(41.0, False, True) == []           # it moved on
    assert guide.waiting(42.0, True, True) == []
    assert guide.waiting(47.0, True, True) == [('say', BLOCKED)]    # a new hold-up, all again
    assert guide.waiting(58.0, True, True) == [('say', STILL_BLOCKED)]


def test_a_trip_waits_until_the_device_is_placed():
    # 17 Nov: before the device has found itself on the map its position is wrong; the trip
    # waits (no route) and starts by itself once it is found.
    guide = Guide()
    guide.placement(False)
    actions = guide.new_goal(Goal(4.0, 0.0, 'the door'))
    assert ('cancel',) in actions and not any(a[0] == 'follow' for a in actions)
    assert 'once I know where you are' in actions[-1][1]
    assert guide.tick() == []
    assert guide.moved(4.0, 0.0) == []                      # a wrong position is no arrival
    assert guide.state == 'guiding'
    actions = guide.placement(True)
    assert actions[0] == ('follow', guide.goal) and actions[1] == ('say', 'Taking you to the door')
    assert guide.tick() == [('follow', guide.goal)]


def test_losing_the_place_holds_a_trip():
    guide = Guide()
    guide.new_goal(Goal(4.0, 0.0, 'the door'))
    assert guide.placement(False) == [('cancel',)]
    assert guide.tick() == []
    assert guide.placement(False) == []                     # said once
    assert guide.placement(True)[0][0] == 'follow'


def test_free_mode_steers_while_a_trip_is_paused_or_waiting_for_its_place():
    # 18 Nov: steering went off while the user pushed with a trip paused or waiting
    from apnvi_stage4.free_mode import steering_allowed
    guide = guiding()
    assert not steering_allowed(guide.summary())            # the device drives itself
    guide.talk_pressed()
    assert steering_allowed(guide.summary())                # paused by red
    guide.placement(False)
    guide.follow_up()
    assert guide.state == 'guiding' and steering_allowed(guide.summary())   # waiting
    assert steering_allowed(Guide().summary())              # no trip
