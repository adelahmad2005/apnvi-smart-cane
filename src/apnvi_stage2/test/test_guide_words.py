"""Stage 4: "take me to ..." and follow-ups are sorted by their words, before Qwen."""

from apnvi_stage2.asking import (answer_typed, guide_place, is_follow_up, LISTENING, Outcome,
                                 request_for, STILL_THINKING)
from apnvi_stage2.stage2_node import press_words


def never(*args, **kwargs):
    raise AssertionError('Qwen must not be asked')


def test_take_me_to_gives_the_place():
    assert guide_place('take me to the restroom') == 'restroom'
    assert guide_place('Please, take me to the Restroom!') == 'restroom'
    assert guide_place('could you guide me to room 117') == 'room 117'
    assert guide_place('go to the exit') == 'exit'


def test_other_questions_are_not_guide_requests():
    assert guide_place('where is the restroom') is None
    assert guide_place('take me to') is None
    assert guide_place('what is in front of me') is None


def test_follow_ups_are_whole_short_questions():
    for words in ('continue', 'Keep going.', 'okay carry on please', 'How far?',
                  'are we there yet'):
        assert is_follow_up(words), words
    assert not is_follow_up('how far is the chair')     # a normal question for Qwen
    assert not is_follow_up('continue to the exit and tell me')


def test_guide_and_follow_up_skip_qwen_and_the_camera():
    outcome = answer_typed('take me to the restroom', None, 'url', 'model', finder=never,
                           router=never, replier=never)
    assert (outcome.kind, outcome.place, outcome.say) == ('guide', 'restroom', '')
    outcome = answer_typed('continue', None, 'url', 'model', finder=never, router=never,
                           replier=never)
    assert (outcome.kind, outcome.say) == ('follow_up', '')


def test_request_for_the_guide():
    assert request_for(Outcome('take me to the door', kind='guide', place='door')) == {
        'kind': 'guide', 'place': 'door', 'question': 'take me to the door'}
    assert request_for(Outcome('continue', kind='follow_up'))['kind'] == 'follow_up'
    assert request_for(Outcome('what is this', 'A chair', kind='answer'))['kind'] == 'other'
    assert request_for(Outcome(say='nothing heard')) is None    # nothing was asked


def test_press_words_while_guided():
    assert press_words(LISTENING, False) == LISTENING
    assert press_words(LISTENING, True) == 'Stopping. Listening'
    assert press_words(STILL_THINKING, True) == 'Stopping. Still thinking'
