"""
Tests for asking.py and the no-ROS helpers in stage2_node.py, with fake helpers.

No campus PC, no microphone, no camera. A fake 'arecord' stands in for the microphone.
"""

import json
import os
import stat
import time

from apnvi_stage2.asking import (
    answer_recording, answer_typed, NO_CAMERA, NO_HELPER, NOT_FOUND, NOT_HEARD)
from apnvi_stage2.finder import FindResult, RouteResult
from apnvi_stage2.speech import SpeechResult
from apnvi_stage2.stage2_node import (bgr_to_image, image_to_bgr, Recorder, result_json,
                                      waiting_words)
import numpy as np
from sensor_msgs.msg import Image

URL, MODEL = 'http://pc:11434', 'qwen3-vl:4b'


def photo():
    """Make a plain grey photo."""
    return np.full((480, 640, 3), 128, dtype=np.uint8)


def finder_saying(result):
    """Make a fake finder that always gives this result and remembers the question."""
    def fake(image, question, url, model):
        fake.asked = question
        return result
    fake.asked = None
    return fake


def speech_saying(text='', error=''):
    """Make a fake speech-to-text that always gives these words."""
    return lambda wav, url: SpeechResult(text, 0.2, error)


# ---- Typed questions ----

def test_found_is_silent_with_red_box():
    outcome = answer_typed('where is the exit', photo(), URL, MODEL,
                           finder_saying(FindResult(True, (10, 10, 50, 50), 'exit sign')))
    assert outcome.say == ''
    assert tuple(outcome.picture[10, 30]) == (0, 0, 255)


def test_an_unread_sign_is_not_the_named_thing():
    # 18 Nov: "where is ESB117" found only "sign", and nothing was said
    outcome = answer_typed('where is ESB117', photo(), URL, MODEL,
                           finder_saying(FindResult(True, (10, 10, 50, 50), 'sign')))
    assert outcome.say == NOT_FOUND
    outcome = answer_typed('where is the sign', photo(), URL, MODEL,
                           finder_saying(FindResult(True, (10, 10, 50, 50), 'sign')))
    assert outcome.say == ''


def test_not_found_says_turn_slowly():
    outcome = answer_typed('where is the exit', photo(), URL, MODEL,
                           finder_saying(FindResult(False)))
    assert outcome.say == NOT_FOUND == "I can't see it, turn slowly and try again"


def test_nonsense_answer_counts_as_not_found():
    result = FindResult(False, error='could not understand the answer')
    assert answer_typed('where is it', photo(), URL, MODEL, finder_saying(result)).say == NOT_FOUND


def test_pc_unreachable_says_so():
    result = FindResult(False, error='cannot reach Qwen at http://pc:11434: refused')
    assert answer_typed('where is it', photo(), URL, MODEL, finder_saying(result)).say == NO_HELPER


def test_no_camera_picture_yet():
    finder = finder_saying(FindResult(True, (1, 1, 2, 2)))
    assert answer_typed('where is it', None, URL, MODEL, finder).say == NO_CAMERA
    assert finder.asked is None                     # never bothered the PC


def test_empty_typed_question():
    assert answer_typed('   ', photo(), URL, MODEL, finder_saying(FindResult(True))).say == \
        NOT_HEARD


# ---- Spoken questions ----

def test_spoken_question_is_passed_to_finder():
    finder = finder_saying(FindResult(False))
    outcome = answer_recording(b'wav', photo(), 'http://pc:8090', URL, MODEL,
                               speech_saying('where is ESB2090'), finder)
    assert finder.asked == 'where is ESB2090' and outcome.question == 'where is ESB2090'


def test_nothing_heard():
    outcome = answer_recording(b'wav', photo(), 'http://pc:8090', URL, MODEL,
                               speech_saying(''), finder_saying(FindResult(True)))
    assert outcome.say == NOT_HEARD


def test_speech_server_down():
    outcome = answer_recording(b'wav', photo(), 'http://pc:8090', URL, MODEL,
                               speech_saying(error='cannot reach the speech server'),
                               finder_saying(FindResult(True)))
    assert outcome.say == NO_HELPER


# ---- Pictures in and out of ROS ----

def test_rgb_picture_becomes_bgr_and_back():
    msg = Image()
    msg.height, msg.width, msg.encoding = 2, 3, 'rgb8'
    msg.step = 3 * 3 + 4                           # rows may have padding at the end
    rows = [bytes([255, 0, 0] * 3) + b'\0' * 4] * 2   # pure red, in red-green-blue order
    msg.data = b''.join(rows)
    picture = image_to_bgr(msg)
    assert picture.shape == (2, 3, 3) and tuple(picture[0, 0]) == (0, 0, 255)
    back = bgr_to_image(picture)
    assert back.encoding == 'bgr8' and back.step == 9
    assert bytes(back.data)[:3] == bytes([0, 0, 255])


def test_result_json():
    from apnvi_stage2.asking import Outcome
    outcome = Outcome('where is the exit', '', FindResult(True, (1, 2, 3, 4), 'exit', 1.234))
    data = json.loads(result_json(outcome))
    assert data['found'] is True and data['box'] == [1, 2, 3, 4] and data['seconds'] == 1.23


# ---- Recording, with a fake arecord ----

FAKE_ARECORD = """#!/usr/bin/env python3
import signal, sys, time
path = sys.argv[-1]
with open(path, 'wb') as f:
    f.write(b'RIFF' + b'\\0' * 40)                  # 44-byte header
def finish(*_):
    with open(path, 'ab') as f:
        f.write(b'sound' * 100)
    sys.exit(0)
signal.signal(signal.SIGINT, finish)
while True:
    time.sleep(0.05)
"""


def test_recorder_gets_the_sound_after_letting_go(tmp_path, monkeypatch):
    fake = tmp_path / 'arecord'
    fake.write_text(FAKE_ARECORD)
    fake.chmod(fake.stat().st_mode | stat.S_IEXEC)
    monkeypatch.setenv('PATH', f'{tmp_path}{os.pathsep}{os.environ["PATH"]}')
    recorder = Recorder()
    recorder.start()
    time.sleep(0.5)
    path = recorder.path
    wav, held = recorder.stop()
    assert wav is not None and wav.endswith(b'sound') and held >= 0.4
    assert not os.path.exists(path)                 # the temporary file is cleaned up


# ---- Campus PC setting ----

def test_result_json_carries_the_picture_time():
    from apnvi_stage2.asking import Outcome
    from apnvi_stage2.stage2_node import result_json
    data = json.loads(result_json(Outcome('q', '', FindResult(False, answer='[]')), stamp=12.5))
    assert data['stamp'] == 12.5 and data['size'] is None and data['answer'] == '[]'


def test_urls_for_pc():
    from apnvi_stage2.stage2_node import urls_for_pc
    assert urls_for_pc('10.1.2.3') == ('http://10.1.2.3:11434', 'http://10.1.2.3:8090')


def test_voice_url_for_pc():
    from apnvi_stage2.stage2_node import voice_url_for_pc
    assert voice_url_for_pc('10.1.2.3') == 'http://10.1.2.3:8091'


def test_front_door_addresses():
    """pc:= can be the PC's front door over the internet (remote link)."""
    from apnvi_stage2.stage2_node import urls_for_pc, voice_url_for_pc
    door = 'https://abc.trycloudflare.com/k/TOKEN'
    assert urls_for_pc(door + '/') == (door + '/ai', door + '/speech')
    assert voice_url_for_pc(door) == door + '/voice'


def test_the_token_is_hidden_in_the_log():
    from apnvi_stage2.stage2_node import hide_token
    assert hide_token('https://abc.trycloudflare.com/k/SECRET/ai') == \
        'https://abc.trycloudflare.com/k/***/ai'
    assert hide_token('https://abc.trycloudflare.com/k/SECRET') == \
        'https://abc.trycloudflare.com/k/***'
    assert hide_token('http://10.1.2.3:11434') == 'http://10.1.2.3:11434'


def test_waiting_words_still_thinking_twice_then_too_long():
    from apnvi_stage2.asking import STILL_THINKING, TOO_LONG
    assert waiting_words(1.0, 0) == (None, 0)
    assert waiting_words(4.2, 0) == (STILL_THINKING, 1)
    assert waiting_words(6.0, 1) == (None, 1)
    assert waiting_words(10.5, 1) == (STILL_THINKING, 2)
    assert waiting_words(20.0, 2) == (None, 2)
    assert waiting_words(30.0, 2) == (TOO_LONG, 3)
    assert waiting_words(45.0, 3) == (None, 3)                 # said once only


# ---- Sorting the question: find or answer in words (26 Oct) ----

def router_saying(kind='find', error=''):
    """Make a fake router that always sorts the question this way."""
    return lambda question, url, model: RouteResult(kind, 0.3, error)


def test_answer_question_speaks_qwens_sentence_with_box():
    replier = finder_saying(FindResult(True, (10, 10, 50, 50), 'sign', 1.0,
                                       say='Restroom, arrow pointing left'))
    finder = finder_saying(FindResult(True, (1, 1, 2, 2)))
    outcome = answer_typed('what is the nearest sign', photo(), URL, MODEL, finder,
                           router_saying('answer'), replier)
    assert outcome.say == 'Restroom, arrow pointing left' and outcome.kind == 'answer'
    assert tuple(outcome.picture[10, 30]) == (0, 0, 255)
    assert finder.asked is None and replier.asked == 'what is the nearest sign'
    assert abs(outcome.result.seconds - 1.3) < 1e-9         # sorting time is counted


def test_answer_without_box_speaks_and_draws_nothing():
    replier = finder_saying(FindResult(False, say='Two chairs ahead of you.'))
    outcome = answer_typed('how many chairs', photo(), URL, MODEL,
                           finder_saying(FindResult(False)), router_saying('answer'), replier)
    assert outcome.say == 'Two chairs ahead of you.'
    assert tuple(outcome.picture[10, 30]) == (128, 128, 128)


def test_find_question_stays_silent_after_sorting():
    finder = finder_saying(FindResult(True, (10, 10, 50, 50), 'water bottle'))
    outcome = answer_typed('where is a water bottle', photo(), URL, MODEL, finder,
                           router_saying('find'), finder_saying(FindResult(False, say='x')))
    assert outcome.say == '' and outcome.kind == 'find' and finder.asked


def test_answer_nobody_understood_says_not_found():
    outcome = answer_typed('read the sign', photo(), URL, MODEL,
                           finder_saying(FindResult(False)), router_saying('answer'),
                           finder_saying(FindResult(False, error='could not understand')))
    assert outcome.say == NOT_FOUND


def test_answer_with_pc_off_says_so():
    outcome = answer_typed('read the sign', photo(), URL, MODEL,
                           finder_saying(FindResult(False)), router_saying('answer'),
                           finder_saying(FindResult(False, error='cannot reach Qwen')))
    assert outcome.say == NO_HELPER


def test_sorting_with_pc_off_says_so_and_asks_nothing_else():
    finder = finder_saying(FindResult(True))
    outcome = answer_typed('read the sign', photo(), URL, MODEL, finder,
                           router_saying('find', 'cannot reach Qwen at http://pc:11434'))
    assert outcome.say == NO_HELPER and finder.asked is None


def test_spoken_question_is_sorted_too():
    replier = finder_saying(FindResult(False, say='Exit, straight ahead.'))
    outcome = answer_recording(b'wav', photo(), 'http://pc:8090', URL, MODEL,
                               speech_saying('read the sign'), finder_saying(FindResult(False)),
                               router_saying('answer'), replier)
    assert outcome.say == 'Exit, straight ahead.'


def test_result_json_says_which_kind():
    from apnvi_stage2.asking import Outcome
    data = json.loads(result_json(Outcome('read the sign', 'Exit.', FindResult(False, say='Exit.'),
                                          kind='answer')))
    assert data['kind'] == 'answer' and data['said'] == 'Exit.'


def test_heard_is_called_with_the_words_before_qwen():
    order = []
    finder = finder_saying(FindResult(False))
    answer_recording(b'wav', photo(), 'http://pc:8090', URL, MODEL,
                     speech_saying('where is the exit'), finder,
                     on_heard=lambda words: order.append(('heard', words, finder.asked)))
    assert order == [('heard', 'where is the exit', None)]     # before the finder was asked


def test_heard_is_not_called_when_nothing_was_heard():
    order = []
    answer_recording(b'wav', photo(), 'http://pc:8090', URL, MODEL, speech_saying(''),
                     finder_saying(FindResult(False)), on_heard=order.append)
    assert order == []


# ---- map commands by voice (16 Nov) ----

def test_map_commands_are_sorted_by_their_words():
    from apnvi_stage2.asking import spoken_command
    assert spoken_command('Save this place as my desk.') == {'cmd': 'save_place',
                                                             'name': 'my desk'}
    assert spoken_command('please scan the room') == {'cmd': 'scan_room'}
    assert spoken_command('Forget the bin from the map') == {'cmd': 'remove_pin', 'name': 'bin'}
    assert spoken_command('save this map as ESB 117') == {'cmd': 'save_map', 'name': 'esb 117'}
    assert spoken_command('load the map called lab') == {'cmd': 'load_map', 'name': 'lab'}
    assert spoken_command('start a new map') == {'cmd': 'new_map'}
    assert spoken_command('where is the exit') is None
    assert spoken_command('save this place as') is None


def test_a_map_command_never_asks_qwen():
    from apnvi_stage2.asking import answer_typed, request_for

    def no_finder(*_a, **_k):
        raise AssertionError('Qwen must not be asked')

    outcome = answer_typed('remember this place as the printer', None, 'url', 'model',
                           finder=no_finder, router=no_finder)
    assert outcome.kind == 'command' and outcome.say == ''
    assert outcome.command == {'cmd': 'save_place', 'name': 'the printer'}
    assert request_for(outcome)['kind'] == 'other'


# ---- "where is X" when X is on the map but not in view (16 Nov) ----

def test_thing_asked():
    from apnvi_stage2.asking import thing_asked
    assert thing_asked('Where is my water bottle?') == 'water bottle'
    assert thing_asked('find the door') == 'door'
    assert thing_asked('what is in front of me') == ''


def test_on_the_map_answers_from_the_pins():
    from apnvi_stage2.asking import on_the_map
    pins = [{'name': 'door', 'x': 4.0, 'y': 0.0}, {'name': 'door', 'x': 9.0, 'y': 0.0},
            {'name': 'water bottle', 'x': 0.0, 'y': 2.0}]
    words = on_the_map('where is the door', pins, (3.0, 0.0, 0.0))
    assert words == ("I can't see it from here, but the door is on the map, about 1 metre "
                     'away. Say take me to the door')
    assert 'about 2 metres away' in on_the_map('where is the bottle', pins, (0.0, 0.0, 0.0))
    assert on_the_map('where is the kitchen', pins, (0.0, 0.0, 0.0)) == ''
    assert on_the_map('read the sign', pins, (0.0, 0.0, 0.0)) == ''
    assert on_the_map('where is the door', pins, None).endswith('Say take me to the door')
    assert on_the_map('where is the door', [], (0.0, 0.0, 0.0)) == ''
