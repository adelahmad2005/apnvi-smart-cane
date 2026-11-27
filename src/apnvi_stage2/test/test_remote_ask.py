"""
Tests for remote_ask.py (remote link): a whole question in one trip.

Fake speech, sorting and Qwen; a small local server stands in for the PC's front door.
No ROS, no campus PC, no network.

On the Jetson: python3 -m pytest test/test_remote_ask.py
On the PC (no pytest): python test/test_remote_ask.py
"""

import base64
from http.server import BaseHTTPRequestHandler, HTTPServer
import json
import os
import sys
import tempfile
import threading

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
from apnvi_stage2.asking import NO_HELPER, NOT_FOUND, answer_typed  # noqa: E402
from apnvi_stage2.finder import draw_box, FindResult, RouteResult  # noqa: E402
from apnvi_stage2.remote_ask import (answer_on_pc, ask_front_door, NoAsk,  # noqa: E402
                                     outcome_from_dict, outcome_to_dict, photo_from_text,
                                     photo_to_text)
from apnvi_stage2.speech import SpeechResult  # noqa: E402
from apnvi_stage2.voice import Voice  # noqa: E402

BOX = (100, 80, 220, 300)


def photo():
    """Make a 640 x 480 test photo with some shapes on it."""
    image = np.zeros((480, 640, 3), np.uint8)
    image[50:200, 60:300] = (40, 160, 220)
    image[250:400, 350:600] = (200, 60, 30)
    return image


def finder_found(image, question, url=None, model=None):
    return FindResult(True, BOX, 'water bottle', 0.4, answer='[{"bbox_2d": [1, 2, 3, 4]}]',
                      boxes=(BOX, (1, 2, 3, 4)))


def finder_missing(image, question, url=None, model=None):
    return FindResult(False, label='', seconds=0.3, answer='[]')


def router_find(question, url=None, model=None):
    return RouteResult('find', 0.1)


def router_answer(question, url=None, model=None):
    return RouteResult('answer', 0.1)


def replier_says(image, question, url=None, model=None):
    return FindResult(True, BOX, 'desk', 0.5, say='A water bottle is on the desk, to your left.')


def speech_saying(words):
    return lambda wav, url=None: SpeechResult(words, 0.2)


# ---- the outcome there and back ----

def test_photo_goes_and_comes_back_full_size():
    image = photo()
    back = photo_from_text(photo_to_text(image))
    assert back.shape == image.shape
    assert np.abs(back.astype(int) - image.astype(int)).mean() < 2      # JPEG at 95
    assert photo_to_text(None) is None and photo_from_text(None) is None


def test_found_comes_back_with_the_box_drawn_on_the_jetsons_photo():
    image = photo()
    outcome = answer_typed('where is the water bottle', image, 'u', 'm', finder=finder_found)
    data = json.loads(json.dumps(outcome_to_dict(outcome, image)))      # as it travels
    assert data['picture'] == 'boxed'
    back = outcome_from_dict(data, image)
    assert back.result == outcome.result and back.say == outcome.say == ''
    assert back.question == 'where is the water bottle' and back.kind == 'find'
    assert np.array_equal(back.picture, draw_box(image, BOX, 'water bottle'))


def test_not_found_comes_back_with_the_plain_photo():
    image = photo()
    outcome = answer_typed('where is the cat', image, 'u', 'm', finder=finder_missing)
    data = outcome_to_dict(outcome, image)
    assert data['picture'] == 'photo' and data['say'] == NOT_FOUND
    back = outcome_from_dict(json.loads(json.dumps(data)), image)
    assert back.picture is image and back.say == NOT_FOUND and not back.result.found


def test_guide_and_command_questions_come_back_unchanged():
    image = photo()
    for question in ('take me to my desk', 'save this place as my desk', 'keep going'):
        outcome = answer_typed(question, image, 'u', 'm', finder=finder_found)
        back = outcome_from_dict(json.loads(json.dumps(outcome_to_dict(outcome, image))), image)
        assert (back.kind, back.place, back.command, back.picture, back.question) == (
            outcome.kind, outcome.place, outcome.command, None, outcome.question), question


# ---- the PC's side ----

def test_pc_answers_a_recording_and_adds_the_voice():
    asked = []

    def voice(sentence, squeezed):
        asked.append((sentence, squeezed))
        return b'OggS sound'

    request = {'wav': base64.b64encode(b'RIFF rec').decode(), 'photo': photo_to_text(photo()),
               'model': 'm', 'voice': 'ogg'}
    answer = answer_on_pc(request, 'speech', 'ai', voice, speech=speech_saying('what is that'),
                          router=router_answer, replier=replier_says)
    assert answer['question'] == 'what is that' and answer['kind'] == 'answer'
    assert answer['say'] == 'A water bottle is on the desk, to your left.'
    assert asked == [(answer['say'], True)]
    assert base64.b64decode(answer['voice']) == b'OggS sound'


def test_pc_adds_no_voice_when_not_asked_or_nothing_to_say():
    def voice(sentence, squeezed):
        raise AssertionError('the voice was not asked for')

    request = {'text': 'where is the water bottle', 'photo': photo_to_text(photo())}
    answer = answer_on_pc(request, 's', 'a', voice, router=router_find, finder=finder_found)
    assert 'voice' not in answer and answer['picture'] == 'boxed'
    request['voice'] = 'ogg'                     # found: nothing said here, so no voice
    answer = answer_on_pc(request, 's', 'a', voice, router=router_find, finder=finder_found)
    assert 'voice' not in answer and answer['say'] == ''


# ---- the Jetson's side, against a pretend front door ----

class Door:
    """A local server standing in for the front door's /ask (or a door without one)."""

    def __init__(self, has_ask=True, voice=b'OggS sound'):
        door = self

        class Handler(BaseHTTPRequestHandler):
            def do_POST(self):  # noqa: N802 (name fixed by http.server)
                body = json.loads(self.rfile.read(int(self.headers['Content-Length'])))
                door.requests.append((self.path, body))
                if not has_ask:
                    data, code = b'{"error": "not found"}', 404
                else:
                    answer = answer_on_pc(body, 's', 'a', lambda say, squeezed: voice,
                                          speech=speech_saying('what is that'),
                                          router=router_answer, replier=replier_says)
                    data, code = json.dumps(answer).encode(), 200
                self.send_response(code)
                self.send_header('Content-Length', str(len(data)))
                self.end_headers()
                self.wfile.write(data)

            def log_message(self, *_):
                pass

        self.requests = []
        self.server = HTTPServer(('127.0.0.1', 0), Handler)
        threading.Thread(target=self.server.serve_forever, daemon=True).start()
        self.url = 'http://127.0.0.1:%d/k/TOKEN' % self.server.server_port

    def close(self):
        self.server.shutdown()
        self.server.server_close()


def test_jetson_gets_answer_and_unpacked_voice_in_one_trip():
    door = Door()
    try:
        unpacked = []

        def unpack(data):
            unpacked.append(data)
            return b'RIFF voice'

        image = photo()
        outcome, sound = ask_front_door(door.url, image, 'm', wav=b'RIFF rec', voice=True,
                                        can_unpack=True, unpack=unpack)
        assert len(door.requests) == 1                                  # one trip
        path, body = door.requests[0]
        assert path == '/k/TOKEN/ask' and body['voice'] == 'ogg' and body['model'] == 'm'
        assert base64.b64decode(body['wav']) == b'RIFF rec' and 'text' not in body
        assert outcome.say == 'A water bottle is on the desk, to your left.'
        assert outcome.kind == 'answer' and outcome.picture.shape == image.shape
        assert unpacked == [b'OggS sound'] and sound == b'RIFF voice'
    finally:
        door.close()


def test_jetson_without_unpacking_asks_for_wav_and_typed_words_go_as_text():
    door = Door(voice=b'RIFF plain')
    try:
        outcome, sound = ask_front_door(door.url, photo(), 'm', text='what is that',
                                        voice=True, can_unpack=False)
        body = door.requests[0][1]
        assert body['voice'] == 'wav' and body['text'] == 'what is that' and 'wav' not in body
        assert sound == b'RIFF plain'
        _, sound = ask_front_door(door.url, photo(), 'm', text='what is that', voice=False)
        assert door.requests[1][1]['voice'] == '' and sound is None     # not asked: none sent
    finally:
        door.close()


def test_a_front_door_without_ask_means_four_trips():
    door = Door(has_ask=False)
    try:
        try:
            ask_front_door(door.url, photo(), 'm', text='hello')
        except NoAsk:
            pass
        else:
            raise AssertionError('NoAsk expected')
    finally:
        door.close()


def test_pc_out_of_reach_says_no_helper():
    outcome, sound = ask_front_door('http://127.0.0.1:1/k/TOKEN', photo(), 'm', text='hi',
                                    timeout_s=3)
    assert outcome.say == NO_HELPER and sound is None
    assert outcome.problem.startswith('cannot reach') and 'TOKEN' not in outcome.problem


# ---- the voice kept from the answer ----

def test_voice_keeps_a_sentence_that_came_with_its_answer():
    with tempfile.TemporaryDirectory() as folder:
        voice = Voice(engine='kokoro', folder=folder,
                      fetch=lambda url, sentence: (_ for _ in ()).throw(AssertionError('no trip')))
        assert voice.keep('Hello there.', b'RIFF voice')
        assert voice.sound_file('Hello there.') == voice.pc_file('Hello there.')    # no fetch
        with open(voice.pc_file('Hello there.'), 'rb') as file:
            assert file.read() == b'RIFF voice'
        assert not voice.keep('Bad.', b'OggS not unpacked')
        espeak = Voice(engine='espeak', folder=folder)
        assert not espeak.keep('Hello there.', b'RIFF voice')


if __name__ == '__main__':
    tests = [value for name, value in sorted(globals().items()) if name.startswith('test_')]
    for test in tests:
        test()
        print('passed', test.__name__)
    print('%d of %d passed' % (len(tests), len(tests)))
