"""
Tests for voice_server.py with a fake voice engine (no Kokoro, no graphics card, no network).

Run from the code folder:  python3 -m pytest campus_pc
"""

import io
import json
import os
import sys
import threading
import urllib.error
import urllib.request
import wave

import pytest

sys.path.insert(0, os.path.dirname(__file__))
from voice_server import say_digits, serve, speakable, to_wav  # noqa: E402 (path line above)


class FakeEngine:
    """Stands in for Kokoro: returns a small WAV, or fails when told to."""

    voice, speed, device = 'af_heart', 1.0, 'CPUExecutionProvider'
    voices = ['af_heart', 'bf_emma']

    def __init__(self, fail=False):
        self.fail, self.asked = fail, []

    def make(self, text, voice, speed):
        """Pretend to make the voice."""
        self.asked.append((text, voice, speed))
        if self.fail:
            raise RuntimeError('model broke')
        return b'RIFF fake wav ' + text.encode('utf-8')


@pytest.fixture
def start():
    """Start a voice server on a free port with the given engine; stop it afterwards."""
    servers = []

    def run(engine):
        server = serve(engine, port=0, host='127.0.0.1')
        threading.Thread(target=server.serve_forever, daemon=True).start()
        servers.append(server)
        return f'http://127.0.0.1:{server.server_port}'

    yield run
    for server in servers:
        server.shutdown()
        server.server_close()


def post(url, body):
    """Send body to url. Return (status, headers, raw reply bytes)."""
    data = body if isinstance(body, bytes) else json.dumps(body).encode('utf-8')
    request = urllib.request.Request(url, data=data,
                                     headers={'Content-Type': 'application/json'})
    try:
        with urllib.request.urlopen(request, timeout=5) as response:
            return response.status, response.headers, response.read()
    except urllib.error.HTTPError as problem:
        return problem.code, problem.headers, problem.read()


def test_sentence_comes_back_as_wav(start):
    engine = FakeEngine()
    status, headers, wav = post(start(engine) + '/speak', {'text': 'Listening'})
    assert status == 200 and wav.startswith(b'RIFF')
    assert headers['Content-Type'] == 'audio/wav'
    assert float(headers['X-Seconds']) >= 0
    assert engine.asked == [('Listening', 'af_heart', 1.0)]


def test_same_sentence_is_made_once(start):
    engine = FakeEngine()
    url = start(engine) + '/speak'
    first = post(url, {'text': 'Still thinking'})
    second = post(url, {'text': 'Still thinking'})
    assert first[2] == second[2] and len(engine.asked) == 1
    assert float(second[1]['X-Seconds']) == 0.0


def test_voice_and_speed_can_be_chosen(start):
    engine = FakeEngine()
    post(start(engine) + '/speak', {'text': 'Heard', 'voice': 'bf_emma', 'speed': 1.2})
    assert engine.asked == [('Heard', 'bf_emma', 1.2)]


def test_health_lists_the_voices(start):
    with urllib.request.urlopen(start(FakeEngine()) + '/health', timeout=5) as response:
        reply = json.loads(response.read())
    assert reply['ok'] is True and reply['voice'] == 'af_heart' and 'bf_emma' in reply['voices']


@pytest.mark.parametrize('body', [b'', b'not json', {'words': 'hi'}, {'text': '   '},
                                  {'text': 'x' * 601}, {'text': 'hi', 'speed': 5}])
def test_bad_requests_are_refused(start, body):
    engine = FakeEngine()
    status, _, reply = post(start(engine) + '/speak', body)
    assert status == 400 and 'error' in json.loads(reply) and engine.asked == []


def test_engine_error_is_reported_and_server_keeps_running(start):
    url = start(FakeEngine(fail=True))
    status, _, reply = post(url + '/speak', {'text': 'Heard'})
    assert status == 500 and 'model broke' in json.loads(reply)['error']
    with urllib.request.urlopen(url + '/health', timeout=5) as response:
        assert response.status == 200


def test_unknown_address(start):
    status, _, _ = post(start(FakeEngine()) + '/talk', {'text': 'Heard'})
    assert status == 404


def test_room_codes_are_said_like_people_say_them():
    assert speakable('The sign says ESB2090.') == 'The sign says E S B 20 90.'
    assert speakable('Room ESB 117 is on your left') == 'Room E S B 1 17 is on your left'
    assert say_digits('2005') == '20 oh 5'
    assert speakable('EXIT is ahead, about 2 metres') == 'EXIT is ahead, about 2 metres'
    assert speakable('  two   spaces ') == 'two spaces'


def test_to_wav_makes_a_real_wav_file():
    pytest.importorskip('numpy')
    data = to_wav([0.0, 0.25, -0.5, 0.1], 24000)
    with wave.open(io.BytesIO(data)) as file:
        assert (file.getnchannels(), file.getsampwidth(), file.getframerate()) == (1, 2, 24000)
        frames = file.readframes(4)
    loudest = max(abs(int.from_bytes(frames[i:i + 2], 'little', signed=True))
                  for i in range(0, 8, 2))
    assert loudest == int(0.9 * 32767)              # every file equally loud
