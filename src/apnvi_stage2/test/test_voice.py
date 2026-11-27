"""
Tests for voice.py: Stage 2's voice, with Stage 1 always first.

No sound hardware: a fake player stands in for paplay, a fake espeak-ng for making files.
"""

from http.server import BaseHTTPRequestHandler, HTTPServer
import json
import os
import threading
import time

from apnvi_stage2.voice import fetch_from_pc, FULL_VOLUME, Mouth, RETRY_S, Voice
import pytest

ASK_AGAIN = 'Please ask again'


class FakeProcess:
    """Stands in for a running paplay: playing until finished or terminated."""

    def __init__(self, command):
        self.command = command
        self.done = False

    def poll(self):
        return 0 if self.done else None

    def terminate(self):
        self.done = True


def fake_voice(tmp_path, quiet_volume=0.4, engine='espeak', fetch=None, clock=time.monotonic):
    """Make a Voice whose player and espeak-ng are fakes. .started lists what was played."""
    started = []

    def run(command, **_):
        started.append(FakeProcess(command))
        return started[-1]

    def make(command, **_):
        with open(command[command.index('-w') + 1], 'wb') as file:
            file.write(b'RIFF')

    voice = Voice(quiet_volume, run=run, make=make, folder=str(tmp_path), engine=engine,
                  url='http://pc:8091', fetch=fetch or FakePC(), clock=clock)
    voice.espeak, voice.player = '/usr/bin/espeak-ng', '/usr/bin/paplay'
    voice.started = started
    return voice


class FakePC:
    """Stands in for the campus PC's voice server. .asked lists the sentences it was sent."""

    def __init__(self, up=True):
        self.up, self.asked = up, []

    def __call__(self, url, sentence):
        self.asked.append(sentence)
        if not self.up:
            raise OSError('no route to host')
        return b'RIFF kokoro ' + sentence.encode('utf-8')


class FakeClock:
    """A clock the test moves by hand."""

    def __init__(self):
        self.now = 100.0

    def __call__(self):
        return self.now


# ---- Voice ----

def test_sound_file_is_made_once(tmp_path):
    voice = fake_voice(tmp_path)
    made = []
    old_make = voice.make
    voice.make = lambda command, **kw: (made.append(command), old_make(command, **kw))
    first = voice.sound_file('Listening')
    assert voice.sound_file('Listening') == first and len(made) == 1


def test_plays_at_full_or_quiet_volume(tmp_path):
    voice = fake_voice(tmp_path, quiet_volume=0.4)
    voice.play('Heard')
    assert voice.started[-1].command[1] == f'--volume={FULL_VOLUME}'
    voice.play('Heard', quiet=True)
    assert voice.started[-1].command[1] == f'--volume={int(FULL_VOLUME * 0.4)}'


def test_new_sentence_cuts_off_the_old_one(tmp_path):
    voice = fake_voice(tmp_path)
    voice.play('Still thinking')
    first = voice.started[-1]
    voice.play('There is a chair ahead')
    assert first.done and voice.playing()


def test_without_paplay_espeak_speaks_directly_and_quietly(tmp_path):
    voice = fake_voice(tmp_path)
    voice.player = None
    voice.play('Heard', quiet=True)
    assert voice.started[-1].command == ['/usr/bin/espeak-ng', '-s', '160', '-a', '40', 'Heard']


# ---- Mouth: Stage 1 always first ----

def test_normal_volume_when_clear_or_notice(tmp_path):
    mouth = Mouth(fake_voice(tmp_path), ASK_AGAIN)
    for level in ('clear', 'notice', None):
        mouth.set_level(level)
        assert mouth.say('Heard') == 'normal'


def test_quieter_at_warning(tmp_path):
    voice = fake_voice(tmp_path)
    mouth = Mouth(voice, ASK_AGAIN)
    mouth.set_level('warning')
    assert mouth.say('The sign says ESB219') == 'quiet'
    assert voice.started[-1].command[1] == f'--volume={int(FULL_VOLUME * 0.4)}'


def test_at_stop_answers_wait_and_feedback_is_dropped(tmp_path):
    voice = fake_voice(tmp_path)
    mouth = Mouth(voice, ASK_AGAIN)
    mouth.set_level('stop')
    assert mouth.say('Listening', keep=False) == 'dropped'
    assert mouth.say('The sign says ESB219') == 'held'
    assert voice.started == []                          # nothing plays during stop
    mouth.set_level('warning')                          # stop is over
    assert voice.started[-1].command[-1].endswith('.wav') and voice.playing()
    assert voice.started[-1].command[1] == f'--volume={int(FULL_VOLUME * 0.4)}'


def test_stop_cuts_off_an_answer_and_says_it_again_after(tmp_path):
    voice = fake_voice(tmp_path)
    mouth = Mouth(voice, ASK_AGAIN)
    mouth.set_level('clear')
    mouth.say('The sign says ESB219')
    playing = voice.started[-1]
    mouth.set_level('stop')
    assert playing.done and not voice.playing()         # cut off at once
    mouth.set_level('stop')                             # Stage 1 repeats its level
    assert len(voice.started) == 1
    mouth.set_level('clear')
    assert len(voice.started) == 2 and voice.playing()  # said again, from the start
    assert voice.started[1].command[-1] == playing.command[-1]


def test_stop_cuts_off_feedback_without_repeating_it(tmp_path):
    voice = fake_voice(tmp_path)
    mouth = Mouth(voice, ASK_AGAIN)
    mouth.say('Still thinking', keep=False)
    mouth.set_level('stop')
    mouth.set_level('clear')
    assert len(voice.started) == 1 and not voice.playing()


def test_finished_answer_is_not_repeated_after_stop(tmp_path):
    voice = fake_voice(tmp_path)
    mouth = Mouth(voice, ASK_AGAIN)
    mouth.say('The sign says ESB219')
    voice.started[-1].done = True                       # it finished playing
    mouth.set_level('stop')
    mouth.set_level('clear')
    assert len(voice.started) == 1


def test_answer_held_too_long_becomes_ask_again(tmp_path):
    voice, clock = fake_voice(tmp_path), FakeClock()
    mouth = Mouth(voice, ASK_AGAIN, hold_s=10.0, clock=clock)
    mouth.set_level('stop')
    mouth.say('The sign says ESB219')
    clock.now += 9.0
    mouth.tick()
    assert mouth.held == 'The sign says ESB219'
    clock.now += 2.0
    mouth.tick()
    assert mouth.held == ASK_AGAIN
    mouth.set_level('notice')
    path = voice.started[-1].command[-1]
    assert path == voice.sound_file(ASK_AGAIN)


def test_voice_off_says_nothing_but_does_not_crash():
    mouth = Mouth(None, ASK_AGAIN)
    assert mouth.say('Heard') == 'off'
    mouth.set_level('stop')
    assert mouth.say('An answer') == 'held'
    mouth.set_level('clear')
    mouth.tick()


def test_empty_sentence_is_ignored(tmp_path):
    voice = fake_voice(tmp_path)
    assert Mouth(voice, ASK_AGAIN).say('') == 'dropped' and voice.started == []


# ---- The PC's natural voice (Kokoro) ----

def test_kokoro_voice_comes_from_the_pc_once(tmp_path):
    pc = FakePC()
    voice = fake_voice(tmp_path, engine='kokoro', fetch=pc)
    path = voice.sound_file('There is a chair ahead')
    assert os.path.basename(path).startswith('kokoro_')
    assert open(path, 'rb').read() == b'RIFF kokoro There is a chair ahead'
    assert voice.sound_file('There is a chair ahead') == path and len(pc.asked) == 1


def test_pc_down_means_espeak_and_the_pc_is_left_alone_for_a_while(tmp_path):
    pc, clock = FakePC(up=False), FakeClock()
    voice = fake_voice(tmp_path, engine='kokoro', fetch=pc, clock=clock)
    path = voice.sound_file('Heard')
    assert not os.path.basename(path).startswith('kokoro_')      # espeak-ng's file
    voice.sound_file('Still thinking')
    assert pc.asked == ['Heard']                # not asked again while it is down
    pc.up = True
    clock.now += RETRY_S + 1
    assert os.path.basename(voice.sound_file('Heard')).startswith('kokoro_')


def test_playing_never_waits_on_the_network(tmp_path):
    pc = FakePC()
    voice = fake_voice(tmp_path, engine='kokoro', fetch=pc)
    voice.play('Listening')
    assert pc.asked == [] and not os.path.basename(voice.started[-1].command[-1]).startswith(
        'kokoro_')


def test_answer_is_said_in_the_pc_voice(tmp_path):
    pc = FakePC()
    voice = fake_voice(tmp_path, engine='kokoro', fetch=pc)
    assert Mouth(voice, ASK_AGAIN).say('The sign says ESB219') == 'normal'
    assert os.path.basename(voice.started[-1].command[-1]).startswith('kokoro_')


def test_feedback_word_plays_at_once_and_the_pc_voice_comes_later(tmp_path):
    pc = FakePC()
    voice = fake_voice(tmp_path, engine='kokoro', fetch=pc)
    Mouth(voice, ASK_AGAIN).say('Listening', keep=False)
    assert not os.path.basename(voice.started[-1].command[-1]).startswith('kokoro_')
    for _ in range(100):                        # fetched in the background for next time
        if os.path.exists(voice.pc_file('Listening')):
            break
        time.sleep(0.01)
    assert pc.asked == ['Listening'] and os.path.exists(voice.pc_file('Listening'))


def test_kokoro_works_without_espeak(tmp_path):
    voice = fake_voice(tmp_path, engine='kokoro')
    voice.espeak = None
    assert voice.ready()
    voice.sound_file('Heard')
    voice.play('Heard')
    assert voice.started[-1].command[-1] == voice.pc_file('Heard')


def test_unknown_engine_is_refused(tmp_path):
    with pytest.raises(ValueError):
        Voice(engine='robot', folder=str(tmp_path))


def test_fetch_from_pc_talks_to_a_voice_server():
    asked = []

    class Handler(BaseHTTPRequestHandler):
        def do_POST(self):  # noqa: N802 (name fixed by http.server)
            asked.append((self.path, json.loads(self.rfile.read(
                int(self.headers['Content-Length'])))))
            body = b'RIFF wav' if asked[-1][1]['text'] != 'bad' else b'{"error": "x"}'
            self.send_response(200)
            self.send_header('Content-Length', str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def log_message(self, *_):
            pass

    server = HTTPServer(('127.0.0.1', 0), Handler)
    threading.Thread(target=server.serve_forever, daemon=True).start()
    try:
        url = f'http://127.0.0.1:{server.server_port}'
        assert fetch_from_pc(url, 'Heard') == b'RIFF wav'
        assert asked == [('/speak', {'text': 'Heard'})]
        with pytest.raises(ValueError):
            fetch_from_pc(url, 'bad')           # a reply that is not a WAV file
    finally:
        server.shutdown()
        server.server_close()


def test_fetch_from_pc_unpacks_the_front_doors_squeezed_voice():
    """Remote link: the front door sends Opus when asked; it is unpacked to WAV."""
    accepts = []

    class Handler(BaseHTTPRequestHandler):
        def do_POST(self):  # noqa: N802 (name fixed by http.server)
            self.rfile.read(int(self.headers['Content-Length']))
            accepts.append(self.headers.get('Accept'))
            body = b'OggS opus' if 'audio/ogg' in (self.headers.get('Accept') or '') \
                else b'RIFF wav'
            self.send_response(200)
            self.send_header('Content-Length', str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def log_message(self, *_):
            pass

    server = HTTPServer(('127.0.0.1', 0), Handler)
    threading.Thread(target=server.serve_forever, daemon=True).start()
    try:
        url = f'http://127.0.0.1:{server.server_port}'
        unpacked = []

        def unpack(data):
            unpacked.append(data)
            return b'RIFF unpacked'

        assert fetch_from_pc(url, 'Heard', unpack=unpack, can_unpack=True) == b'RIFF unpacked'
        assert unpacked == [b'OggS opus'] and 'audio/ogg' in accepts[-1]
        # a Jetson that cannot unpack does not ask, and gets the WAV
        assert fetch_from_pc(url, 'Heard', unpack=unpack, can_unpack=False) == b'RIFF wav'
        assert accepts[-1] is None
    finally:
        server.shutdown()
        server.server_close()


# ---- sound through the browser ----

def browser_voice(tmp_path, clock):
    """Make a Voice that hands its sound to a list instead of playing it."""
    import wave
    events = []

    def make(command, **_):
        with wave.open(command[command.index('-w') + 1], 'wb') as sound:
            sound.setnchannels(1)
            sound.setsampwidth(2)
            sound.setframerate(16000)
            sound.writeframes(b'\0\0' * 16000 * 2)          # 2 seconds

    def run(command, **_):
        raise AssertionError('nothing may be played on the Jetson: %s' % command)

    voice = Voice(0.4, run=run, make=make, folder=str(tmp_path), clock=clock,
                  out=events.append)
    voice.espeak = '/usr/bin/espeak-ng'
    return voice, events


def test_browser_voice_hands_over_the_file_and_the_volume(tmp_path):
    clock = FakeClock()
    voice, events = browser_voice(tmp_path, clock)
    assert voice.ready()
    voice.play('You have arrived')
    voice.play('Careful', quiet=True)
    assert [e['kind'] for e in events] == ['play', 'stop', 'play']
    assert events[0]['volume'] == 1.0 and events[2]['volume'] == 0.4
    assert events[0]['sentence'] == 'You have arrived' and os.path.exists(events[0]['file'])


def test_browser_voice_knows_when_a_sentence_ends_and_stop_cuts_it(tmp_path):
    clock = FakeClock()
    voice, events = browser_voice(tmp_path, clock)
    voice.play('You have arrived')
    assert voice.playing()
    clock.now += 2.5
    assert not voice.playing()
    voice.stop()                                # nothing playing: no "stop" sent
    assert [e['kind'] for e in events] == ['play']
    voice.play('Again')
    voice.stop()
    assert events[-1] == {'kind': 'stop'} and not voice.playing()


def test_stage1_stop_cuts_the_browser_answer_and_it_comes_back_after(tmp_path):
    clock = FakeClock()
    voice, events = browser_voice(tmp_path, clock)
    mouth = Mouth(voice, ASK_AGAIN, clock=clock)
    mouth.set_level('clear')
    assert mouth.say('The door is on your left') == 'normal'
    mouth.set_level('stop')
    assert events[-1] == {'kind': 'stop'}
    mouth.set_level('warning')
    assert events[-1]['kind'] == 'play' and events[-1]['volume'] == 0.4
    assert events[-1]['sentence'] == 'The door is on your left'


# ---- the device's own sentences are urgent (16 Nov) ----

def test_an_urgent_sentence_is_said_at_stop_and_not_cut_off(tmp_path):
    voice = fake_voice(tmp_path)
    mouth = Mouth(voice, ASK_AGAIN)
    mouth.set_level('stop')
    assert mouth.say('Something is in the way. Waiting', urgent=True) == 'normal'
    assert len(voice.started) == 1 and not voice.started[0].done
    mouth.set_level('stop')                                 # Stage 1 keeps saying stop
    assert not voice.started[0].done                        # still playing
    assert mouth.say('Still thinking', keep=False) == 'dropped'


def test_an_urgent_sentence_is_full_volume_at_warning(tmp_path):
    voice = fake_voice(tmp_path)
    mouth = Mouth(voice, ASK_AGAIN)
    mouth.set_level('warning')
    assert mouth.say('You have arrived', urgent=True) == 'normal'
    assert f'--volume={FULL_VOLUME}' in voice.started[0].command


def test_an_answer_waits_for_an_urgent_sentence_then_is_said(tmp_path):
    voice = fake_voice(tmp_path)
    mouth = Mouth(voice, ASK_AGAIN)
    mouth.set_level('clear')
    mouth.say('You have arrived', urgent=True)
    assert mouth.say('The sign says ESB117') == 'held'
    mouth.set_level('clear')
    assert len(voice.started) == 1                          # the urgent one still plays
    voice.started[0].done = True
    mouth.tick()
    assert len(voice.started) == 2 and 'ESB117' not in voice.started[0].command[-1]
