"""
Tests for front_door.py with three fake servers standing in for Qwen, speech and voice.
No tunnel, no Ollama, no graphics card.

Run from the code folder:  python3 -m pytest ai_server/test_front_door.py
or without pytest:         python ai_server/test_front_door.py
"""

import json
import os
import sys
import tempfile
import threading
import urllib.error
import urllib.request
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

sys.path.insert(0, os.path.dirname(__file__))
from front_door import load_token, serve, shown, split_path  # noqa: E402 (path line above)

TOKEN = 'test-token-0123456789abcdef'


class Echo(BaseHTTPRequestHandler):
    """A fake server: answers with what it got, or an error for /fail."""

    def log_message(self, *args):
        pass

    def answer(self, method):
        body = self.rfile.read(int(self.headers.get('Content-Length') or 0))
        if self.path == '/fail':
            data = b'{"error": "broken"}'
            self.send_response(500)
        else:
            data = json.dumps({'server': self.server.name, 'method': method, 'path': self.path,
                               'type': self.headers.get('Content-Type'),
                               'body': body.decode('latin-1')}).encode()
            self.send_response(200)
        self.send_header('Content-Type', 'application/json')
        self.send_header('Content-Length', str(len(data)))
        self.end_headers()
        self.wfile.write(data)

    def do_GET(self):
        self.answer('GET')

    def do_POST(self):
        self.answer('POST')


def start(server):
    threading.Thread(target=server.serve_forever, daemon=True).start()
    return server


def setup():
    """Fake ai and voice servers, speech left down; the front door in front of them."""
    fakes = {}
    for name in ('ai', 'voice'):
        fake = ThreadingHTTPServer(('127.0.0.1', 0), Echo)
        fake.name = name
        fakes[name] = start(fake)
    backends = {name: 'http://127.0.0.1:%d' % fake.server_address[1]
                for name, fake in fakes.items()}
    backends['speech'] = 'http://127.0.0.1:1'          # nothing listens there
    door = start(serve(0, TOKEN, backends, quiet=True))
    base = 'http://127.0.0.1:%d' % door.server_address[1]
    return base, [door] + list(fakes.values())


def call(url, body=None, kind='application/json'):
    request = urllib.request.Request(url, data=body, headers={'Content-Type': kind} if body else {})
    try:
        with urllib.request.urlopen(request, timeout=10) as reply:
            return reply.status, reply.read()
    except urllib.error.HTTPError as error:
        return error.code, error.read()


def stop(servers):
    for server in servers:
        server.shutdown()
        server.server_close()


def test_split_path():
    assert split_path('/k/%s/voice/speak' % TOKEN, TOKEN) == ('voice', '/speak')
    assert split_path('/k/%s/ai/api/chat' % TOKEN, TOKEN) == ('ai', '/api/chat')
    assert split_path('/k/%s/voice?x=1' % TOKEN, TOKEN) == ('voice', '?x=1')
    assert split_path('/k/%s/health' % TOKEN, TOKEN) == ('health', '')
    assert split_path('/k/wrong/voice/speak', TOKEN) is None
    assert split_path('/voice/speak', TOKEN) is None
    assert split_path('/k/%s/other/x' % TOKEN, TOKEN) is None
    assert split_path('/k/%s' % TOKEN, TOKEN) is None


def test_shown_hides_the_token():
    assert TOKEN not in shown('/k/%s/voice/speak' % TOKEN)
    assert shown('/k/%s/voice/speak' % TOKEN) == '/k/***/voice/speak'


def test_passes_requests_like_the_jetson_makes_them():
    base, servers = setup()
    try:
        door = base + '/k/' + TOKEN
        # voice.py: url.rstrip('/') + '/speak', JSON in
        code, data = call(door + '/voice' + '/speak', b'{"text": "hello"}')
        got = json.loads(data)
        assert code == 200 and got['server'] == 'voice' and got['path'] == '/speak'
        assert got['method'] == 'POST' and got['body'] == '{"text": "hello"}'
        # finder.py: url.rstrip('/') + '/api/chat'
        code, data = call(door + '/ai' + '/api/chat', b'{"model": "q"}')
        assert code == 200 and json.loads(data)['path'] == '/api/chat'
        # a binary body (a WAV) arrives unchanged, with its type
        wav = bytes(range(256)) * 40
        code, data = call(door + '/voice/echo', wav, 'audio/wav')
        got = json.loads(data)
        assert got['body'].encode('latin-1') == wav and got['type'] == 'audio/wav'
        # GET with a query
        code, data = call(door + '/ai/api/tags?x=1')
        assert code == 200 and json.loads(data)['path'] == '/api/tags?x=1'
    finally:
        stop(servers)


def test_wrong_or_missing_token_is_not_found():
    base, servers = setup()
    try:
        for path in ('/k/wrong/voice/speak', '/voice/speak', '/', '/k/%s/nothing' % TOKEN):
            code, _ = call(base + path, b'{}')
            assert code == 404, path
    finally:
        stop(servers)


def test_server_errors_and_down_servers():
    base, servers = setup()
    try:
        door = base + '/k/' + TOKEN
        code, data = call(door + '/voice/fail', b'{}')
        assert code == 500 and b'broken' in data          # the server's own error, passed on
        code, data = call(door + '/speech/transcribe', b'RIFF', 'audio/wav')
        assert code == 502 and b'speech' in data          # nothing listening
    finally:
        stop(servers)


def test_health_says_which_servers_answer():
    base, servers = setup()
    try:
        code, data = call(base + '/k/%s/health' % TOKEN)
        assert code == 200
        assert json.loads(data) == {'front_door': True, 'ai': True, 'voice': True,
                                    'speech': False}
    finally:
        stop(servers)


class Wav(Echo):
    """A fake voice server: always a WAV, whatever is asked."""

    def do_POST(self):  # noqa: N802 (name fixed by http.server)
        self.rfile.read(int(self.headers.get('Content-Length') or 0))
        data = make_wav(0.5)
        self.send_response(200)
        self.send_header('Content-Type', 'audio/wav')
        self.send_header('Content-Length', str(len(data)))
        self.end_headers()
        self.wfile.write(data)


def make_wav(seconds, rate=24000):
    """A WAV like Kokoro's (24 kHz, mono, 16 bit) holding a rising tone."""
    import io
    import math
    import struct
    import wave
    frames = b''.join(struct.pack('<h', int(9000 * math.sin(2 * math.pi * (200 + i / 40) * i / rate)))
                      for i in range(int(seconds * rate)))
    out = io.BytesIO()
    with wave.open(out, 'wb') as sound:
        sound.setnchannels(1)
        sound.setsampwidth(2)
        sound.setframerate(rate)
        sound.writeframes(frames)
    return out.getvalue()


def test_squeeze_makes_opus_much_smaller():
    from front_door import FFMPEG, squeeze
    if not os.path.exists(FFMPEG):
        print('(no ffmpeg here: squeeze not tried)')
        return
    wav = make_wav(4.0)
    small = squeeze(wav)
    assert small is not None and small.startswith(b'OggS')
    assert len(small) * 6 < len(wav), (len(small), len(wav))
    assert squeeze(b'not a sound') is None                  # ffmpeg refuses: the WAV goes


def test_voice_is_squeezed_only_when_the_jetson_asks():
    fake = start(ThreadingHTTPServer(('127.0.0.1', 0), Wav))
    backends = {'ai': 'http://127.0.0.1:1', 'speech': 'http://127.0.0.1:1',
                'voice': 'http://127.0.0.1:%d' % fake.server_address[1]}
    results = iter([b'OggS small', None])
    door = start(serve(0, TOKEN, backends, quiet=True, squeeze=lambda wav: next(results)))
    url = 'http://127.0.0.1:%d/k/%s/voice/speak' % (door.server_address[1], TOKEN)
    try:
        def speak(accept):
            headers = {'Content-Type': 'application/json'}
            if accept:
                headers['Accept'] = accept
            request = urllib.request.Request(url, data=b'{"text": "hi"}', headers=headers)
            with urllib.request.urlopen(request, timeout=10) as reply:
                return reply.headers['Content-Type'], reply.read()

        assert speak('audio/ogg, audio/wav') == ('audio/ogg', b'OggS small')
        kind, data = speak('audio/ogg, audio/wav')               # squeeze failed: the WAV
        assert kind == 'audio/wav' and data.startswith(b'RIFF')
        kind, data = speak(None)                                 # not asked: the WAV, no squeeze
        assert kind == 'audio/wav' and data.startswith(b'RIFF')
    finally:
        stop([door, fake])


def test_ask_runs_the_question_code_with_the_local_servers():
    fake = start(ThreadingHTTPServer(('127.0.0.1', 0), Wav))
    backends = {'ai': 'http://127.0.0.1:11', 'speech': 'http://127.0.0.1:12',
                'voice': 'http://127.0.0.1:%d' % fake.server_address[1]}
    got = []

    def answer_on_pc(request, speech_url, ollama_url, voice=None):
        got.append((request, speech_url, ollama_url))
        sound = voice('Hello.', True)
        return {'say': 'Hello.', 'voice': sound.decode('latin-1')}

    door = start(serve(0, TOKEN, backends, quiet=True, squeeze=lambda wav: b'OggS small',
                       answer_on_pc=answer_on_pc))
    base = 'http://127.0.0.1:%d/k/%s' % (door.server_address[1], TOKEN)
    try:
        code, data = call(base + '/ask', b'{"text": "hi"}')
        assert code == 200 and json.loads(data) == {'say': 'Hello.', 'voice': 'OggS small'}
        assert got == [({'text': 'hi'}, backends['speech'], backends['ai'])]
        code, _ = call(base + '/ask')                          # GET: not a question
        assert code == 404
        code, _ = call('http://127.0.0.1:%d/k/wrong/ask' % door.server_address[1], b'{}')
        assert code == 404
    finally:
        stop([door, fake])


def test_ask_trouble_is_an_error_not_a_crash():
    def broken(request, speech_url, ollama_url, voice=None):
        raise RuntimeError('boom')

    down = 'http://127.0.0.1:1'
    door = start(serve(0, TOKEN, {'ai': down, 'speech': down, 'voice': 'nonsense'}, quiet=True,
                       answer_on_pc=broken))
    base = 'http://127.0.0.1:%d/k/%s' % (door.server_address[1], TOKEN)
    try:
        code, data = call(base + '/ask', b'{"text": "hi"}')
        assert code == 500 and b'boom' in data
        code, data = call(base + '/health')                    # still answering
        assert code == 200
    finally:
        stop([door])


def test_the_jetsons_question_code_loads_on_this_pc():
    from front_door import load_answer_on_pc
    try:
        import cv2  # noqa: F401
        import numpy  # noqa: F401
    except ImportError:
        print('(no numpy or OpenCV here: /ask would say not found)')
        return
    assert load_answer_on_pc() is not None


def test_the_jetson_says_where_its_tunnel_is_and_the_pc_reads_it():
    from front_door import read_jetson
    with tempfile.TemporaryDirectory() as folder:
        kept = os.path.join(folder, 'jetson.json')
        down = 'http://127.0.0.1:1'
        door = start(serve(0, TOKEN, {'ai': down, 'speech': down, 'voice': down}, quiet=True,
                           jetson_file=kept))
        base = 'http://127.0.0.1:%d/k/%s' % (door.server_address[1], TOKEN)
        try:
            code, data = call(base + '/jetson')
            assert code == 200 and json.loads(data) == {}                  # never reported
            address = 'https://some-words-here.trycloudflare.com'
            code, _ = call(base + '/jetson', json.dumps({'ssh': address}).encode())
            assert code == 200
            code, data = call(base + '/jetson')
            got = json.loads(data)
            assert got['ssh'] == address and 0 <= got['age_s'] < 5
            assert read_jetson(kept)['ssh'] == address
            for bad in ('http://evil.example.com', 'https://x.trycloudflare.com.evil.com', ''):
                code, _ = call(base + '/jetson', json.dumps({'ssh': bad}).encode())
                assert code == 400, bad                                      # only a tunnel
            assert read_jetson(kept)['ssh'] == address                       # kept the good one
            code, _ = call('http://127.0.0.1:%d/k/wrong/jetson' % door.server_address[1])
            assert code == 404
        finally:
            stop([door])


def test_jetson_link_only_trusts_fresh_news():
    from jetson_link import where
    assert where(lambda: {}) == ''
    assert where(lambda: {'ssh': 'https://a.trycloudflare.com', 'age_s': 31}) == \
        'https://a.trycloudflare.com'
    assert where(lambda: {'ssh': 'https://a.trycloudflare.com', 'age_s': 600}) == ''


def test_token_file():
    with tempfile.TemporaryDirectory() as folder:
        path = os.path.join(folder, 'token.txt')
        first = load_token(path)
        assert len(first) >= 16 and load_token(path) == first      # kept between runs
        second = load_token(path, new=True)
        assert second != first and load_token(path) == second


if __name__ == '__main__':
    tests = [value for name, value in sorted(globals().items()) if name.startswith('test_')]
    for test in tests:
        test()
        print('passed', test.__name__)
    print('%d of %d passed' % (len(tests), len(tests)))
