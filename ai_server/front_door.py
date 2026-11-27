"""
The front door: one address on the campus PC for the Jetson to reach over the internet.

The remote link. The campus firewall blocks every
connection coming in, so a tunnel program (Cloudflare quick tunnel) dials out from this PC and
gives this front door a web address. The Jetson then reaches the PC's three servers through it:

    https://<tunnel address>/k/<token>/ai/...      -> Qwen in Ollama   (127.0.0.1:11434)
    https://<tunnel address>/k/<token>/speech/...  -> speech to text   (127.0.0.1:8090)
    https://<tunnel address>/k/<token>/voice/...   -> Kokoro voice     (127.0.0.1:8091)
    https://<tunnel address>/k/<token>/health      -> which of the three answer
    https://<tunnel address>/k/<token>/ask         -> a whole question in one trip: speech to text,
                                                      sorting, Qwen and the voice, run here with
                                                      the Jetson's own code (remote_ask.py)
    https://<tunnel address>/k/<token>/jetson      -> POST {"ssh": address}: the Jetson says where
                                                      its own ssh tunnel is (remote_link.sh);
                                                      GET: where it last said (jetson_link.py)

The token is in the address so the Jetson's code only needs one setting:
    pc:=https://<tunnel address>/k/<token>
Anything without the right token gets "404 not found", so a stranger learns nothing.

Listens on 127.0.0.1 only: the tunnel runs on this PC, nothing else needs to reach it.
Python's own web server, no installs.

Run:
    python front_door.py                 port 8095, token from front_door_token.txt (made on
                                         the first run; the file is never committed)
    python front_door.py --new-token     make a new token (the old address stops working)
Then, in another window:
    cloudflared tunnel --url http://127.0.0.1:8095
and give the Jetson   pc:=<the trycloudflare.com address>/k/<token>
"""

import argparse
import hmac
import json
import os
import re
import secrets
import shutil
import socket
import subprocess
import sys
import time
import urllib.error
import urllib.request
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

PORT = 8095
TOKEN_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'front_door_token.txt')
BACKENDS = {                      # first part of the path after the token -> local server
    'ai': 'http://127.0.0.1:11434',
    'speech': 'http://127.0.0.1:8090',
    'voice': 'http://127.0.0.1:8091',
}
MAX_BODY = 25 * 1024 * 1024       # a photo for Qwen or a recording is a few MB at most
TIMEOUT_S = 120.0                 # Qwen's first answer after it unloaded takes a while
PASSED_HEADERS = ('Content-Type', 'Accept')
FFMPEG = shutil.which('ffmpeg') or r'C:\ffmpeg\bin\ffmpeg.exe'
OPUS_RATE = '32k'                 # Opus at 32 kbit/s: speech stays clear, about 12 times smaller


def squeeze(wav, ffmpeg=FFMPEG):
    """WAV bytes -> Ogg Opus bytes, or None if ffmpeg cannot (then the WAV goes as it is).

    Over the internet the voice's 200 kB WAV took 7 to 10 s (18 Nov), and the Jetson waits 3 s.
    Only the voice going to the Jetson is squeezed, never the user's recording for speech to
    text, so the words heard (and what Qwen gets) do not change.
    """
    try:
        done = subprocess.run([ffmpeg, '-hide_banner', '-loglevel', 'error', '-i', 'pipe:0',
                               '-c:a', 'libopus', '-b:a', OPUS_RATE, '-f', 'ogg', 'pipe:1'],
                              input=wav, capture_output=True, timeout=10)
    except (OSError, subprocess.TimeoutExpired):
        return None
    return done.stdout if done.returncode == 0 and done.stdout.startswith(b'OggS') else None


def load_token(path=TOKEN_FILE, new=False):
    """The token from its file; makes one (and the file) on the first run or with new=True."""
    if not new and os.path.exists(path):
        with open(path, encoding='utf-8') as f:
            token = f.read().strip()
        if len(token) >= 16:
            return token
    token = secrets.token_urlsafe(24)
    with open(path, 'w', encoding='utf-8') as f:
        f.write(token + '\n')
    return token


def split_path(path, token):
    """'/k/<token>/voice/speak?x=1' -> ('voice', '/speak?x=1'); None if the token is wrong.

    '/k/<token>/health' -> ('health', '').
    """
    parts = path.split('/', 3)            # ['', 'k', token, 'voice/speak?x=1']
    if len(parts) < 3 or parts[0] != '' or parts[1] != 'k':
        return None
    if not hmac.compare_digest(parts[2].encode(), token.encode()):
        return None
    rest = parts[3] if len(parts) > 3 else ''
    name, slash, tail = rest.partition('/')
    name, query, after = name.partition('?')
    if name in ('health', 'ask', 'jetson'):
        return name, ''
    if name not in BACKENDS:
        return None
    return name, '/' + tail if slash else ('?' + after if query else '')


def shown(path):
    """The path with the token hidden, for the log."""
    parts = path.split('/', 3)
    if len(parts) >= 3 and parts[1] == 'k':
        parts[2] = '***'
    return '/'.join(parts)


def port_open(url, timeout=1.0):
    try:
        host, port = url.split('//', 1)[1].rsplit(':', 1)
        with socket.create_connection((host, int(port)), timeout=timeout):
            return True
    except (OSError, ValueError, IndexError):   # down, or an address this cannot read
        return False


JETSON_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'front_door_jetson.json')


def save_jetson(address, path=JETSON_FILE, clock=time.time):
    """Keep where the Jetson's ssh tunnel is; False if it is not a tunnel address."""
    if not re.fullmatch(r'https://[a-z0-9-]+\.trycloudflare\.com', address or ''):
        return False
    with open(path, 'w', encoding='utf-8') as f:
        json.dump({'ssh': address, 'at': clock()}, f)
    return True


def read_jetson(path=JETSON_FILE, clock=time.time):
    """Where the Jetson last said its ssh tunnel is: {'ssh', 'age_s'}, or {} if never."""
    try:
        with open(path, encoding='utf-8') as f:
            kept = json.load(f)
    except (OSError, ValueError):
        return {}
    return {'ssh': kept.get('ssh', ''), 'age_s': round(clock() - float(kept.get('at', 0)), 1)}


STAGE2_CODE = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'src',
                           'apnvi_stage2')


def load_answer_on_pc():
    """The Jetson's own question code (apnvi_stage2.remote_ask), or None if it cannot load
    here (it needs numpy and OpenCV); then /ask says "not found" and the Jetson asks in four trips.
    """
    if STAGE2_CODE not in sys.path:
        sys.path.insert(0, STAGE2_CODE)
    try:
        from apnvi_stage2.remote_ask import answer_on_pc
    except Exception as problem:                # noqa: BLE001 (any import trouble: no /ask)
        sys.stderr.write('No /ask here (the Jetson asks in four trips): %s\n' % problem)
        return None
    return answer_on_pc


def make_handler(token, backends, quiet=False, squeeze=squeeze, answer_on_pc=None,
                 jetson_file=JETSON_FILE):
    asker = [None]          # the question code, loaded on the first /ask (None = not tried yet)

    class Handler(BaseHTTPRequestHandler):
        protocol_version = 'HTTP/1.1'

        def log_message(self, fmt, *args):
            if not quiet:
                sys.stderr.write('%s %s\n' % (self.log_date_time_string(),
                                              (fmt % args).replace(self.path, shown(self.path))))

        def reply(self, code, body, kind='application/json'):
            self.send_response(code)
            self.send_header('Content-Type', kind)
            self.send_header('Content-Length', str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def not_found(self, body_read=False):
            # Read the body first: closing with it unread makes Windows cut the connection, and
            # the caller sees a dropped line instead of "not found".
            length = int(self.headers.get('Content-Length') or 0)
            if length > MAX_BODY:
                self.close_connection = True
            elif length and not body_read:
                self.rfile.read(length)
            self.reply(404, b'{"error": "not found"}')

        def voice_for(self, sentence, squeezed):
            """The voice server's sound for a sentence, squeezed if asked; None if it fails."""
            request = urllib.request.Request(
                backends['voice'] + '/speak', data=json.dumps({'text': sentence}).encode(),
                headers={'Content-Type': 'application/json'})
            try:
                with urllib.request.urlopen(request, timeout=15) as answer:
                    wav = answer.read()
            except (urllib.error.URLError, OSError):
                return None
            if not wav.startswith(b'RIFF'):
                return None
            return (squeeze(wav) or wav) if squeezed else wav

        def ask(self, method):
            """One trip for a whole question (remote_ask.py in apnvi_stage2)."""
            if method != 'POST':
                return self.not_found()
            length = int(self.headers.get('Content-Length') or 0)
            if length > MAX_BODY:
                self.close_connection = True
                return self.reply(413, b'{"error": "too big"}')
            body = self.rfile.read(length) if length else b''
            if asker[0] is None:
                asker[0] = answer_on_pc or load_answer_on_pc() or False
            if not asker[0]:
                return self.not_found(body_read=True)
            try:
                answer = asker[0](json.loads(body), backends['speech'], backends['ai'],
                                  voice=self.voice_for)
            except Exception as problem:        # noqa: BLE001 (never kill the front door)
                data = json.dumps({'error': 'the PC could not answer: %s' % problem}).encode()
                return self.reply(500, data)
            self.reply(200, json.dumps(answer).encode())

        def jetson(self, method):
            """Where the Jetson's own ssh tunnel is: it POSTs, the PC GETs."""
            if method == 'GET':
                return self.reply(200, json.dumps(read_jetson(jetson_file)).encode())
            length = int(self.headers.get('Content-Length') or 0)
            if length > 4096:
                self.close_connection = True
                return self.reply(413, b'{"error": "too big"}')
            try:
                address = json.loads(self.rfile.read(length) or b'{}').get('ssh', '')
            except (ValueError, AttributeError):
                address = ''
            if not save_jetson(address, jetson_file):
                return self.reply(400, b'{"error": "not a tunnel address"}')
            self.reply(200, b'{"ok": true}')

        def handle_any(self, method):
            where = split_path(self.path, token)
            if where is None:
                return self.not_found()
            name, tail = where
            if name == 'ask':
                return self.ask(method)
            if name == 'jetson':
                return self.jetson(method)
            if name == 'health':
                state = {key: port_open(url) for key, url in backends.items()}
                return self.reply(200, json.dumps({'front_door': True, **state}).encode())
            length = int(self.headers.get('Content-Length') or 0)
            if length > MAX_BODY:
                return self.reply(413, b'{"error": "too big"}')
            body = self.rfile.read(length) if length else None
            headers = {h: self.headers[h] for h in PASSED_HEADERS if self.headers.get(h)}
            request = urllib.request.Request(backends[name] + tail, data=body, method=method,
                                             headers=headers)
            try:
                with urllib.request.urlopen(request, timeout=TIMEOUT_S) as answer:
                    code, data = answer.status, answer.read()
                    kind = answer.headers.get('Content-Type', 'application/octet-stream')
            except urllib.error.HTTPError as error:      # the server answered with an error
                code, data = error.code, error.read()
                kind = error.headers.get('Content-Type', 'application/json')
            except (urllib.error.URLError, OSError) as problem:
                data = json.dumps({'error': '%s server on the PC did not answer: %s'
                                   % (name, getattr(problem, 'reason', problem))}).encode()
                return self.reply(502, data)
            if (name == 'voice' and code == 200 and data.startswith(b'RIFF')
                    and 'audio/ogg' in (self.headers.get('Accept') or '')):
                small = squeeze(data)                 # the Jetson said it can unpack Opus
                if small:
                    data, kind = small, 'audio/ogg'
            self.reply(code, data, kind)

        def do_GET(self):
            self.handle_any('GET')

        def do_POST(self):
            self.handle_any('POST')

    return Handler


def serve(port=PORT, token=None, backends=None, quiet=False, squeeze=squeeze, answer_on_pc=None,
          jetson_file=JETSON_FILE):
    """Start the front door; returns the server (call serve_forever or shutdown on it)."""
    handler = make_handler(token or load_token(), backends or BACKENDS, quiet, squeeze,
                           answer_on_pc, jetson_file)
    return ThreadingHTTPServer(('127.0.0.1', port), handler)


def main():
    parser = argparse.ArgumentParser(description=__doc__.split('\n\n')[1])
    parser.add_argument('--port', type=int, default=PORT)
    parser.add_argument('--new-token', action='store_true',
                        help='make a new token; the old address stops working')
    args = parser.parse_args()
    token = load_token(new=args.new_token)
    server = serve(args.port, token)
    print('Front door on http://127.0.0.1:%d' % args.port)
    print('Tunnel:  cloudflared tunnel --url http://127.0.0.1:%d' % args.port)
    print('Jetson:  pc:=<the trycloudflare.com address>/k/%s' % token)
    print('Check:   curl <the trycloudflare.com address>/k/%s/health' % token)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
