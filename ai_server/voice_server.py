r"""
Voice server for the campus PC: a sentence goes in, a natural human voice comes out.

Runs on the campus PC (Windows), next to the speech server and Ollama. The Jetson sends a
sentence ("Listening", or Qwen's answer) and gets back a WAV sound file to play
(Adel, 31 Oct: a voice that sounds human, made on the PC, in place of espeak-ng).

Uses Kokoro, a small free text-to-speech model (82 million numbers, Apache licence), through
the kokoro-onnx library. The first start downloads Kokoro's two files (about 350 MB) from
GitHub into C:\apnvi\kokoro. Nothing from ROS: plain Python.

Start it (in a Command Prompt, in this folder, with the voice environment, see README.md):
  python voice_server.py                       processor, voice af_bella, port 8091
  python voice_server.py --device cuda         graphics card (needs onnxruntime-gpu)
  python voice_server.py --voice bf_emma       another voice (list: GET /health)

What it answers:
  POST /speak    body = {"text": "Listening"}   -> the WAV file (24 kHz, mono, 16 bit)
                 optional "voice" and "speed" in the body; the reply header X-Seconds says
                 how long making it took
  GET  /health                                  -> {"ok": true, "voice": "af_bella", ...}
"""

import argparse
from collections import OrderedDict
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import io
import json
import os
import re
import threading
import time
import urllib.request
import wave

# ---- Settings (each can be changed on the command line) ----
PORT = 8091
VOICE = 'af_bella'                      # Adel's pick, 31 Oct. a/b = US/UK, f/m = woman/man
SPEED = 1.0                             # 1.0 = normal; 1.1 = a little faster
DEVICE = 'cpu'                          # 'cpu' = processor, 'cuda' = graphics card
FOLDER = r'C:\apnvi\kokoro' if os.name == 'nt' else os.path.expanduser('~/kokoro')
MODEL_FILE = 'kokoro-v1.0.onnx'
VOICES_FILE = 'voices-v1.0.bin'
DOWNLOAD_FROM = 'https://github.com/thewh1teagle/kokoro-onnx/releases/download/model-files-v1.0/'
MAX_TEXT = 600                          # characters; Qwen's answers are one or two sentences
KEEP = 200                              # sentences kept ready in memory (feedback words, repeats)
PEAK = 0.9                              # loudest point of every file, so all are equally loud
PROVIDERS = {'cpu': ['CPUExecutionProvider'],
             'cuda': ['CUDAExecutionProvider', 'CPUExecutionProvider'],
             'directml': ['DmlExecutionProvider', 'CPUExecutionProvider']}

CODE = re.compile(r'\b([A-Z]{2,5}) ?(\d{2,4})\b')     # room codes like ESB2090 or ESB 117


def say_digits(digits):
    """'2090' -> '20 90', '117' -> '1 17', '2005' -> '20 oh 5': how people read room numbers."""
    if len(digits) == 3:
        parts = [digits[0], digits[1:]]
    elif len(digits) == 4:
        parts = [digits[:2], digits[2:]]
    else:
        parts = [digits]
    return ' '.join('oh ' + part[1] if len(part) == 2 and part[0] == '0' else part
                    for part in parts)


def speakable(text):
    """Make text easier to say: 'ESB2090' -> 'E S B 20 90' (said 'E S B twenty ninety')."""
    text = CODE.sub(lambda m: ' '.join(m.group(1)) + ' ' + say_digits(m.group(2)), text)
    return ' '.join(text.split())


def to_wav(samples, rate):
    """Turn Kokoro's sound (numbers from -1 to 1) into the bytes of a 16-bit WAV file."""
    import numpy as np
    samples = np.asarray(samples, dtype=np.float32)
    loudest = float(np.max(np.abs(samples))) if samples.size else 0.0
    if loudest > 0:
        samples = samples * (PEAK / loudest)
    pcm = (np.clip(samples, -1.0, 1.0) * 32767).astype('<i2').tobytes()
    out = io.BytesIO()
    with wave.open(out, 'wb') as file:
        file.setnchannels(1)
        file.setsampwidth(2)
        file.setframerate(rate)
        file.writeframes(pcm)
    return out.getvalue()


def download(folder):
    """Download Kokoro's two files into folder, the first time only."""
    os.makedirs(folder, exist_ok=True)
    for name in (MODEL_FILE, VOICES_FILE):
        path = os.path.join(folder, name)
        if os.path.exists(path):
            continue
        print(f'Downloading {name} (first start only)...', flush=True)
        urllib.request.urlretrieve(DOWNLOAD_FROM + name, path + '.part')
        os.replace(path + '.part', path)            # only a finished download gets the name


class KokoroEngine:
    """Wraps kokoro-onnx: load the model once, then turn sentences into WAV files."""

    def __init__(self, folder=FOLDER, voice=VOICE, speed=SPEED, device=DEVICE):
        import onnxruntime                          # only needed on the campus PC
        from kokoro_onnx import Kokoro
        download(folder)
        session = onnxruntime.InferenceSession(os.path.join(folder, MODEL_FILE),
                                               providers=PROVIDERS[device])
        self.kokoro = Kokoro.from_session(session, os.path.join(folder, VOICES_FILE))
        self.voices = sorted(self.kokoro.get_voices())
        if voice not in self.voices:
            raise ValueError(f'unknown voice {voice}. Voices: {", ".join(self.voices)}')
        self.voice, self.speed = voice, speed
        self.device = session.get_providers()[0]    # what really runs it, e.g. CPU or CUDA

    def make(self, text, voice, speed):
        """Return the WAV bytes of text said by voice at speed."""
        if voice not in self.voices:
            raise ValueError(f'unknown voice {voice}')
        samples, rate = self.kokoro.create(speakable(text), voice=voice, speed=speed,
                                           lang='en-us' if voice[0] == 'a' else 'en-gb')
        return to_wav(samples, rate)


def make_handler(engine):
    """Build the request handler that uses this engine."""
    lock = threading.Lock()                         # one sentence at a time
    kept = OrderedDict()                            # (text, voice, speed) -> WAV bytes

    def wav_for(text, voice, speed):
        """Return (WAV bytes, seconds it took). Sentences said before come back at once."""
        key = (text, voice, speed)
        start = time.monotonic()
        with lock:
            if key in kept:
                kept.move_to_end(key)
                return kept[key], 0.0
            wav = engine.make(text, voice, speed)
            kept[key] = wav
            if len(kept) > KEEP:
                kept.popitem(last=False)            # forget the oldest
        return wav, time.monotonic() - start

    class Handler(BaseHTTPRequestHandler):
        def send_json(self, status, body):
            """Send a JSON reply."""
            data = json.dumps(body).encode('utf-8')
            self.send_response(status)
            self.send_header('Content-Type', 'application/json')
            self.send_header('Content-Length', str(len(data)))
            self.end_headers()
            self.wfile.write(data)

        def do_GET(self):  # noqa: N802 (name fixed by http.server)
            """Answer /health, so the Jetson can check the server is up."""
            if self.path != '/health':
                self.send_json(404, {'error': 'unknown address'})
                return
            self.send_json(200, {'ok': True, 'voice': engine.voice, 'speed': engine.speed,
                                 'device': engine.device, 'voices': engine.voices})

        def do_POST(self):  # noqa: N802 (name fixed by http.server)
            """Answer /speak: the body is {"text": ..., optional "voice", "speed"}."""
            if self.path != '/speak':
                self.send_json(404, {'error': 'unknown address'})
                return
            length = int(self.headers.get('Content-Length') or 0)
            try:
                if not 0 < length <= 4 * MAX_TEXT:
                    raise ValueError('body too long or empty')
                asked = json.loads(self.rfile.read(length).decode('utf-8'))
                text = ' '.join(str(asked['text']).split())
                voice = str(asked.get('voice') or engine.voice)
                speed = float(asked.get('speed') or engine.speed)
            except (ValueError, KeyError, TypeError, AttributeError) as problem:
                self.send_json(400, {'error': f'send {{"text": "a sentence"}}: {problem}'})
                return
            if not 0 < len(text) <= MAX_TEXT or not 0.5 <= speed <= 2.0:
                self.send_json(400, {'error': f'text must be 1 to {MAX_TEXT} characters, '
                                              'speed 0.5 to 2'})
                return
            try:
                wav, seconds = wav_for(text, voice, speed)
            except Exception as problem:            # unknown voice, model error: keep running
                self.send_json(500, {'error': f'could not make the voice: {problem}'})
                return
            self.send_response(200)
            self.send_header('Content-Type', 'audio/wav')
            self.send_header('Content-Length', str(len(wav)))
            self.send_header('X-Seconds', f'{seconds:.3f}')
            self.end_headers()
            self.wfile.write(wav)

        def log_message(self, fmt, *args):
            """Print one short line per request."""
            print(f'{self.address_string()} {fmt % args}', flush=True)

    return Handler


def serve(engine, port=PORT, host='0.0.0.0'):
    """Return a server (not started yet) that answers on every network address."""
    return ThreadingHTTPServer((host, port), make_handler(engine))


def main():
    """Load Kokoro and answer requests until Ctrl+C."""
    parser = argparse.ArgumentParser(description='APNVI voice server for the campus PC')
    parser.add_argument('--port', type=int, default=PORT)
    parser.add_argument('--voice', default=VOICE)
    parser.add_argument('--speed', type=float, default=SPEED)
    parser.add_argument('--device', default=DEVICE, choices=sorted(PROVIDERS))
    parser.add_argument('--folder', default=FOLDER, help="where Kokoro's two files are kept")
    args = parser.parse_args()

    print(f'Loading Kokoro, voice {args.voice}, on {args.device}...', flush=True)
    engine = KokoroEngine(args.folder, args.voice, args.speed, args.device)
    start = time.monotonic()
    engine.make('Ready', engine.voice, engine.speed)     # the first sentence is the slowest
    print(f'Runs on {engine.device}. A first test sentence took '
          f'{time.monotonic() - start:.2f} s.', flush=True)
    server = serve(engine, args.port)
    print(f'Ready. Listening on port {args.port}. Ctrl+C to stop.', flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()


if __name__ == '__main__':
    main()
