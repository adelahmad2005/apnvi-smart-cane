"""
Stage 2's voice: plays sentences in the AirPods, with Stage 1 always first.

Plain Python, nothing from ROS, so it can be tested on its own. stage2_node.py uses it.

Voice  turns a sentence into a sound file once, then plays it with paplay, at full or lower
       volume. Fixed phrases ("Listening", "Heard") are made at start-up, so later they play
       at once. Two engines (Adel, 31 Oct):
         espeak  espeak-ng on the Jetson (the robot voice, works with no network)
         kokoro  a natural human voice made on the campus PC by ai_server/voice_server.py.
                 If the PC does not answer, espeak-ng says it instead, and the PC is left
                 alone for RETRY_S, so nothing waits on a dead network
       Sound through the browser (Adel, 15 Nov): with out= set, nothing is played on
       the Jetson. Each sentence is handed to `out` as {'kind': 'play', 'file': the .wav,
       'volume': 0..1, 'sentence': ...} and a cut-off as {'kind': 'stop'}; stage2_node sends
       them on /voice/out and the 8081 page plays them on whatever computer has it open.
Mouth  decides when and how loud, from Stage 1's level (Stage 1 always comes first):
         clear, notice -> normal volume
         warning       -> lower volume, so Stage 1's beeps (always full) stand out
         stop          -> nothing plays. A sentence that is playing is cut off. An answer is
                          kept and said again once Stage 1 is below stop; after HOLD_S at
                          stop it is swapped for "Please ask again". Short feedback words
                          ("Listening", "Still thinking") are dropped instead
       The device's own sentences (16 Nov: "You have arrived", "Something is in the way.
       Waiting", "The camera is covered") are urgent: they are said at once at any level and
       are not cut off. They are often about the very thing that put Stage 1 at stop, and
       Stage 1's beeps, which the rule protected, have been off since 26 Oct.
"""

import hashlib
import json
import os
import shutil
import subprocess
import tempfile
import threading
import time
import urllib.request
import wave

VOICE_SPEED = '160'         # espeak-ng words per minute, same as Stage 1
FULL_VOLUME = 65536         # paplay's scale: 65536 = 100%
QUIET_VOLUME = 0.4          # share of full volume while Stage 1 is at warning (option 1)
HOLD_S = 10.0               # how long an answer waits during stop before "Please ask again"
ENGINES = ('espeak', 'kokoro')
VOICE_URL = 'http://localhost:8091'     # ai_server/voice_server.py; built from pc:= normally
FETCH_TIMEOUT_S = 3.0       # wait this long for the PC's voice, then espeak-ng says it
RETRY_S = 30.0              # after the PC failed, use espeak-ng only, for this long
UNPACK_TIMEOUT_S = 5.0      # remote link: unpacking one squeezed sentence takes well under this


def wav_seconds(path, unknown=2.0):
    """Give how long a .wav file plays, in seconds (a guess if it cannot be read)."""
    try:
        with wave.open(path, 'rb') as sound:
            return sound.getnframes() / float(sound.getframerate() or 1)
    except (OSError, EOFError, wave.Error):
        return unknown


def unpack_opus(data, timeout_s=UNPACK_TIMEOUT_S):
    """
    Turn the squeezed sound (Ogg Opus) the PC's front door sends into WAV bytes.

    Remote link: over the internet a 200 kB WAV took 7 to 10 s, so the front door
    sends Opus, about 12 times smaller. GStreamer, which the Jetson already has, unpacks it.
    """
    gst = shutil.which('gst-launch-1.0')
    if gst is None:
        raise ValueError('no gst-launch-1.0 to unpack the squeezed voice')
    with tempfile.TemporaryDirectory(prefix='apnvi_opus_') as folder:
        squeezed, wav_path = os.path.join(folder, 'in.ogg'), os.path.join(folder, 'out.wav')
        with open(squeezed, 'wb') as file:
            file.write(data)
        subprocess.run([gst, '-q', 'filesrc', f'location={squeezed}', '!', 'oggdemux', '!',
                        'opusdec', '!', 'audioconvert', '!', 'audioresample', '!',
                        'audio/x-raw,format=S16LE,rate=24000,channels=1', '!', 'wavenc', '!',
                        'filesink', f'location={wav_path}'],
                       check=True, timeout=timeout_s, capture_output=True)
        with open(wav_path, 'rb') as file:
            return file.read()


def fetch_from_pc(url, sentence, timeout_s=FETCH_TIMEOUT_S, unpack=unpack_opus,
                  can_unpack=None):
    """
    Ask the campus PC's voice server for a sentence; return the WAV bytes.

    When this Jetson can unpack Opus, it says so (Accept: audio/ogg). Only the PC's front door
    (remote link) acts on that; the voice server itself always sends a WAV.
    """
    if can_unpack is None:
        can_unpack = shutil.which('gst-launch-1.0') is not None
    headers = {'Content-Type': 'application/json'}
    if can_unpack:
        headers['Accept'] = 'audio/ogg, audio/wav'
    request = urllib.request.Request(url.rstrip('/') + '/speak',
                                     data=json.dumps({'text': sentence}).encode('utf-8'),
                                     headers=headers)
    with urllib.request.urlopen(request, timeout=timeout_s) as response:
        wav = response.read()
    if wav.startswith(b'OggS') and can_unpack:
        wav = unpack(wav)
    if not wav.startswith(b'RIFF'):
        raise ValueError('the voice server did not send a WAV file')
    return wav


class Voice:
    """Makes sound files (espeak-ng or the PC's voice) and plays them, one at a time."""

    def __init__(self, quiet_volume=QUIET_VOLUME, run=subprocess.Popen, make=subprocess.run,
                 folder=None, engine='espeak', url=VOICE_URL, fetch=fetch_from_pc,
                 clock=time.monotonic, out=None):
        if engine not in ENGINES:
            raise ValueError(f'voice engine must be one of {ENGINES}, not {engine!r}')
        self.espeak = shutil.which('espeak-ng')
        self.player = shutil.which('paplay')
        self.quiet_volume = quiet_volume
        self.run, self.make = run, make         # swapped for fakes in the tests
        self.engine, self.url = engine, url
        self.fetch, self.clock = fetch, clock   # also swapped for fakes in the tests
        self.pc_down_until = 0.0                # PC failed: espeak-ng only until this time
        self.folder = folder or tempfile.mkdtemp(prefix='apnvi_voice_')
        self.process = None                     # what is playing now
        self.out = out                          # set: sound goes to the browser, not paplay
        self.ends_at = 0.0                      # browser: when the sentence playing now ends
        if out is not None:
            self.player = 'browser'             # sound files are made and handed to `out`

    def ready(self):
        """Return True if this computer can speak (espeak-ng, or the PC's voice and paplay)."""
        return self.espeak is not None or (self.engine == 'kokoro' and self.player is not None)

    def play_in_browser(self, sentence, share):
        """Hand the sentence's sound file to the browser page (see the top of this file)."""
        path = self.sound_file(sentence, use_pc=False)
        if path is None:
            return
        self.out({'kind': 'play', 'file': path, 'volume': round(share, 2),
                  'sentence': sentence})
        self.ends_at = self.clock() + wav_seconds(path)

    def pc_file(self, sentence):
        """Where the PC's version of this sentence is kept."""
        key = hashlib.sha1(sentence.encode('utf-8')).hexdigest()[:16]
        return os.path.join(self.folder, 'kokoro_' + key + '.wav')

    def keep(self, sentence, wav):
        """
        Keep the PC's voice for a sentence that came with its answer; True if kept.

        Remote link, the one-trip ask: saying the sentence then needs no second trip.
        """
        if self.engine != 'kokoro' or not sentence or not wav or not wav.startswith(b'RIFF'):
            return False
        path = self.pc_file(sentence)
        part = f'{path}.{threading.get_ident()}.part'
        with open(part, 'wb') as file:
            file.write(wav)
        os.replace(part, path)                  # only a whole file gets the name
        return True

    def sound_file(self, sentence, use_pc=True):
        """
        Return a .wav file of the sentence, made the first time (None if it cannot be).

        use_pc=False never waits on the network: it takes the PC's file if it is already
        here, otherwise espeak-ng's.
        """
        key = hashlib.sha1(sentence.encode('utf-8')).hexdigest()[:16]
        if self.engine == 'kokoro':
            path = self.pc_file(sentence)
            if os.path.exists(path):
                return path
            if use_pc and self.clock() >= self.pc_down_until:
                try:
                    wav = self.fetch(self.url, sentence)
                    part = f'{path}.{threading.get_ident()}.part'
                    with open(part, 'wb') as file:
                        file.write(wav)
                    os.replace(part, path)      # only a whole file gets the name
                    return path
                except Exception:               # PC off, slow or broken: espeak-ng for now
                    self.pc_down_until = self.clock() + RETRY_S
        if self.espeak is None:
            return None
        path = os.path.join(self.folder, key + '.wav')
        if os.path.exists(path):
            return path
        try:
            self.make([self.espeak, '-s', VOICE_SPEED, '-w', path, sentence],
                      check=True, timeout=5, capture_output=True)
        except Exception:                       # could not make the file: speak directly
            return None
        return path if os.path.exists(path) else None

    def prepare_soon(self, sentence):
        """Fetch the PC's version in the background, if missing, so a later play has it."""
        if (self.engine == 'kokoro' and not os.path.exists(self.pc_file(sentence))
                and self.clock() >= self.pc_down_until):
            threading.Thread(target=self.sound_file, args=(sentence,), daemon=True).start()

    def play(self, sentence, quiet=False):
        """Stop whatever is playing and play this sentence, at lower volume if quiet."""
        self.stop()
        if not self.ready():
            return
        share = self.quiet_volume if quiet else 1.0
        if self.out is not None:
            self.play_in_browser(sentence, share)
            return
        path = self.sound_file(sentence, use_pc=False) if self.player else None
        if path:
            command = [self.player, f'--volume={int(FULL_VOLUME * share)}', path]
        elif self.espeak:                       # no paplay: espeak-ng speaks it directly
            command = [self.espeak, '-s', VOICE_SPEED, '-a', str(int(100 * share)), sentence]
        else:
            return
        self.process = self.run(command, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

    def playing(self):
        """Return True while a sentence is still playing."""
        if self.out is not None:
            return self.clock() < self.ends_at
        return self.process is not None and self.process.poll() is None

    def stop(self):
        """Cut off the sentence that is playing, if any."""
        if self.out is not None:
            if self.playing():
                self.out({'kind': 'stop'})
            self.ends_at = 0.0
            return
        if self.playing():
            self.process.terminate()
        self.process = None


class Mouth:
    """Says Stage 2's sentences, with Stage 1 always first (see the top of this file)."""

    def __init__(self, voice, ask_again, hold_s=HOLD_S, clock=time.monotonic):
        self.voice = voice                      # a Voice, or None when the voice is off
        self.ask_again = ask_again              # said instead of an answer held too long
        self.hold_s = hold_s
        self.clock = clock
        self.lock = threading.Lock()            # called from ROS callbacks and answer threads
        self.level = None                       # Stage 1's latest level
        self.current = None                     # (sentence, keep, urgent) playing now
        self.held = None                        # answer waiting for Stage 1 to leave stop
        self.held_since = 0.0

    def say(self, sentence, keep=True, urgent=False):
        """
        Say a sentence now, quietly, or later, depending on Stage 1's level.

        keep=True for answers (held during stop), False for feedback words (dropped).
        urgent=True for the device's own sentences: said now at any level, never cut off.
        Returns what happened: 'normal', 'quiet', 'held', 'dropped' or 'off'.
        """
        if not sentence:
            return 'dropped'
        if self.voice is not None and self.voice.player:
            if keep:                            # answers: get the PC's voice first, outside
                self.voice.sound_file(sentence)  # the lock, so Stage 1's levels never wait
            else:                               # feedback words never wait on the network
                self.voice.prepare_soon(sentence)
        with self.lock:
            if urgent:
                self.held = None                # it replaces an answer that was waiting
                return self._play(sentence, keep, urgent=True)
            if self.level == 'stop':
                if not keep:
                    return 'dropped'
                self.held, self.held_since = sentence, self.clock()
                return 'held'
            if self.current and self.current[2] and self.voice is not None \
                    and self.voice.playing():
                return 'dropped' if not keep else self._queue(sentence)
            return self._play(sentence, keep)

    def set_level(self, level):
        """Take Stage 1's newest level (about 10 times a second)."""
        with self.lock:
            self.level = level
            urgent_now = bool(self.current and self.current[2] and self.voice is not None
                              and self.voice.playing())
            if level == 'stop' and urgent_now:
                pass                            # the device's own sentence: let it finish
            elif level == 'stop':
                if self.voice is not None and self.voice.playing():
                    self.voice.stop()           # stop always wins
                    if self.current and self.current[1] and self.held is None:
                        self.held, self.held_since = self.current[0], self.clock()
                self.current = None
            elif self.held is not None and urgent_now:
                pass                            # wait for the urgent sentence to end
            elif self.held is not None:         # stop is over: say the waiting answer now
                sentence, self.held = self.held, None
                self._play(sentence, True)

    def tick(self):
        """Call regularly: an answer held at stop for too long becomes "Please ask again"."""
        with self.lock:
            if (self.held is not None and self.held != self.ask_again
                    and self.clock() - self.held_since > self.hold_s):
                self.held = self.ask_again
            if self.held is not None and self.level != 'stop' and not (
                    self.voice is not None and self.voice.playing()):
                sentence, self.held = self.held, None       # an answer that waited for an
                self._play(sentence, True)                  # urgent sentence to end

    def _queue(self, sentence):
        """Keep an answer until the urgent sentence playing now has ended (see tick)."""
        self.held, self.held_since = sentence, self.clock()
        return 'held'

    def _play(self, sentence, keep, urgent=False):
        """Play now, quietly while Stage 1 is at warning. The caller holds the lock."""
        if self.voice is None or not self.voice.ready():
            return 'off'
        quiet = self.level == 'warning' and not urgent
        self.voice.play(sentence, quiet=quiet)
        self.current = (sentence, keep, urgent)
        return 'quiet' if quiet else 'normal'
