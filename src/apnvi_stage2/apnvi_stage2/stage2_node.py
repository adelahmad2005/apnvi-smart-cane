"""
stage2_node: the user holds the button, asks for something, and gets a red box or a sentence.

While the button is held, the microphone records. When it is let go, the node takes the
latest colour picture, sends the recording and the picture to the campus PC. Qwen first sorts
the question (Adel, 26 Oct): "where is a water bottle" is FIND, "what is the nearest sign" is
ANSWER.
  FIND, found      -> publishes the picture with the red box drawn on it, stays silent
  FIND, not found  -> says "I can't see it, turn slowly and try again"
  ANSWER           -> says Qwen's short sentence, with a red box on what it is about if any
The work runs in the background, so Stage 1's warnings never wait for it.
What the user hears (voice plan, Adel, 26 Oct, all in the AirPods): "Listening" when the
button is pressed, "Heard" when the words come back, "Still thinking" at about 4 s and 10 s,
"Sorry, that took too long. Please ask again" at 30 s, then the answer.
Stage 1 always comes first (voice.py): at "warning" Stage 2's voice is quieter and the beeps
stay at full volume; at "stop" Stage 2 is cut off and the answer is said once stop is over
("Please ask again" if that takes more than 10 s). The box and the result are always sent.
One question at a time: a press while a question is being answered is ignored.

Tracking (demo screen, added 2026-10-12 at Adel's request): a question sent on /stage2/track is
asked again and again, each time with the newest picture, until an empty one stops it. The
browser view draws the newest box on the live picture. Tracking stays silent and always finds
(no sorting).

Stage 4 (12 Nov): the grip's red button is a second talk button (/grip/red, from the drive
link); the GPIO 23 button keeps working. Pressed while the guide is guiding, the user hears
"Stopping. Listening" (the master has already stopped the wheels). After each question the
guide node gets a line on /stage2/request: "take me to ..." gives it a goal, a follow-up
("continue", "how far") carries on, anything else drops a paused route. The guide's own
sentences ("You have arrived") come in on /voice/say and are said like answers.

Things on the map (16 Nov, Adel: "take me to the water bottle"): a found thing is placed on
the map from its box, the depth picture and the device's position (locate.py), and pinned
through Stage 3's map manager, so the guide can take the user to it. The guide asks for a
thing that is not on the map yet on /stage2/locate and gets the answer on /stage2/located.
Map commands by voice ("save this place as ...", "scan the room") go to /stage3/command.

Sound through the browser (15 Nov; sound_out:=browser): nothing plays on the Jetson.
Sentences go out on /voice/out and the 8081 page plays them on whatever computer has it open.
That computer's microphone records while the talk button is held: the page is told to start
and stop on /voice/out, and its recording comes back as a file named on /voice/recording.

Listens: /button/pressed   Bool    from esp32_reader (True = held, False = let go)
         /grip/red         Bool    the grip's red button, from Stage 4's drive link (same use)
         /voice/say        String  a sentence to say, from Stage 4's guide node
         /stage4/guide_state String JSON from the guide node, to know if the user is guided
         /voice/question   String  a typed question, for testing without a microphone:
             ros2 topic pub --once /voice/question std_msgs/String "{data: 'where is the exit'}"
         /stage2/track     String  keep asking this question; '' stops (sent by the browser view)
         /hazard/level     String  Stage 1's level, so Stage 2 stays quiet during a warning
         /stage3/position  PoseStamped  where the device is on the map
         /stage3/placement String  placed / searching / lost after a map was loaded
         /stage2/locate    String  JSON {"name", "id"} from the guide: find this thing now
         /voice/recording  String  a WAV file recorded by the browser page (sound_out browser)
         the depth picture and both camera infos, only while a thing is being placed (eye.py)
         colour picture    Image   from the RealSense (setting color_topic)
Sends:   /target/result    String  JSON: question, found, box [left, top, right, bottom],
                                    label, seconds, said, problem, size [width, height] of
                                    the picture the box belongs to, tracking (true/false),
                                    stamp (camera time of that picture, in seconds),
                                    answer (Qwen's raw reply, first 300 characters),
                                    kind ('find' or 'answer')
         /target/image     Image   the picture the question was asked about, with the box
         /stage2/status    String  listening / thinking / ready, for the live view
         /stage2/request   String  JSON for Stage 4's guide node after each question:
                                    {"kind": "guide", "place": ...}, {"kind": "follow_up"}
                                    or {"kind": "other"} (asking.request_for)
         /stage2/located   String  JSON answer to /stage2/locate: id, name, found, x, y,
                                    from_x, from_y, distance_m, problem (not_seen, no_depth,
                                    too_far, no_position, moved, no_camera, no_helper, busy)
         /stage3/command   String  pins for the map ({"cmd": "pin", ...}) and map commands
         /voice/out        String  sound_out browser: JSON events for the 8081 page:
                                    play (file, volume, sentence), stop, listen (on)

Settings (--ros-args -p name:=value):
  pc            the campus PC's IP address. When set, ollama_url, speech_url and voice_url
                are built from it (ports 11434, 8090 and 8091), so this is the only one to change.
                Or the PC's front door over the internet (remote link), a full address
                with the token: https://<tunnel address>/k/<token> (ai_server/front_door.py)
  ollama_url    where Qwen runs, default http://localhost:11434
  speech_url    where the speech server runs, default http://localhost:8090
  model         default qwen3-vl:4b-instruct
  color_topic   default /camera/camera/color/image_raw
  mic           microphone for arecord, default 'default'
  voice         true = speak the sentences, default true
  voice_engine  espeak = the Jetson's espeak-ng (default); kokoro = the natural voice made on
                the campus PC (ai_server/voice_server.py), espeak-ng if the PC does not answer
  voice_url     where the voice server runs, default http://localhost:8091
  quiet_volume  share of full volume for Stage 2's voice while Stage 1 is at warning,
                default 0.4
  sound_out     jetson = play and record on the Jetson (default); browser = through the 8081
                page
  depth_topic, depth_info_topic, color_info_topic   the camera's depth picture and infos
  camera_back_m how far the camera sits behind the wheels' centre, default 0.70
"""

from dataclasses import replace
import json
import math
import os
import shutil
import signal
import subprocess
import tempfile
import threading
import time

from apnvi_stage1.hazard_node import image_to_depth_mm, NEWEST_PICTURE
from geometry_msgs.msg import PoseStamped
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Bool, String

from .asking import (answer_recording, answer_typed, ASK_AGAIN, HEARD, LISTENING, NO_CAMERA,
                     NO_HELPER, NOT_FOUND, NOT_HEARD, on_the_map, Outcome, request_for,
                     STILL_THINKING, STOPPING, TOO_LONG, unreachable)
from .eye import Eye
from .finder import draw_box, find, MODEL, OLLAMA_URL, route
from .locate import (CAMERA_BACK_M, lens_from_info, locate, Located, NO_DEPTH, same_spot)
from .speech import SPEECH_URL
from .remote_ask import ask_front_door, NoAsk
from .voice import ENGINES, Mouth, QUIET_VOLUME, Voice, VOICE_URL

MIN_HOLD_S = 0.3            # shorter than this = an accidental tap, ignored
MAX_HOLD_S = 15.0           # stop recording after this, even if still held
STILL_THINKING_S = (4.0, 10.0)   # say "Still thinking" when the wait passes each of these
GIVE_UP_S = 30.0            # then "took too long", and the late answer is not spoken
GUIDED_RECENTLY_S = 1.0     # the guide said "guiding" this recently: a press stops a trip
POSE_FRESH_S = 1.0          # a map position older than this is not used to place a thing
PLACED_STATES = ('', 'placed')      # Stage 3 knows where the device is ('' = drawing a new map)
BROWSER_WAIT_S = 4.0        # after the button is let go, wait this long for the page's recording
LOCATE_WAIT_S = 10.0        # the guide's "find this thing" waits this long for a running answer
SOUND_OUTS = ('jetson', 'browser')
READY_SENTENCES = (LISTENING, HEARD, STILL_THINKING, TOO_LONG, ASK_AGAIN, NOT_FOUND,
                   NOT_HEARD, NO_HELPER, NO_CAMERA,
                   STOPPING + '. ' + LISTENING)         # made into sound files at start-up


def press_words(sentence, guided):
    """Give what a talk-button press says: "Stopping. " first while the user is guided."""
    return STOPPING + '. ' + sentence if guided else sentence


def waiting_words(waited_s, times_said):
    """
    Return (sentence or None, new times_said) for a question waiting this long.

    "Still thinking" once at each of STILL_THINKING_S, then TOO_LONG once at GIVE_UP_S.
    """
    if times_said < len(STILL_THINKING_S) and waited_s >= STILL_THINKING_S[times_said]:
        return STILL_THINKING, times_said + 1
    if times_said == len(STILL_THINKING_S) and waited_s >= GIVE_UP_S:
        return TOO_LONG, times_said + 1
    return None, times_said


def find_every(image, question, url, model):
    """Tracking's finder: asks Qwen for every match, so the follower can keep its own one."""
    return find(image, question, url=url, model=model, every=True)


def image_to_bgr(msg):
    """Turn a ROS colour picture (rgb8 or bgr8) into a numpy grid in OpenCV's order."""
    if msg.encoding not in ('rgb8', 'bgr8'):
        raise ValueError(f'expected an rgb8 or bgr8 picture, got "{msg.encoding}"')
    grid = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.step)
    grid = grid[:, :msg.width * 3].reshape(msg.height, msg.width, 3)
    return grid[:, :, ::-1].copy() if msg.encoding == 'rgb8' else grid.copy()


def bgr_to_image(picture, stamp=None):
    """Turn a numpy picture in OpenCV's order into a ROS bgr8 picture."""
    msg = Image()
    msg.height, msg.width = picture.shape[:2]
    msg.encoding = 'bgr8'
    msg.step = msg.width * 3
    msg.data = np.ascontiguousarray(picture, dtype=np.uint8).tobytes()
    if stamp is not None:
        msg.header.stamp = stamp
    return msg


def yaw_of(q):
    """Give the way the device faces (radians, anticlockwise from x) from a quaternion."""
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def pin_command(name, located, kind='thing', sure=True):
    """Make the /stage3/command line that pins a located thing on the map."""
    return {'cmd': 'pin', 'name': name, 'kind': kind, 'sure': sure,
            'x': round(located.x, 3), 'y': round(located.y, 3),
            'from_x': round(located.from_x, 3), 'from_y': round(located.from_y, 3)}


def located_json(ident, name, located):
    """Pack the answer to the guide's "find this thing" for /stage2/located."""
    return json.dumps({
        'id': ident, 'name': name, 'found': bool(located.found),
        'x': round(located.x, 3), 'y': round(located.y, 3),
        'from_x': round(located.from_x, 3), 'from_y': round(located.from_y, 3),
        'distance_m': round(located.distance_m, 2), 'problem': located.problem})


def hide_token(url):
    """Hide a front door token in an address, for the log: .../k/***/..."""
    head, marker, rest = str(url).partition('/k/')
    if not marker:
        return url
    _token, slash, tail = rest.partition('/')
    return head + '/k/***' + slash + tail


def front_door(pc):
    """Give the PC's front door address (remote link) if pc is one, else ''."""
    return pc.rstrip('/') if '://' in pc else ''


def voice_url_for_pc(pc):
    """Build the voice server's address from the campus PC's IP address or its front door."""
    door = front_door(pc)
    return f'{door}/voice' if door else f'http://{pc}:8091'


def urls_for_pc(pc):
    """Build the Qwen and speech server addresses from the campus PC's IP address or front door."""
    door = front_door(pc)
    if door:
        return f'{door}/ai', f'{door}/speech'
    return f'http://{pc}:11434', f'http://{pc}:8090'


def stamp_of(msg):
    """Return a ROS picture's camera time in seconds, or None."""
    try:
        return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
    except AttributeError:
        return None


def result_json(outcome, tracking=False, stamp=None):
    """Pack an Outcome into the JSON sent on /target/result."""
    result = outcome.result
    picture = outcome.picture
    return json.dumps({
        'question': outcome.question,
        'found': bool(result and result.found),
        'box': list(result.box) if result and result.found else None,
        'boxes': [list(box) for box in result.boxes] if result and result.found else [],
        'label': result.label if result else '',
        'seconds': round(result.seconds, 2) if result else None,
        'said': outcome.say,
        'problem': outcome.problem,
        'size': [int(picture.shape[1]), int(picture.shape[0])] if picture is not None else None,
        'tracking': tracking,
        'stamp': stamp,
        'answer': (result.answer or '')[:300] if result else '',
        'kind': outcome.kind,
    })


class Recorder:
    """Records the microphone with arecord into a WAV file, between start() and stop()."""

    def __init__(self, mic='default'):
        self.mic = mic
        self.process = None
        self.path = None
        self.started = 0.0

    def start(self):
        """Start recording: 16 kHz, mono, 16-bit, which is what Whisper wants."""
        handle, self.path = tempfile.mkstemp(prefix='apnvi_question_', suffix='.wav')
        os.close(handle)
        self.process = subprocess.Popen(
            ['arecord', '-q', '-D', self.mic, '-f', 'S16_LE', '-r', '16000', '-c', '1',
             '-t', 'wav', self.path], stdout=subprocess.DEVNULL, stderr=subprocess.PIPE)
        self.started = time.monotonic()

    def stop(self):
        """Stop recording. Returns (WAV bytes, seconds held), or (None, seconds) on failure."""
        held = time.monotonic() - self.started
        if self.process is None:
            return None, held
        self.process.send_signal(signal.SIGINT)        # arecord finishes the file cleanly
        try:
            self.process.wait(timeout=2)
        except subprocess.TimeoutExpired:
            self.process.kill()
        self.process = None
        try:
            with open(self.path, 'rb') as file:
                data = file.read()
        except OSError:
            data = None
        finally:
            if os.path.exists(self.path):
                os.remove(self.path)
        return (data if data and len(data) > 44 else None), held   # 44 = empty WAV header


class Stage2Node(Node):
    """Hold the button, ask, get a red box or a sentence."""

    def __init__(self):
        super().__init__('stage2_node')
        self.declare_parameter('pc', '')
        self.declare_parameter('ollama_url', OLLAMA_URL)
        self.declare_parameter('speech_url', SPEECH_URL)
        self.declare_parameter('model', MODEL)
        self.declare_parameter('color_topic', '/camera/camera/color/image_raw')
        self.declare_parameter('mic', 'default')
        self.declare_parameter('voice', True)
        self.declare_parameter('quiet_volume', QUIET_VOLUME)
        self.declare_parameter('voice_engine', 'espeak')
        self.declare_parameter('voice_url', VOICE_URL)
        self.declare_parameter('sound_out', 'jetson')
        self.declare_parameter('depth_topic', '/camera/camera/depth/image_rect_raw')
        self.declare_parameter('depth_info_topic', '/camera/camera/depth/camera_info')
        self.declare_parameter('color_info_topic', '/camera/camera/color/camera_info')
        self.declare_parameter('camera_back_m', CAMERA_BACK_M)
        self.camera_back_m = float(self.get_parameter('camera_back_m').value)
        self.sound_out = str(self.get_parameter('sound_out').value).strip().lower()
        if self.sound_out not in SOUND_OUTS:
            self.get_logger().warn(f'sound_out {self.sound_out!r} unknown: using jetson')
            self.sound_out = 'jetson'
        self.browser = self.sound_out == 'browser'
        self.sound_seq = 0
        self.sound_lock = threading.Lock()
        self.sound_pub = self.create_publisher(String, '/voice/out', 20)
        self.ollama_url = self.get_parameter('ollama_url').value
        self.speech_url = self.get_parameter('speech_url').value
        self.model = self.get_parameter('model').value
        pc = str(self.get_parameter('pc').value).strip()
        voice_url = self.get_parameter('voice_url').value
        if pc:
            self.ollama_url, self.speech_url = urls_for_pc(pc)
            voice_url = voice_url_for_pc(pc)
        self.door = front_door(pc)              # remote link: one trip per question
        if self.door:
            self.get_logger().info('Remote link: questions go in one trip through the PC\'s '
                                   f'front door {hide_token(self.door)}')
        voice = None
        if bool(self.get_parameter('voice').value):
            engine = str(self.get_parameter('voice_engine').value).strip().lower()
            if engine not in ENGINES:
                self.get_logger().warn(f'voice_engine {engine!r} unknown: using espeak')
                engine = 'espeak'
            voice = Voice(float(self.get_parameter('quiet_volume').value),
                          engine=engine, url=voice_url,
                          out=self.sound_event if self.browser else None)
            if self.browser:
                self.get_logger().info('Sound goes to the browser page (8081), not the Jetson')
            if engine == 'kokoro':
                self.get_logger().info(f'Voice: Kokoro from {hide_token(voice_url)}, '
                                       'espeak-ng if it fails')
            if not voice.ready():
                self.get_logger().warn('espeak-ng not found: sentences will only be printed')
            elif not voice.player:
                self.get_logger().warn('paplay not found: espeak-ng speaks directly')
            else:
                for sentence in READY_SENTENCES:       # so feedback plays at once later
                    voice.sound_file(sentence)
        self.mouth = Mouth(voice, ASK_AGAIN)
        self.voice = voice
        self.kokoro = voice is not None and voice.engine == 'kokoro'
        self.asking = None                      # [question number, started, times waited]
        self.question_no = 0

        self.recorder = Recorder(self.get_parameter('mic').value)
        self.record_started = 0.0               # browser: when the talk button went down
        self.pending = None                     # browser: (picture, stamp, snapshot, deadline)
        if not self.browser and not shutil.which('arecord'):
            self.get_logger().warn('arecord not found: the button cannot record. '
                                   'Typed questions on /voice/question still work')
        self.latest = None                      # latest colour picture message
        self.busy = threading.Lock()            # one question at a time
        self.recording = False
        self.track_question = ''                # '' = not tracking

        self.result_pub = self.create_publisher(String, '/target/result', 10)
        self.image_pub = self.create_publisher(Image, '/target/image', 1)
        self.status_pub = self.create_publisher(String, '/stage2/status', 10)
        self.request_pub = self.create_publisher(String, '/stage2/request', 10)
        self.command_pub = self.create_publisher(String, '/stage3/command', 10)
        self.located_pub = self.create_publisher(String, '/stage2/located', 10)
        self.pose, self.pose_at, self.placement = None, -1e9, ''
        self.eye = Eye(self, {
            'depth': (Image, self.get_parameter('depth_topic').value, NEWEST_PICTURE),
            'depth_info': (CameraInfo, self.get_parameter('depth_info_topic').value,
                           NEWEST_PICTURE),
            'colour_info': (CameraInfo, self.get_parameter('color_info_topic').value,
                            NEWEST_PICTURE)})
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(PoseStamped, '/stage3/position', self.on_position, 10)
        self.create_subscription(String, '/stage3/placement', self.on_placement, latched)
        self.create_subscription(String, '/stage2/locate', self.on_locate, 10)
        self.create_subscription(String, '/voice/recording', self.on_recording, 10)
        self.map_pins = []                      # the map's pins, to answer "where is ..."
        self.create_subscription(String, '/stage3/pins', self.on_pins, latched)
        self.guided_at = -1e9                   # when the guide last said "guiding"
        self.create_subscription(Image, self.get_parameter('color_topic').value,
                                 self.on_picture, NEWEST_PICTURE)
        self.create_subscription(Bool, '/button/pressed', self.on_button, 10)
        self.create_subscription(Bool, '/grip/red', self.on_button, 10)
        self.create_subscription(String, '/voice/say', self.on_say, 10)
        self.create_subscription(String, '/stage4/guide_state', self.on_guide_state, 10)
        self.create_subscription(String, '/voice/question', self.on_typed, 10)
        self.create_subscription(String, '/stage2/track', self.on_track, 10)
        self.stage1_level = None                # Stage 1's latest level
        self.create_subscription(String, '/hazard/level', self.on_level, 10)
        self.create_timer(0.5, self.check_long_hold)
        self.create_timer(0.5, self.check_pending)
        self.create_timer(0.5, self.check_waiting)
        self.get_logger().info(f'Ready. Qwen at {hide_token(self.ollama_url)}, '
                               f'speech at {hide_token(self.speech_url)}')

    # ---- sound through the browser ------------------------------------------

    def sound_event(self, event):
        """Send one event to the 8081 page: play a file, stop, or start/stop listening."""
        with self.sound_lock:
            self.sound_seq += 1
            event = dict(event, seq=self.sound_seq)
        self.sound_pub.publish(String(data=json.dumps(event)))

    def check_pending(self):
        """Browser: give up on a recording that never came (no page open, no microphone)."""
        pending = self.pending
        if pending is not None and time.monotonic() > pending[3]:
            self.pending = None
            self.status_pub.publish(String(data='ready'))
            self.get_logger().error('No recording came from a browser page: is the 8081 page '
                                    'open with "Sound on", at http://localhost:8081?')
            self.say(NOT_HEARD, keep=False)

    def on_recording(self, msg):
        """Browser: the page's recording arrived as a WAV file; answer it."""
        path = msg.data.strip()
        pending, self.pending = self.pending, None
        try:
            with open(path, 'rb') as file:
                wav = file.read()
        except OSError:
            wav = None
        finally:
            if path.startswith(tempfile.gettempdir()) and os.path.exists(path):
                os.remove(path)
        if pending is None:
            return                              # a second page sent the same question: ignore
        image, stamp, snapshot, _deadline = pending
        if not wav or len(wav) <= 44:
            self.status_pub.publish(String(data='ready'))
            self.get_logger().error('The browser sent an empty recording: is its microphone '
                                    'allowed?')
            self.say(NOT_HEARD, keep=False)
            return
        self.start_answer(lambda: self.answer_spoken(wav, image), stamp=stamp,
                          snapshot=snapshot)

    # ---- where the device is, and placing a thing on the map (16 Nov) ---------------------

    def on_position(self, msg):
        position = msg.pose.position
        self.pose = (position.x, position.y, yaw_of(msg.pose.orientation))
        self.pose_at = time.monotonic()

    def on_placement(self, msg):
        self.placement = msg.data

    def on_pins(self, msg):
        try:
            self.map_pins = list(json.loads(msg.data).get('pins', []))
        except (ValueError, AttributeError):
            pass

    def pose_now(self):
        """Give the device's (x, y, yaw) on the map, or None if Stage 3 does not know it."""
        if time.monotonic() - self.pose_at > POSE_FRESH_S or self.placement not in PLACED_STATES:
            return None
        return self.pose

    def snapshot(self):
        """Note where the device is as a question's picture is taken, and wake the depth."""
        self.eye.want()
        return {'pose': self.pose_now()}

    def place_on_map(self, box, size, pose):
        """
        Place a box of the colour picture (size = width, height) on the map: a Located.

        Blocks up to 1.5 s for a depth picture: call it from an answer thread only.
        pose: where the device was when the picture was taken. If it has moved since, the
        depth picture no longer matches the box and nothing is placed.
        """
        got = self.eye.wait(('depth',), timeout_s=1.5, max_age_s=1.0)
        if got is None:
            return Located(False, problem=NO_DEPTH)
        if pose is not None and not same_spot(pose, self.pose_now()):
            return Located(False, problem='moved')
        try:
            depth_mm = image_to_depth_mm(got['depth'])
        except ValueError as problem:
            self.get_logger().error(str(problem), throttle_duration_sec=10.0)
            return Located(False, problem=NO_DEPTH)
        lenses = []
        for name in ('colour_info', 'depth_info'):
            info = self.eye.get(name, max_age_s=5.0)
            lenses.append(None if info is None else
                          lens_from_info(info.k, info.width, info.height))
        return locate(box, size, depth_mm, pose, lenses[0], lenses[1], self.camera_back_m)

    def pin(self, name, located, kind='thing'):
        """Pin a located thing on Stage 3's map."""
        self.command_pub.publish(String(data=json.dumps(pin_command(name, located, kind))))
        self.get_logger().info('pinned "%s" at (%.2f, %.2f), %.1f m from the camera' % (
            name, located.x, located.y, located.distance_m))

    def on_locate(self, msg):
        """Take the guide's request: find this thing in the camera picture now."""
        try:
            request = json.loads(msg.data)
            name, ident = str(request['name']).strip(), request.get('id')
        except (ValueError, KeyError, TypeError):
            self.get_logger().warn('bad locate request: %r' % msg.data)
            return
        self.eye.want()
        threading.Thread(target=self.locate_thing, args=(name, ident), daemon=True).start()

    def locate_thing(self, name, ident):
        """Find `name` in the newest picture, place it, pin it, and answer the guide."""
        if not self.busy.acquire(timeout=LOCATE_WAIT_S):
            self.located_pub.publish(String(data=located_json(
                ident, name, Located(False, problem='busy'))))
            return
        located = Located(False, problem='not_seen')
        try:
            self.status_pub.publish(String(data='thinking'))
            image, stamp, pose = self.latest_bgr(), stamp_of(self.latest), self.pose_now()
            if image is None:
                located = Located(False, problem='no_camera')
                return
            result = find(image, 'where is the %s' % name, url=self.ollama_url,
                          model=self.model)
            if not result.found:
                if unreachable(result.error):
                    located = Located(False, problem='no_helper')
                return
            picture = draw_box(image, result.box, result.label)
            self.result_pub.publish(String(data=result_json(
                Outcome('take me to the %s' % name, '', result, picture), stamp=stamp)))
            self.image_pub.publish(bgr_to_image(picture))
            located = self.place_on_map(result.box, (image.shape[1], image.shape[0]), pose)
            if located.found:
                self.pin(name, located)
        except Exception as problem:            # never let one question kill the node
            self.get_logger().error(f'Locating failed: {problem}')
            located = Located(False, problem='no_helper')
        finally:
            self.get_logger().info('locate "%s": %s' % (
                name, 'found' if located.found else located.problem))
            self.located_pub.publish(String(data=located_json(ident, name, located)))
            if not self.track_question:
                self.status_pub.publish(String(data='ready'))
            self.busy.release()
            self.ask_track()

    def on_level(self, msg):
        """Remember Stage 1's level (clear, notice, warning, stop); the voice follows it."""
        self.stage1_level = msg.data
        self.mouth.set_level(msg.data)

    def say(self, sentence, keep=True, urgent=False):
        """Say a sentence through the Mouth (Stage 1 first) and log what happened."""
        how = self.mouth.say(sentence, keep, urgent)
        if how in ('held', 'dropped', 'quiet'):
            self.get_logger().info(f'Stage 1 is at "{self.stage1_level}": "{sentence}" {how}')

    def check_waiting(self):
        """Say "Still thinking" or "took too long" while a question waits for its answer."""
        self.mouth.tick()
        if self.asking is None:
            return
        number, started, times = self.asking
        sentence, times = waiting_words(time.monotonic() - started, times)
        if sentence and self.asking is not None and self.asking[0] == number:
            self.asking = [number, started, times]
            self.say(sentence, keep=sentence == TOO_LONG)

    def on_say(self, msg):
        """
        Say one of the device's own sentences ("You have arrived", "Something is in the way").

        They come from the guide, the map manager, the labeller and the health program.

        They are urgent: said at once whatever Stage 1's level, and never cut off (voice.py).
        """
        self.say(msg.data.strip(), urgent=True)

    def on_guide_state(self, msg):
        """Remember when the guide last said it is guiding."""
        try:
            if json.loads(msg.data).get('state') == 'guiding':
                self.guided_at = time.monotonic()
        except (ValueError, AttributeError):
            pass

    def guided(self):
        """Say if the user was being guided just now (a press stops the trip)."""
        return time.monotonic() - self.guided_at <= GUIDED_RECENTLY_S

    def on_picture(self, msg):
        """Keep only the newest colour picture."""
        self.latest = msg

    def latest_bgr(self):
        """Return the newest colour picture as numpy, or None if there is none yet."""
        if self.latest is None:
            return None
        try:
            return image_to_bgr(self.latest)
        except ValueError as problem:
            self.get_logger().error(str(problem), throttle_duration_sec=10.0)
            return None

    def on_button(self, msg):
        """Button held: start recording. Let go: stop and answer."""
        if msg.data and not self.recording:
            if self.busy.locked():
                self.get_logger().info('Still answering the last question: press ignored')
                self.say(press_words(STILL_THINKING, self.guided()), keep=False)  # never silent
                return
            self.eye.want()                     # the depth picture is ready by the answer
            if self.browser:                    # the page's microphone records, not arecord
                self.record_started = time.monotonic()
            else:
                try:
                    self.recorder.start()
                except OSError as problem:
                    self.get_logger().error(f'Cannot start the microphone: {problem}')
                    return
            self.recording = True
            self.status_pub.publish(String(data='listening'))
            self.get_logger().info('Listening...')
            self.say(press_words(LISTENING, self.guided()), keep=False)
            if self.browser:
                self.sound_event({'kind': 'listen', 'on': True})
        elif not msg.data and self.recording:
            self.finish_recording()

    def check_long_hold(self):
        """Stop recording if the button has been held for too long."""
        started = self.record_started if self.browser else self.recorder.started
        if self.recording and time.monotonic() - started > MAX_HOLD_S:
            self.get_logger().info(f'Held for over {MAX_HOLD_S:.0f} s: answering now')
            self.finish_recording()

    def finish_recording(self):
        """Stop the microphone and answer in the background."""
        self.recording = False
        image = self.latest_bgr()               # the picture at the moment of letting go
        stamp = stamp_of(self.latest)
        snapshot = self.snapshot()
        if self.browser:
            held = time.monotonic() - self.record_started
            self.sound_event({'kind': 'listen', 'on': False, 'send': held >= MIN_HOLD_S})
            if held < MIN_HOLD_S:
                self.status_pub.publish(String(data='ready'))
                self.get_logger().info('Button tapped too briefly: ignored')
                return
            self.pending = (image, stamp, snapshot, time.monotonic() + BROWSER_WAIT_S)
            return                              # on_recording carries on when the file comes
        wav, held = self.recorder.stop()
        if held < MIN_HOLD_S:
            self.status_pub.publish(String(data='ready'))
            self.get_logger().info('Button tapped too briefly: ignored')
            return
        if wav is None:
            self.status_pub.publish(String(data='ready'))
            self.get_logger().error('Nothing was recorded: is the microphone plugged in?')
            return
        self.start_answer(lambda: self.answer_spoken(wav, image), stamp=stamp,
                          snapshot=snapshot)

    def answer_spoken(self, wav, image):
        """Answer a recording: one trip through the front door (remote link), or as before."""
        def heard(_words):
            self.say(HEARD, keep=False)
        if self.door:
            outcome = self.ask_door(image, wav=wav, on_heard=heard)
            if outcome is not None:
                return outcome
        return answer_recording(wav, image, self.speech_url, self.ollama_url, self.model,
                                router=route, on_heard=heard)

    def answer_text(self, question, image):
        """Answer typed words: one trip through the front door (remote link), or as before."""
        if self.door:
            outcome = self.ask_door(image, text=question)
            if outcome is not None:
                return outcome
        return answer_typed(question, image, self.ollama_url, self.model, router=route)

    def ask_door(self, image, wav=None, text=None, on_heard=None):
        """
        Ask the whole question in one trip through the PC's front door (remote_ask.py).

        None if the front door has no /ask: then the caller asks in four trips, as on the
        hotspot. The answer's voice comes in the same trip and is kept, so saying it waits on
        nothing. "Heard" is said only when Stage 2 says nothing else: the answer comes straight
        after the words now, and would cut "Heard" off half way.
        """
        try:
            outcome, sound = ask_front_door(self.door, image, self.model, wav=wav, text=text,
                                            voice=self.kokoro)
        except NoAsk:
            self.get_logger().warn('The front door has no /ask: asking in four trips')
            return None
        if sound and outcome.say and self.voice is not None:
            self.voice.keep(outcome.say, sound)
        if on_heard is not None and outcome.question and not outcome.say:
            on_heard(outcome.question)
        return outcome

    def on_typed(self, msg):
        """Answer a typed question (testing without a microphone)."""
        if self.busy.locked():
            self.get_logger().info('Still answering the last question: typed one ignored')
            return
        image, stamp = self.latest_bgr(), stamp_of(self.latest)
        self.start_answer(lambda: self.answer_text(msg.data, image), stamp=stamp,
                          snapshot=self.snapshot())

    def on_track(self, msg):
        """Start tracking this question, or stop tracking if it is empty."""
        self.track_question = msg.data.strip()
        if self.track_question:
            self.get_logger().info(f'Tracking "{self.track_question}"')
            self.ask_track()
        else:
            self.get_logger().info('Tracking stopped')
            self.status_pub.publish(String(data='ready'))

    def ask_track(self):
        """Ask the tracked question once more, with the newest picture, if nothing else runs."""
        question = self.track_question
        if not question or self.busy.locked():
            return                              # the running answer starts the next one
        image, stamp = self.latest_bgr(), stamp_of(self.latest)
        self.start_answer(lambda: answer_typed(question, image, self.ollama_url, self.model,
                                               finder=find_every),
                          tracking=True, stamp=stamp)

    def start_answer(self, work, tracking=False, stamp=None, snapshot=None):
        """Run the answer in a background thread so nothing else waits for it."""
        self.busy.acquire()
        self.status_pub.publish(String(data='tracking' if tracking else 'thinking'))
        number = None
        if not tracking:                        # tracking is silent: no "Still thinking"
            self.question_no += 1
            number = self.question_no
            self.asking = [number, time.monotonic(), 0]

        def run():
            try:
                outcome = work()
                self.deliver(outcome, tracking, stamp, number, snapshot)
            except Exception as problem:        # never let one question kill the node
                self.get_logger().error(f'Answering failed: {problem}')
                time.sleep(1.0)                 # do not spin if tracking keeps failing
            finally:
                if number is not None and self.asking and self.asking[0] == number:
                    self.asking = None          # answered: no more "Still thinking"
                if not self.track_question:
                    self.status_pub.publish(String(data='ready'))
                self.busy.release()
                self.ask_track()                # tracking: straight into the next round

        threading.Thread(target=run, daemon=True).start()

    def pin_found(self, outcome, snapshot):
        """Place a thing the user asked for, and that was found, on the map (silent)."""
        result = outcome.result
        if outcome.kind != 'find' or result is None or not result.found or not result.label \
                or outcome.picture is None or snapshot is None:
            return
        size = (int(outcome.picture.shape[1]), int(outcome.picture.shape[0]))
        located = self.place_on_map(result.box, size, snapshot.get('pose'))
        if located.found:
            self.pin(result.label, located)
        else:
            self.get_logger().info('"%s" not pinned: %s' % (result.label, located.problem))

    def deliver(self, outcome, tracking=False, stamp=None, number=None, snapshot=None):
        """Publish the result. A normal question also gets the picture, a log line, speech."""
        self.result_pub.publish(String(data=result_json(outcome, tracking, stamp)))
        if tracking:
            return                              # tracking: only the box, silent, no log spam
        if outcome.kind == 'command' and outcome.command:    # the map manager or labeller answers
            self.command_pub.publish(String(data=json.dumps(dict(outcome.command, speak=True))))
            self.get_logger().info(f'"{outcome.question}": map command {outcome.command}')
            self.asking = None
            self.request_pub.publish(String(data=json.dumps(request_for(outcome))))
            return
        self.pin_found(outcome, snapshot)
        if outcome.say == NOT_FOUND and outcome.kind == 'find':     # not in view: on the map?
            known = on_the_map(outcome.question, self.map_pins, self.pose_now())
            if known:
                outcome = replace(outcome, say=known)
        request = request_for(outcome)          # Stage 4's guide decides what to do next
        if request is not None:
            self.request_pub.publish(String(data=json.dumps(request)))
        if outcome.picture is not None:
            self.image_pub.publish(bgr_to_image(outcome.picture))
        found = outcome.result is not None and outcome.result.found
        self.get_logger().info(
            f'"{outcome.question}": ' + (f'found {outcome.result.label}' if found and
                                         not outcome.say else f'said "{outcome.say}"') +
            (f' ({outcome.problem})' if outcome.problem else '') +
            (f' | Qwen replied: {outcome.result.answer[:200]!r}' if outcome.result else ''))
        late = self.asking is None or self.asking[0] != number or self.asking[2] > len(
            STILL_THINKING_S)                   # "took too long" was already said
        if outcome.say and late:
            self.get_logger().info(f'Answer came after "{TOO_LONG}": not saying it')
        elif outcome.say:
            self.asking = None                  # answered: no more "Still thinking"
            self.say(outcome.say)


def main(args=None):
    """Start the node and keep it running until Ctrl+C."""
    rclpy.init(args=args)
    node = Stage2Node()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
