"""
The console's link to the device. Used by helper.py.

RosLink    one ROS node that stays alive while the device starts, stops and restarts. It listens
           to the topics the operator needs to see and sends the few commands the console may
           send. It never sends a speed other than zero.
FakeLink   the same shape with no ROS: a pretend device, for trying the screens on a PC.
diagnose() what looks wrong in a status, and which fix button to offer. No ROS here.

What the console may send (Adel, 17 Nov: features stay on voice; the console does what voice can't,
and the fixes):
  /stage3/command        new_map, save_map, load_map, set_entrance, rename_pin, remove_pin
  /stage4/vibration_test one signal or one grip motor for a few seconds (the wheels don't move)
  /stage4/goal "cancel"  and one zero speed on /cmd_vel_nav: the STOP button
  the two "clear the obstacle map" services of Nav2
"""

import base64
import collections
import json
import math
import re
import threading
import time

SIGNALS = ('near', 'stopped', 'left', 'right', 'motor1', 'motor2')
MAP_COMMANDS = ('new_map', 'save_map', 'load_map', 'set_entrance', 'rename_pin', 'remove_pin',
                'save_place', 'pin', 'set_kind')
PIN_KINDS = ('place', 'thing', 'sign', 'room')     # what a pin can be (map_names.add_pin)
PICTURE_IDLE_S = 10.0           # nobody asked for the map picture this long: stop listening
SAID_KEEP = 40                  # sentences kept for the screen
LOG_KEEP = 60                   # warnings and errors kept from /rosout
NOTES_KEEP = 8                  # the map manager's newest placing notes
STALE_S = 3.0                   # a topic silent this long counts as not coming
FIT = re.compile(r'(?<!next )(\d+(?:\.\d+)?)\s*%')     # "72%", but not the runner-up "next 52%"
SCORE = re.compile(r'\bscore=(\d*\.\d+)')               # Fit(... score=0.72 runner_up=0.40)


def note_fit(note):
    """The best fit in one map manager note, in %, or None (C18: the log text is the source)."""
    fits = [float(f) for f in FIT.findall(note)] + [100 * float(s) for s in SCORE.findall(note)]
    return max(fits) if fits else None


def heading_deg(qz, qw):
    return math.degrees(2 * math.atan2(qz, qw))


def parse_json(text, default=None):
    try:
        return json.loads(text)
    except (TypeError, ValueError):
        return default


class State:
    """What the console knows about the device. Filled by a link, read by the web server."""

    def __init__(self):
        self.lock = threading.Lock()
        self.values = {}            # key -> value
        self.times = {}             # key -> time it last came
        self.said = collections.deque(maxlen=SAID_KEEP)
        self.log = collections.deque(maxlen=LOG_KEEP)
        self.notes = collections.deque(maxlen=NOTES_KEEP)
        self.replies = collections.deque(maxlen=SAID_KEEP)
        self.count = 0              # messages taken so far (the helper's deaf watchdog)

    def put(self, key, value):
        with self.lock:
            self.count += 1
            self.values[key] = value
            self.times[key] = time.time()

    def add(self, where, item):
        with self.lock:
            getattr(self, where).append((round(time.time(), 1), item))

    def snapshot(self):
        now = time.time()
        with self.lock:
            out = dict(self.values)
            out['age'] = {k: round(now - t, 1) for k, t in self.times.items()}
            out['said'] = list(self.said)
            out['log'] = list(self.log)
            out['notes'] = list(self.notes)
            out['replies'] = list(self.replies)
        fit = None
        for _, note in reversed(out['notes']):
            fit = note_fit(note)
            if fit is not None:
                break
        out['fit'] = fit
        return out

    def replies_since(self, since):
        with self.lock:
            return [text for when, text in self.replies if when >= since]


def fresh(snap, key, limit=STALE_S):
    age = snap.get('age', {}).get(key)
    return age is not None and age <= limit


def diagnose(snap, running):
    """Give a list of {problem, fix, why} for what looks wrong now. fix = a button's name."""
    if not running:
        return []
    out = []
    health = snap.get('health') or {}
    if fresh(snap, 'grip_connected', 10) and snap.get('grip_connected') is False:
        out.append({'problem': 'The grip dropped out (look for its blue light)',
                    'fix': 'reset_master',
                    'why': 'Three drops on 17 Nov came with the wheels pulling hard. The fix '
                           'stops the device, restarts the master over USB, starts again.'})
    drive = snap.get('drive') or {}
    if fresh(snap, 'drive', 5) and drive.get('master') is False:
        out.append({'problem': 'The master ESP32 does not answer', 'fix': 'reset_master',
                    'why': 'The drive link gave up or the USB dropped.'})
    if not fresh(snap, 'health', 5):
        out.append({'problem': 'No health report: the device programs may have stopped',
                    'fix': 'restart', 'why': 'Nothing on /stage4/health for 5 s.'})
    grip_listed = any(p['fix'] == 'reset_master' for p in out)
    for part, ok in health.items():
        if part == 'handle' and grip_listed:
            continue                    # already offered as "the grip dropped out"
        if part == 'handle' and ok is False:
            out.append({'problem': 'Health says the handle is not connected',
                        'fix': 'reset_master', 'why': 'From the health program.'})
            continue
        if ok is False:
            out.append({'problem': 'Health says "%s" is not working' % part,
                        'fix': 'restart' if part != 'place' else 'replace',
                        'why': 'From the health program.'})
    placement = snap.get('placement')
    if placement == 'lost':
        out.append({'problem': 'The device has lost its place on the map', 'fix': 'replace',
                    'why': 'Walk it to the entrance and press Re-place, or restart there.'})
    guide = snap.get('guide') or {}
    if guide.get('route_ok') is False and guide.get('state') not in (None, 'idle'):
        out.append({'problem': 'The trip finds no route', 'fix': 'clear',
                    'why': 'Old camera marks can block the planner. Clear empties them.'})
    return out


class FakeLink:
    """A pretend device for trying the screens with no ROS (helper.py --fake)."""

    def __init__(self, state):
        self.state = state
        self.map_name = 'esb117_final'
        self.maps = ['esb117_final', 'esb117_device']
        self.pins = [{'name': 'ESB117', 'kind': 'sign', 'x': 4.3, 'y': -2.6},
                     {'name': 'bin', 'kind': 'thing', 'x': -3.0, 'y': 1.0},
                     {'name': 'entrance', 'kind': 'place', 'x': -8.2, 'y': -1.2}]
        self.placement = 'placed'
        self.running = False
        self.trouble = {}       # POST /api/fake/trouble: {"grip": false, "camera": false, ...}
        threading.Thread(target=self.tick, daemon=True).start()

    def device_up(self, up):
        self.running = up
        if up:
            self.trouble = {}           # a restart clears pretend problems
            self.say('Ready')

    def say(self, text):
        self.state.add('said', text)

    def reply(self, text):
        self.state.add('replies', text)
        self.state.add('said', text)

    def tick(self):
        step = 0
        while True:
            time.sleep(1.0)
            if not self.running:
                continue
            step += 1
            s, bad = self.state, self.trouble
            if 'placement' in bad:
                self.placement = bad.pop('placement')
            grip = bad.get('grip', True)
            s.put('health', {'warnings': True, 'camera': bad.get('camera', True),
                             'lidar': True, 'wheels': True, 'handle': grip,
                             'place': self.placement != 'lost'})
            s.put('placement', self.placement)
            s.put('map_name', self.map_name)
            s.put('position', {'x': round(math.sin(step / 9), 2), 'y': 0.2, 'heading': 12})
            s.put('pins', self.pins)
            s.put('grip_red', False)
            s.put('grip_green', False)
            s.put('grip_connected', grip)
            s.put('drive', {'connected': True, 'master': True, 'ready': True, 'stops': [],
                            'handle': grip, 'left_pct': 0, 'right_pct': 0, 'resets': 0})
            s.put('guide', {'state': bad.get('guide', 'idle'), 'route_ok': bad.get('route', True)})
            s.put('hazard', {'level': 'clear', 'distance': 2.4, 'side': ''})
            if self.placement == 'searching':
                fit = min(75, 55 + step % 25)
                s.add('notes', 'following 3 places (%d %%, next 52 %%)' % fit)
                if fit >= 72:
                    self.placement = 'placed'
                    self.say('Found where you are.')

    def map_command(self, command):
        cmd = command['cmd']
        if cmd == 'new_map':
            self.map_name, self.placement = '', ''
            self.pins = []
            self.reply('Started a new map.')
        elif cmd == 'save_map':
            self.map_name = command['name']
            if self.map_name not in self.maps:
                self.maps.append(self.map_name)
            self.reply('Map saved as %s' % self.map_name)
        elif cmd == 'load_map':
            self.map_name, self.placement = command['name'], 'searching'
            self.reply('Loaded %s' % self.map_name)
            self.say('Finding where you are. Keep walking.')
        elif cmd == 'set_entrance':
            self.reply('This is the entrance of the room now')
        elif cmd == 'remove_pin':
            hit = self.pins_called(command)
            self.pins = [p for p in self.pins if p not in hit]
            self.reply('Removed %s from the map' % command['name'] if hit
                       else 'There is no %s on the map' % command['name'])
        elif cmd == 'rename_pin':
            for pin in self.pins_called(command):
                pin['name'] = command['to']
            self.reply('%s is now called %s' % (command['name'], command['to']))
        elif cmd == 'save_place':
            pos = self.state.snapshot().get('position') or {'x': 0.0, 'y': 0.0}
            self.pins.append({'name': command['name'], 'kind': 'place', 'x': pos['x'],
                              'y': pos['y']})
            self.reply('Saved this place as %s' % command['name'])
        elif cmd == 'pin':
            self.pins.append({'name': command['name'], 'kind': command['kind'],
                              'x': command['x'], 'y': command['y']})

    def pins_called(self, command):
        """The pins a command means: all of that name, or with x, y the nearest within 0.5 m."""
        same = [p for p in self.pins if p['name'] == command['name']]
        if 'x' not in command:
            return same
        near = [(math.hypot(p['x'] - command['x'], p['y'] - command['y']), i)
                for i, p in enumerate(same)]
        near = [n for n in near if n[0] <= 0.5]
        return [same[min(near)[1]]] if near else []

    def signal(self, name, secs):
        self.state.add('log', 'fake: buzz %s for %.0f s' % (name, secs))

    def stop_moving(self):
        self.say('(fake) trip cancelled, zero speed sent')

    def go(self, text):
        self.say('(fake) a trip to %s' % text)

    def clear(self):
        return ['fake: cleared both obstacle maps']

    def saved_maps(self):
        return list(self.maps)

    def map_picture(self):
        """A plain drawing with the pins on it, in the shape the real one has."""
        x_left, y_top, span, size = -10.0, 5.0, 15.0, 480
        scale = size / span
        shown, dots = [], []
        for pin in self.pins:
            col, row = (pin['x'] - x_left) * scale, (y_top - pin['y']) * scale
            shown.append({'name': pin['name'], 'kind': pin.get('kind', 'thing'),
                          'col': round(col), 'row': round(row), 'x': pin['x'], 'y': pin['y']})
            dots.append('<circle cx="%.0f" cy="%.0f" r="6" fill="#c33"/>' % (col, row))
        svg = ('<svg xmlns="http://www.w3.org/2000/svg" width="480" height="480">'
               '<rect width="480" height="480" fill="#888"/><rect x="30" y="60" width="420" '
               'height="300" fill="#fff" stroke="#000" stroke-width="5"/>%s</svg>' % ''.join(dots))
        return {'image': base64.b64encode(svg.encode()).decode(), 'format': 'svg+xml',
                'view': {'x_left': x_left, 'y_top': y_top, 'span': span, 'size': size,
                         'zoom': 1.0, 'pins': shown}, 'age': 0.0}


class RosLink:
    """The real link: one rclpy node, spun in its own thread."""

    def __init__(self, state, maps_dir, spin=True, on_stop=None):
        import rclpy
        from rclpy.node import Node
        from rclpy.qos import DurabilityPolicy, QoSProfile
        from geometry_msgs.msg import PoseStamped, Twist
        from std_msgs.msg import Bool, String
        self.rclpy, self.state, self.maps_dir = rclpy, state, maps_dir
        self.on_stop = on_stop
        self.String, self.Twist = String, Twist
        rclpy.init()
        node = self.node = Node('apnvi_console')
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        put = state.put

        def sub(kind, topic, fn, qos=10):
            node.create_subscription(kind, topic, fn, qos)

        sub(String, '/stage4/health', lambda m: put('health', parse_json(m.data, {})))
        sub(String, '/stage3/placement', lambda m: put('placement', m.data), latched)
        sub(String, '/stage3/map_name', lambda m: put('map_name', m.data), latched)
        sub(String, '/stage3/pins', self.on_pins, latched)
        sub(PoseStamped, '/stage3/position', self.on_position)
        sub(String, '/stage3/reply', lambda m: (state.add('replies', m.data),
                                                state.add('said', m.data)))
        sub(String, '/voice/say', lambda m: state.add('said', m.data))
        sub(String, '/stage4/guide_state', lambda m: put('guide', parse_json(m.data, {})),
            latched)
        sub(String, '/stage4/drive_status', lambda m: put('drive', parse_json(m.data, {})))
        sub(String, '/stage4/stops', lambda m: put('stops', parse_json(m.data, [])), latched)
        sub(String, '/stage4/free_mode', lambda m: put('free', parse_json(m.data, {})))
        sub(String, '/stage2/status', lambda m: put('stage2', m.data))
        sub(String, '/stage2/labels', lambda m: put('labels', m.data))
        sub(Bool, '/grip/red', lambda m: put('grip_red', m.data), latched)
        sub(Bool, '/grip/green', lambda m: put('grip_green', m.data), latched)
        sub(Bool, '/grip/connected', lambda m: put('grip_connected', m.data), latched)
        sub(String, '/stage4/vibration_signal', lambda m: put('buzz', m.data))
        # The obstacle level only: /hazard/distance and /hazard/side come 10 times a second each
        # and cost the helper a third of its CPU for one line on the page (C2, 19 Nov: 9 % of a
        # core with all three).
        sub(String, '/hazard/level', lambda m: self.hazard('level', m.data))
        try:                            # the programs' own log lines: errors and placing notes
            from rcl_interfaces.msg import Log
            sub(Log, '/rosout', self.on_log, 100)
        except ImportError:
            pass
        self.map_pub = node.create_publisher(String, '/stage3/command', 10)
        self.signal_pub = node.create_publisher(String, '/stage4/vibration_test', 10)
        self.goal_pub = node.create_publisher(String, '/stage4/goal', 10)
        self.zero_pub = node.create_publisher(Twist, '/cmd_vel_nav', 10)
        self.hazard_now = {}
        self.picture, self.picture_view = None, None     # (JPEG bytes, time), the box it shows
        self.picture_asked = -1e9                        # monotonic time of the newest ask
        self.picture_subs = []
        if spin:                        # tests with the ROS stand-in deliver messages directly
            threading.Thread(target=self.spin, daemon=True).start()

    def spin(self):
        from rclpy.executors import SingleThreadedExecutor
        executor = SingleThreadedExecutor()
        executor.add_node(self.node)
        try:
            while self.rclpy.ok():
                self.picture_listening()
                executor.spin_once(timeout_sec=0.2)
        finally:
            # rclpy shuts itself down on SIGTERM (systemctl stop, pkill; spin_once then raises)
            # but the web server would go on answering, deaf (19 Nov). End the whole helper.
            if self.on_stop is not None:
                self.on_stop()

    # ---- listening --------------------------------------------------------------------------

    def hazard(self, key, value):
        self.hazard_now[key] = value
        self.state.put('hazard', dict(self.hazard_now))

    def on_pins(self, msg):
        data = parse_json(msg.data, {}) or {}
        self.state.put('pins', data.get('pins', []))
        self.state.put('pins_waiting', int(data.get('waiting', 0)))

    def on_position(self, msg):
        p, q = msg.pose.position, msg.pose.orientation
        self.state.put('position', {'x': round(p.x, 2), 'y': round(p.y, 2),
                                    'heading': round(heading_deg(q.z, q.w))})

    def on_log(self, msg):
        if 'map_manager' in msg.name and msg.level <= 20:
            self.state.add('notes', msg.msg)
        if msg.level >= 30:                 # WARN and ERROR
            self.state.add('log', '%s: %s' % (msg.name, msg.msg))

    # ---- sending ----------------------------------------------------------------------------

    def map_command(self, command):
        self.map_pub.publish(self.String(data=json.dumps(command)))

    def signal(self, name, secs):
        self.signal_pub.publish(self.String(data='%s %.1f' % (name, secs)))

    def stop_moving(self):
        """STOP: drop the trip, then one zero speed (as the guide's own cancel does)."""
        self.goal_pub.publish(self.String(data='cancel'))
        self.zero_pub.publish(self.Twist())

    def go(self, text):
        """Start a trip: a place name or "x y" on the map (the guide's /stage4/goal)."""
        self.goal_pub.publish(self.String(data=text))

    def clear(self):
        """Empty both planners' obstacle maps (session.py clear)."""
        from nav2_msgs.srv import ClearEntireCostmap
        done = []
        for name in ('/global_costmap/clear_entirely_global_costmap',
                     '/local_costmap/clear_entirely_local_costmap'):
            client = self.node.create_client(ClearEntireCostmap, name)
            if not client.wait_for_service(timeout_sec=2.0):
                done.append('not there: ' + name)
                continue
            future = client.call_async(ClearEntireCostmap.Request())
            end = time.time() + 3.0
            while not future.done() and time.time() < end:
                time.sleep(0.05)
            done.append(('cleared ' if future.done() else 'no answer: ') + name)
            self.node.destroy_client(client)
        return done

    def picture_listening(self):
        """
        Listen to the map picture only while the console shows it: map_view_node draws only
        while someone listens. Called from the spin thread, so the node is never changed from
        two threads at once.
        """
        wanted = time.monotonic() - self.picture_asked < PICTURE_IDLE_S
        if wanted and not self.picture_subs:
            from sensor_msgs.msg import CompressedImage
            self.picture_subs = [
                self.node.create_subscription(CompressedImage, '/stage3/map_image',
                                              self.on_picture, 1),
                self.node.create_subscription(self.String, '/stage3/map_view',
                                              self.on_picture_view, 1)]
        elif not wanted and self.picture_subs:
            for sub in self.picture_subs:
                self.node.destroy_subscription(sub)
            self.picture_subs, self.picture, self.picture_view = [], None, None

    def on_picture(self, msg):
        self.picture = (bytes(msg.data), time.time())

    def on_picture_view(self, msg):
        self.picture_view = parse_json(msg.data, None)

    def map_picture(self):
        """The newest map picture and the box it shows, or None (none yet)."""
        self.picture_asked = time.monotonic()
        picture, view = self.picture, self.picture_view
        if picture is None or not view:
            return None
        return {'image': base64.b64encode(picture[0]).decode(), 'format': 'jpeg', 'view': view,
                'age': round(time.time() - picture[1], 1)}

    def saved_maps(self):
        import os
        folder = os.path.join(self.maps_dir, 'named')
        if not os.path.isdir(folder):
            return []
        return sorted(f[:-3] for f in os.listdir(folder) if f.endswith('.db'))
