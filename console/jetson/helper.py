#!/usr/bin/env python3
"""
The console's helper on the Jetson. Always on, started at boot
(apnvi-console.service), even while the device itself is stopped.

It answers the PC's console (console/pc/hub.py) through the ssh tunnel, on 127.0.0.1:8082 only:
nobody else on the Wi-Fi can reach it. It starts and stops the device with the session scripts,
shows what the device is doing, and sends the few commands voice can't (links.py says which).
It never drives the wheels: its only speed is the STOP button's zero.

  python3 helper.py            the real device (needs ROS sourced: run_helper.sh does it)
  python3 helper.py --fake     a pretend device, to try the screens on any computer

The API (JSON in and out):
  GET  /api/status            running?, the job in progress, what the device is doing, problems
  GET  /api/maps              the saved maps
  GET  /api/log               the newest lines of /tmp/apnvi_session.log, warnings first
  POST /api/device/start      {"preset": "mapping"|"guided", "speed": 0.3, "map": "esb117_final"}
  POST /api/device/stop | restart | reset_master | hard_stop
  POST /api/stop              STOP: cancel the trip, one zero speed
  POST /api/map               {"cmd": "new_map"|"save_map"|"load_map"|"set_entrance"|
                               "rename_pin"|"remove_pin"|"save_place"|"pin"|"set_kind",
                               "name": ..., "to": ..., "x": ..., "y": ..., "kind": ...,
                               "force": false}
  POST /api/map/delete        {"name": ...}: a saved map moves to <maps>/deleted/<time>/
  POST /api/map/rename_map    {"name": ..., "to": ...}
  POST /api/goal              {"name": ...} or {"x": ..., "y": ...}: start a trip there
  GET  /api/map/picture       the map picture (base64) and the box it shows, for clicks
  POST /api/replace           find the place again from the entrance (loads the running map again)
  POST /api/signal            {"name": "near"|"stopped"|"left"|"right"|"motor1"|"motor2", "secs": 3}
  POST /api/clear             empty both obstacle maps
"""

import argparse
import json
import math
import os
import re
import subprocess
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from links import (FakeLink, MAP_COMMANDS, PIN_KINDS, RosLink, SIGNALS, State,  # noqa: E402
                   diagnose)

SESSION = os.path.normpath(os.path.join(HERE, '..', '..', 'device'))
LOG_FILE = '/tmp/apnvi_session.log'
CONF_FILE = os.path.expanduser('~/apnvi_device.conf')
PORT = 8082
PROTECTED_MAPS = ('esb117_final',)     # never written over from the console without "force"
# 23 Nov (Adel): the labeller is always on; there is no "labeller off" any more. The old preset
# names are still taken (a page or a test that sends "guided" gets the labeller too).
PRESETS = {
    'mapping': ['labels:=true'],       # pins appear while a new map is walked
    'guided': ['labels:=true'],
}
JOB_LINES = 80
REPLY_WAIT_S = {'save_map': 30.0, 'load_map': 30.0, 'new_map': 15.0}
DEAF_S = 90.0           # device running this long with not one message: the link is deaf
DEAF_EXIT = 3           # exit code; the service's Restart=on-failure starts a fresh helper


class DeafWatch:
    """19 Nov (T1): a helper started at boot, before the network settled, never heard the device
    (14 topics after a restart, none in 4 hours). Deaf = the device ran DEAF_S without a message.
    Uses a message count and the monotonic clock: the Jetson's clock jumps after boot."""

    def __init__(self, deaf_s=DEAF_S):
        self.deaf_s = deaf_s
        self.since = None           # (monotonic time, message count) when the wait began

    def deaf(self, running, count, now):
        if not running or self.since is None or self.since[1] != count:
            self.since = (now, count) if running else None
            return False
        return now - self.since[0] >= self.deaf_s


def watch_deaf(state, device, period_s=5.0):
    watch = DeafWatch()
    while True:
        time.sleep(period_s)
        job = device.job_view()
        if job is not None and job['state'] == 'running':
            watch.since = None      # a start, stop or reset is under way
            continue
        if watch.deaf(device.running(), state.count, time.monotonic()):
            print('deaf: the device runs but no message came for %.0f s; exiting so the '
                  'service starts a fresh helper' % watch.deaf_s, flush=True)
            os._exit(DEAF_EXIT)


def slug(name):
    """The map manager's file name for a map (map_names.slug)."""
    return re.sub(r'[^a-z0-9]+', '_', str(name).lower()).strip('_')


def start_options(preset, speed, map_name=''):
    """Give the words after `bash start.sh`."""
    try:
        speed = float(speed)
    except (TypeError, ValueError):
        speed = 0.2                     # 23 Nov: 0.3 -> 0.2
    speed = min(max(speed, 0.1), 0.5)
    options = ['%.1f' % speed] + list(PRESETS.get(preset, PRESETS['guided']))
    if map_name:
        options.append('map:=%s' % slug(map_name))
    return options


def again_options(body, last):
    """
    Give the words for Restart and Reset master: the Get ready choices sent with the press.

    22 Nov (Adel): Restart reused the last start's words, so a device started as Guided stayed
    labels:=false after "Mapping demo" was picked and Restart pressed (and Get ready itself
    said to press Restart). A map:= of the last start is kept. Nothing sent: the last words.
    """
    if not body.get('preset'):
        return list(last)
    kept = [word for word in last if word.startswith('map:=')]
    return start_options(body.get('preset'), body.get('speed', 0.3)) + kept


def check_map_command(body):
    """Give (command dict, None) or (None, why not)."""
    cmd = body.get('cmd')
    if cmd not in MAP_COMMANDS:
        return None, 'unknown map command'
    name = str(body.get('name', '')).strip()
    command = {'cmd': cmd, 'name': name}
    if cmd in ('save_map', 'load_map', 'rename_pin', 'remove_pin', 'save_place', 'pin',
               'set_kind') and not name:
        return None, 'a name is needed'
    if cmd == 'save_map' and slug(name) in PROTECTED_MAPS and not body.get('force'):
        return None, ('%s is the tested map. Save under another name, or tick "write over it"'
                      % name)
    if cmd in ('rename_pin', 'remove_pin'):
        # 23 Nov: one Remove took all 60 tables off outside_2. The console always names the one
        # pin it means by its spot; without one, nothing is changed (by voice is Stage 2's way).
        if 'x' not in body or 'y' not in body:
            return None, 'say which pin: click it on the map or in the list'
        try:
            x, y = float(body.get('x')), float(body.get('y'))
        except (TypeError, ValueError):
            return None, 'x and y must be numbers (metres on the map)'
        if not (math.isfinite(x) and math.isfinite(y)):
            return None, 'x and y must be on the map (metres)'
        command.update({'x': x, 'y': y})
    if cmd == 'rename_pin':
        to = str(body.get('to', '')).strip()
        if not to:
            return None, 'the new name is missing'
        command['to'] = to
    if cmd == 'set_entrance':
        command['name'] = 'entrance'
    if cmd in ('pin', 'set_kind'):     # a pin put by hand: sure at once, no looks needed
        try:
            x, y = float(body.get('x')), float(body.get('y'))
        except (TypeError, ValueError):
            return None, 'x and y must be numbers (metres on the map)'
        if not (math.isfinite(x) and math.isfinite(y)) or max(abs(x), abs(y)) > 200:
            return None, 'x and y must be on the map (metres)'
        kind = body.get('kind', 'thing')
        if kind not in PIN_KINDS:
            return None, 'a pin is a %s' % ', '.join(PIN_KINDS)
        command.update({'x': x, 'y': y, 'kind': kind, 'sure': True})
    return command, None


def goal_text(body):
    """The guide's goal for a console trip: a pin's name, or "x y" on the map. (text, why not)."""
    name = str(body.get('name', '')).strip()
    if name:
        return name, None
    try:
        x, y = float(body.get('x')), float(body.get('y'))
    except (TypeError, ValueError):
        return None, 'a trip needs a pin name, or x and y on the map'
    if not (math.isfinite(x) and math.isfinite(y)) or max(abs(x), abs(y)) > 200:
        return None, 'x and y must be on the map (metres)'
    return '%.2f %.2f' % (x, y), None


def rename_saved_map(maps_dir, old, new):
    """Give a saved map a new name (its files and the name inside). None, or why not."""
    a, b = slug(old), slug(new)
    if not a or not b:
        return 'a name is needed'
    named, work = os.path.join(maps_dir, 'named'), os.path.join(maps_dir, 'work')
    if not os.path.exists(os.path.join(named, a + '.db')):
        return 'there is no saved map called %s' % old
    if a == b:
        return 'that is already its name'
    if os.path.exists(os.path.join(named, b + '.db')):
        return 'a map called %s already exists' % b
    moves = [(os.path.join(named, a + end), os.path.join(named, b + end))
             for end in ('.db', '.json', '.session.json')]
    moves.append((os.path.join(work, a + '_work.db'), os.path.join(work, b + '_work.db')))
    for source, target in moves:
        if os.path.exists(source):
            os.replace(source, target)
    table = os.path.join(named, b + '.json')
    try:
        with open(table) as handle:
            data = json.load(handle)
        data['map'] = str(new).strip()
        with open(table + '.partial', 'w') as handle:
            json.dump(data, handle, indent=1)
        os.replace(table + '.partial', table)
    except (OSError, ValueError, TypeError):
        pass
    return None


def delete_saved_map(maps_dir, name, when=None):
    """
    Take a saved map off the list: its files move to <maps>/deleted/<time>/, where they can be
    put back by hand. Give the files moved (none: there was no such map).
    """
    key = slug(name)
    if not key:
        return []
    found = []
    for folder, pattern in (('named', re.compile(re.escape(key) + r'\.')),
                            ('work', re.compile(re.escape(key) + r'_work\.'))):
        where = os.path.join(maps_dir, folder)
        if os.path.isdir(where):
            found += [os.path.join(where, f) for f in sorted(os.listdir(where))
                      if pattern.match(f)]
    if not any(f.endswith('.db') and os.sep + 'named' + os.sep in f for f in found):
        return []
    bin_dir = os.path.join(maps_dir, 'deleted', time.strftime('%Y%m%d_%H%M%S',
                                                              time.localtime(when)))
    os.makedirs(bin_dir, exist_ok=True)
    for path in found:
        os.replace(path, os.path.join(bin_dir, os.path.basename(path)))
    return found


class Device:
    """Starts and stops the device, one job at a time, and keeps each job's output."""

    def __init__(self, fake_link=None):
        self.fake = fake_link
        self.lock = threading.Lock()
        self.job = None                 # {'name', 'state': running/done/failed, 'lines', ...}
        self.last_options = start_options('guided', 0.3)

    def running(self):
        if self.fake is not None:
            return self.fake.running
        found = subprocess.run(['pgrep', '-f', 'ros2 launch apnvi_stage4'],
                               stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        return found.returncode == 0

    def job_view(self):
        with self.lock:
            if self.job is None:
                return None
            view = dict(self.job)
            view['lines'] = list(view['lines'])[-JOB_LINES:]
            return view

    def begin(self, name, steps):
        """Run steps (a list of (label, function)) in a thread. False if a job is running."""
        with self.lock:
            if self.job is not None and self.job['state'] == 'running':
                return False
            self.job = {'name': name, 'state': 'running', 'lines': [], 'step': '',
                        'started': time.time(), 'ended': None}
        threading.Thread(target=self.run, args=(steps,), daemon=True).start()
        return True

    def note(self, line):
        with self.lock:
            self.job['lines'].append(line.rstrip())

    def run(self, steps):
        ok = True
        for label, step in steps:
            with self.lock:
                self.job['step'] = label
            self.note('--- ' + label)
            try:
                if step() is False:
                    ok = False
                    break
            except Exception as error:     # noqa: BLE001 (shown to the operator)
                self.note('failed: %s' % error)
                ok = False
                break
        with self.lock:
            self.job['state'] = 'done' if ok else 'failed'
            self.job['ended'] = time.time()

    def shell(self, words, timeout=180):
        """Run a session script, copying its output into the job. True if it ended well."""
        if self.fake is not None:
            self.note('(fake) ' + ' '.join(words))
            time.sleep(2.0)
            if words[1].endswith('start.sh'):
                self.fake.device_up(True)
            elif words[1].endswith('stop.sh'):
                self.fake.device_up(False)
            return True
        process = subprocess.Popen(words, cwd=SESSION, stdout=subprocess.PIPE,
                                   stderr=subprocess.STDOUT, text=True, bufsize=1)
        end = time.monotonic() + timeout    # not time.time(): the clock jumps when the Jetson syncs
        for line in process.stdout:
            self.note(line)
            if time.monotonic() > end:
                process.kill()
                self.note('stopped waiting after %d s' % timeout)
                return False
        return process.wait() == 0

    # ---- the jobs ---------------------------------------------------------------------------

    def start_steps(self, options):
        return [('start the device (about 50 s)',
                 lambda: self.shell(['bash', os.path.join(SESSION, 'start.sh')] + options))]

    def stop_steps(self):
        return [('stop the device',
                 lambda: self.shell(['bash', os.path.join(SESSION, 'stop.sh')], timeout=40))]

    def reset_steps(self, options):
        reset = ('restart the master ESP32 over USB',
                 lambda: self.shell(['python3', os.path.join(SESSION, 'session.py'), 'reset'],
                                    timeout=30))
        return self.stop_steps() + [reset] + self.start_steps(options)

    def hard_stop(self):
        """Stop the drive link: the master hears nothing and stops the wheels within 0.5 s."""
        if self.fake is not None:
            return 'fake: drive link stopped'
        subprocess.run(['pkill', '-f', 'drive_link_node'])
        return 'drive link stopped; the master stops the wheels by itself. Restart to drive again.'


# Known, harmless and frequent (19 Nov on the Jetson: the deskew line came 10 times a second and
# filled "Show last errors"; the Stage 1 board is not plugged in on purpose).
LOG_NOISE = re.compile(r'deskew_impl|computeTransform\(\) Missing visual features|'
                       r'Icp/CorrespondenceRatio|stage1-esp32-not-plugged')
LOG_READ_BYTES = 1500000            # the session log grows to many MB: read only its end


def tail_log(path=LOG_FILE, lines=60):
    try:
        with open(path, 'rb') as f:
            start = max(0, os.path.getsize(path) - LOG_READ_BYTES)
            f.seek(start)
            text = f.read().decode('utf-8', 'replace').splitlines()
            text = text[1 if start else 0:][-4000:]     # the first line may be cut
    except OSError:
        return {'warnings': [], 'tail': ['no log yet: %s' % path]}
    text = [line for line in text if not LOG_NOISE.search(line)]
    bad = [line for line in text if re.search(r'\b(ERROR|WARN|WARNING)\b', line)
           or re.search(r'\b(Traceback|failed|died)\b', line, re.I)]
    return {'warnings': bad[-30:], 'tail': text[-lines:]}


def read_settings(path=CONF_FILE):
    try:
        with open(path) as f:
            return [line.split('#')[0].strip() for line in f if line.split('#')[0].strip()]
    except OSError:
        return []


def make_handler(state, link, device):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *args):       # quiet
            pass

        def send_json(self, data, code=200):
            body = json.dumps(data).encode('utf-8')
            self.send_response(code)
            self.send_header('Content-Type', 'application/json')
            self.send_header('Content-Length', str(len(body)))
            self.send_header('Cache-Control', 'no-store')
            self.end_headers()
            self.wfile.write(body)

        def body(self):
            size = min(int(self.headers.get('Content-Length') or 0), 10000)
            data = json.loads(self.rfile.read(size) or b'{}') if size else {}
            return data if isinstance(data, dict) else {}

        def do_GET(self):  # noqa: N802
            path = self.path.split('?')[0]
            if path == '/api/status':
                running = device.running()
                snap = state.snapshot()
                self.send_json({'running': running, 'job': device.job_view(),
                                'device': snap, 'problems': diagnose(snap, running),
                                'settings': read_settings(), 'options': device.last_options,
                                'time': time.time()})
            elif path == '/api/map/picture':
                picture = link.map_picture()
                self.send_json(picture or {'image': None, 'why': (
                    'The map shows here while the device runs.' if not device.running()
                    else 'Waiting for the map picture ...')})
            elif path == '/api/maps':
                self.send_json({'maps': link.saved_maps(), 'protected': list(PROTECTED_MAPS)})
            elif path == '/api/log':
                self.send_json(tail_log())
            else:
                self.send_json({'error': 'unknown address'}, 404)

        def do_POST(self):  # noqa: N802
            path = self.path.split('?')[0]
            try:
                body = self.body()
            except ValueError:
                self.send_json({'error': 'not JSON'}, 400)
                return
            if path.startswith('/api/device/'):
                self.device_job(path.rsplit('/', 1)[1], body)
            elif path == '/api/stop':
                link.stop_moving()
                self.send_json({'ok': True, 'said': 'trip cancelled, zero speed sent'})
            elif path == '/api/map':
                self.map_command(body)
            elif path == '/api/goal':
                self.goal(body)
            elif path == '/api/map/delete':
                self.delete_map(str(body.get('name', '')).strip())
            elif path == '/api/map/rename_map':
                self.rename_map(str(body.get('name', '')).strip(),
                                str(body.get('to', '')).strip())
            elif path == '/api/replace':
                name = (state.snapshot().get('map_name') or '').strip()
                if not name:
                    self.send_json({'error': 'no saved map is loaded (a new map is being '
                                             'drawn): load one first'}, 409)
                    return
                self.map_command({'cmd': 'load_map', 'name': name})
            elif path == '/api/signal':
                name = body.get('name')
                if name not in SIGNALS:
                    self.send_json({'error': 'unknown signal'}, 400)
                    return
                secs = min(max(float(body.get('secs', 3)), 1.0), 10.0)
                link.signal(name, secs)
                self.send_json({'ok': True})
            elif path == '/api/clear':
                self.send_json({'ok': True, 'done': link.clear()})
            elif path == '/api/fake/trouble' and isinstance(link, FakeLink):
                link.trouble = body             # e.g. {"grip": false, "placement": "lost"}
                self.send_json({'ok': True})
            else:
                self.send_json({'error': 'unknown address'}, 404)

        def device_job(self, what, body):
            if what == 'hard_stop':
                self.send_json({'ok': True, 'said': device.hard_stop()})
                return
            options = device.last_options
            if what == 'start':
                options = start_options(body.get('preset'), body.get('speed', 0.3),
                                        body.get('map', ''))
                steps = device.start_steps(options)
            elif what == 'stop':
                steps = device.stop_steps()
            elif what == 'restart':
                options = again_options(body, device.last_options)
                steps = device.stop_steps() + device.start_steps(options)
            elif what == 'reset_master':
                options = again_options(body, device.last_options)
                steps = device.reset_steps(options)
            else:
                self.send_json({'error': 'unknown device job'}, 404)
                return
            if not device.begin(what, steps):
                self.send_json({'error': 'another job is still running'}, 409)
                return
            device.last_options = options
            self.send_json({'ok': True})

        def goal(self, body):
            text, why = goal_text(body)
            if text is None:
                self.send_json({'error': why}, 400)
                return
            if not device.running():
                self.send_json({'error': 'the device is not running'}, 409)
                return
            link.go(text)
            self.send_json({'ok': True, 'reply': 'Trip asked: %s' % text})

        def rename_map(self, name, to):
            if slug(name) in PROTECTED_MAPS or slug(to) in PROTECTED_MAPS:
                self.send_json({'error': '%s is the tested map: its name stays'
                                         % PROTECTED_MAPS[0]}, 409)
                return
            if device.running() and slug(state.snapshot().get('map_name') or '') == slug(name):
                self.send_json({'error': '%s is the map in use: load another map first'
                                         % name}, 409)
                return
            if isinstance(link, FakeLink):
                found = [m for m in link.maps if slug(m) == slug(name)]
                why = None if found else 'there is no saved map called %s' % name
                if found:
                    link.maps = [slug(to) if m == found[0] else m for m in link.maps]
            else:
                why = rename_saved_map(link.maps_dir, name, to)
            if why:
                self.send_json({'error': why}, 400)
                return
            self.send_json({'ok': True, 'reply': '%s is now called %s' % (name, slug(to))})

        def delete_map(self, name):
            if slug(name) in PROTECTED_MAPS:
                self.send_json({'error': '%s is the tested map: it cannot be deleted from the '
                                         'console' % name}, 409)
                return
            if device.running() and slug(state.snapshot().get('map_name') or '') == slug(name):
                self.send_json({'error': '%s is the map in use: load another map first'
                                         % name}, 409)
                return
            if isinstance(link, FakeLink):
                gone = [m for m in link.maps if slug(m) == slug(name)]
                link.maps = [m for m in link.maps if slug(m) != slug(name)]
            else:
                gone = delete_saved_map(link.maps_dir, name)
            if not gone:
                self.send_json({'error': 'there is no saved map called %s' % name}, 404)
                return
            self.send_json({'ok': True, 'reply': 'Deleted %s (its files are in the deleted '
                                                 'folder of the maps)' % name})

        def map_command(self, body):
            command, why = check_map_command(body)
            if command is None:
                self.send_json({'error': why}, 400)
                return
            if not device.running():
                self.send_json({'error': 'the device is not running'}, 409)
                return
            trip = (state.snapshot().get('guide') or {}).get('state')
            if command['cmd'] in ('new_map', 'load_map') and trip == 'guiding':
                self.send_json({'error': 'a trip is on: press STOP first, then change the map'},
                               409)
                return
            since = time.time()
            if command['cmd'] == 'set_kind':    # take it off, then pin it again as the new kind
                link.map_command({'cmd': 'remove_pin', 'name': command['name'],
                                  'x': command['x'], 'y': command['y']})
                end = since + 5.0
                while time.time() < end and not state.replies_since(since - 0.05):
                    time.sleep(0.1)
                command = dict(command, cmd='pin')
            link.map_command(command)
            if command['cmd'] == 'pin':         # the map manager says nothing: watch the pins
                end = since + 5.0
                while time.time() < end:
                    pins = state.snapshot().get('pins') or []
                    if any(p.get('name') == command['name'] and p.get('kind') in (
                            command['kind'], 'room') for p in pins):
                        self.send_json({'ok': True, 'reply': 'Pinned %s (%s) at %.1f, %.1f' % (
                            command['name'], command['kind'], command['x'], command['y'])})
                        return
                    time.sleep(0.1)
                self.send_json({'ok': True, 'reply': '(sent; the pin has not shown yet)'})
                return
            end = since + REPLY_WAIT_S.get(command['cmd'], 5.0)
            while time.time() < end:
                replies = state.replies_since(since - 0.05)
                if replies:
                    self.send_json({'ok': True, 'reply': replies[-1]})
                    return
                time.sleep(0.1)
            self.send_json({'ok': True, 'reply': '(sent; no answer yet)'})

    return Handler


def main():
    parser = argparse.ArgumentParser(description=__doc__.split('\n')[1])
    parser.add_argument('--fake', action='store_true', help='a pretend device, no ROS')
    parser.add_argument('--port', type=int, default=PORT)
    parser.add_argument('--maps-dir', default=os.path.expanduser('~/.ros/apnvi_maps'))
    args = parser.parse_args()
    state = State()
    if args.fake:
        link = FakeLink(state)
        device = Device(fake_link=link)
    else:
        link = RosLink(state, args.maps_dir, on_stop=lambda: os._exit(0))
        device = Device()
        threading.Thread(target=watch_deaf, args=(state, device), daemon=True).start()
    server = ThreadingHTTPServer(('127.0.0.1', args.port), make_handler(state, link, device))
    print('console helper on 127.0.0.1:%d%s' % (args.port, ' (fake device)' if args.fake else ''))
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
