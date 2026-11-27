#!/usr/bin/env python3
"""
The APNVI console on the campus PC. Opened by start_console.bat.

It shows the console page on http://localhost:8100 and does what only the PC can do:
  - starts and checks the PC's three servers (Ollama 11434, speech 8090, voice 8091) from the
    same Startup shortcuts the lab uses, and warms up Qwen,
  - keeps the ssh tunnel to the Jetson open (8081: the device page; 8082: the console helper),
  - runs "Get ready": every step from a cold lab to "Ready", each with a status,
  - passes the page's device buttons to the Jetson's helper (/api/dev/... -> helper /api/...).

  python hub.py           the real lab
  python hub.py --fake    no ssh, no starting: talks to a helper started with
                          `python ../jetson/helper.py --fake` on this computer

Settings: hub_config.json beside this file (optional; see DEFAULTS below).
Python 3 only, nothing to install.
"""

import argparse
import json
import os
import re
import socket
import subprocess
import sys
import threading
import time
import urllib.error
import urllib.request
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

HERE = os.path.dirname(os.path.abspath(__file__))
STARTUP = os.path.join(os.environ.get('APPDATA', ''),
                       r'Microsoft\Windows\Start Menu\Programs\Startup')
DEFAULTS = {
    'port': 8100,
    'ssh_host': 'jetson',              # the name in ~/.ssh/config
    'jetson_address': '',              # empty: read HostName from ~/.ssh/config
    # Remote link: off the hotspot (AUS wireless), reach the Jetson through its own
    # Cloudflare tunnel. Its address comes from the front door (ai_server/jetson_link.py).
    'remote_ssh_host': 'jetson-remote',
    'jetson_file': os.path.normpath(os.path.join(
        HERE, '..', '..', 'ai_server', 'front_door_jetson.json')),
    'helper_port': 8082,
    'page_port': 8081,
    'servers': [
        {'name': 'AI (Ollama)', 'key': 'ollama', 'port': 11434,
         'start': os.path.join(STARTUP, 'Ollama.lnk')},
        {'name': 'Speech to text', 'key': 'speech', 'port': 8090,
         'start': os.path.join(STARTUP, 'APNVI speech.lnk')},
        {'name': 'Voice (Kokoro)', 'key': 'voice', 'port': 8091,
         'start': os.path.join(STARTUP, 'APNVI voice.lnk')},
    ],
    'model': '',                       # empty: model:= in the Jetson's settings, else the default
    'server_wait_s': 90,
    'jetson_wait_s': 240,
}
HEALTH_PARTS = ('warnings', 'camera', 'lidar', 'wheels', 'handle')
DEVICE_DEFAULT_MODEL = 'qwen3-vl:4b-instruct'     # stage2.launch.py's default for model:=


def load_config():
    config = json.loads(json.dumps(DEFAULTS))
    path = os.path.join(HERE, 'hub_config.json')
    if os.path.exists(path):
        with open(path) as f:
            config.update(json.load(f))
    if not config['jetson_address']:
        config['jetson_address'] = ssh_hostname(config['ssh_host']) or config['ssh_host']
    return config


def ssh_hostname(alias):
    """HostName of `alias` in ~/.ssh/config, or ''."""
    try:
        with open(os.path.expanduser('~/.ssh/config')) as f:
            text = f.read()
    except OSError:
        return ''
    block = re.search(r'^Host\s+%s\s*$(.*?)(?=^Host\s|\Z)' % re.escape(alias), text, re.M | re.S)
    if not block:
        return ''
    found = re.search(r'^\s*HostName\s+(\S+)', block.group(1), re.M)
    return found.group(1) if found else ''


def port_open(host, port, timeout=1.0):
    try:
        with socket.create_connection((host, port), timeout=timeout):
            return True
    except OSError:
        return False


def listening_pids(port, netstat_text=None):
    """Windows: the process ids listening on a local TCP port (from netstat -ano)."""
    if netstat_text is None:
        try:
            netstat_text = subprocess.run(['netstat', '-ano', '-p', 'TCP'], capture_output=True,
                                          text=True, timeout=10).stdout
        except (OSError, subprocess.SubprocessError):
            return []
    pids = set()
    for line in netstat_text.splitlines():
        parts = line.split()
        if (len(parts) == 5 and parts[0] == 'TCP' and parts[3] == 'LISTENING'
                and parts[1].rsplit(':', 1)[-1] == str(port) and parts[4].isdigit()):
            pids.add(int(parts[4]))
    pids.discard(0)
    pids.discard(os.getpid())
    return sorted(pids)


REMOTE_FRESH_S = 120.0      # the Jetson reports its tunnel every 30 s


def remote_fresh(config, clock=time.time):
    """True if the Jetson reported its own ssh tunnel lately (remote link)."""
    if not config.get('remote_ssh_host'):
        return False
    try:
        with open(config['jetson_file'], encoding='utf-8') as f:
            kept = json.load(f)
        return bool(kept.get('ssh')) and clock() - float(kept.get('at', 0)) <= REMOTE_FRESH_S
    except (OSError, ValueError, KeyError, TypeError):
        return False


def jetson_route(config, timeout=3.0):
    """How to reach the Jetson now: (ssh name, ssh connect timeout), or (None, None).

    On the hotspot: straight to it. Off it (AUS wireless): through its own tunnel, if it has
    reported one lately. The direct way is always tried first.

    23 Nov: `Host jetson` is the Jetson's name (apnvi-desktop.local), so the wired campus PC
    reaches it directly on AUS wireless too, at whatever address it has; the tunnel died after
    the Jetson moved access point and the console had used only it. 3 s: the name lookup alone
    took about 1 s.
    """
    name = config['jetson_address']
    address = ipv4(name) or name
    if port_open(address, 22, timeout):
        return config['ssh_host'], 5
    fresh = ipv4(name, fresh=True)          # it may have a new address (another access point)
    if fresh and fresh != address and port_open(fresh, 22, timeout):
        return config['ssh_host'], 5
    if remote_fresh(config):
        return config['remote_ssh_host'], 30
    return None, None


LAST_ADDRESS = {}           # name -> the IPv4 address it last had (the lookup can take 4 s)


def ipv4(name, fresh=False):
    """The IPv4 address of a name (Python finds .local names; Windows' ssh does not), or ''.

    The last address found is kept and given at once; fresh=True looks the name up again.
    """
    if not fresh and LAST_ADDRESS.get(name):
        return LAST_ADDRESS[name]
    try:
        LAST_ADDRESS[name] = socket.getaddrinfo(name, 22, socket.AF_INET)[0][4][0]
    except (OSError, IndexError):
        pass
    return LAST_ADDRESS.get(name, '')


def host_words(config, host):
    """The last words of an ssh command for `host`: the direct route gets the address found by
    name now (23 Nov: Windows' ssh.exe could not resolve apnvi-desktop.local)."""
    if host == config['ssh_host']:
        address = ipv4(config['jetson_address'])
        if address:
            return ['-o', 'HostName=%s' % address, host]
    return [host]


def http_json(url, data=None, timeout=5.0):
    """GET (data None) or POST JSON; give (status code, decoded JSON or {'error': ...})."""
    body = None if data is None else json.dumps(data).encode('utf-8')
    request = urllib.request.Request(url, data=body, method='GET' if data is None else 'POST',
                                     headers={'Content-Type': 'application/json'})
    try:
        with urllib.request.urlopen(request, timeout=timeout) as reply:
            return reply.status, json.loads(reply.read() or b'{}')
    except urllib.error.HTTPError as error:
        try:
            return error.code, json.loads(error.read() or b'{}')
        except ValueError:
            return error.code, {'error': str(error)}
    except (OSError, ValueError) as error:
        return 0, {'error': str(error)}


def helper_reply(code, data):
    """What to send back for a call passed on to the helper; a dead link said in plain words."""
    if code:
        return data, code
    return {'error': 'The console helper on the Jetson does not answer (%s). Press Fix on '
            '"Link to the Jetson".' % data.get('error', '')}, 502


class Tunnel:
    """Keeps `ssh -N -L ...` to the Jetson running while the Jetson answers."""

    def __init__(self, config, fake):
        self.config, self.fake = config, fake
        self.process = None
        self.settled_at = 0.0       # monotonic time by which the tunnel should have its ports
        self.note = 'not started'
        self.lock = threading.Lock()
        if not fake:
            threading.Thread(target=self.keep, daemon=True).start()

    def forwards(self):
        words = []
        for port in (self.config['page_port'], self.config['helper_port']):
            if port_open('127.0.0.1', port, 0.3):
                continue                # already forwarded (another ssh window, or ours)
            words += ['-L', '%d:localhost:%d' % (port, port)]
        return words

    def keep(self):
        while True:
            with self.lock:
                alive = self.process is not None and self.process.poll() is None
            # 19 Nov: over the remote link ssh needs 5 to 10 s to open its ports; checking sooner
            # killed every tunnel before it was up.
            if alive and time.monotonic() > self.settled_at and self.forwards():
                # A port we need is not forwarded (another ssh window held it and closed):
                # start our tunnel again with both ports.
                with self.lock:
                    self.process.terminate()
                alive = False
                time.sleep(1.0)
            host, wait_s = jetson_route(self.config) if not alive else (None, None)
            if host:
                forwards = self.forwards()
                if forwards:
                    self.open(forwards, host, wait_s)
                else:
                    self.note = 'ports already open on this PC'
            time.sleep(3.0)

    def open(self, forwards, host=None, wait_s=5):
        host = host or self.config['ssh_host']
        words = ['ssh', '-N', '-o', 'ExitOnForwardFailure=yes', '-o', 'ServerAliveInterval=10',
                 '-o', 'ServerAliveCountMax=3', '-o', 'ConnectTimeout=%d' % wait_s,
                 '-o', 'BatchMode=yes'] + forwards + host_words(self.config, host)
        flags = getattr(subprocess, 'CREATE_NO_WINDOW', 0)
        with self.lock:
            self.settled_at = time.monotonic() + wait_s + 15
            self.process = subprocess.Popen(words, stdin=subprocess.DEVNULL,
                                            stdout=subprocess.DEVNULL,
                                            stderr=subprocess.DEVNULL, creationflags=flags)
            self.note = 'open%s: %s' % (' (remote link)' if host != self.config['ssh_host'] else '',
                                        ' '.join(forwards))

    def state(self):
        if self.fake:
            return 'fake (no tunnel)'
        with self.lock:
            alive = self.process is not None and self.process.poll() is None
        return self.note if alive or 'already' in self.note else 'closed'


class Hub:
    def __init__(self, config, fake):
        self.config, self.fake = config, fake
        self.tunnel = Tunnel(config, fake)
        self.helper = 'http://127.0.0.1:%d/api' % config['helper_port']
        self.lock = threading.Lock()
        self.ready = None           # the Get ready run: {'state', 'steps': [...]}

    # ---- the PC's servers -------------------------------------------------------------------

    def servers(self):
        return [{'name': s['name'], 'key': s['key'], 'port': s['port'],
                 'up': port_open('127.0.0.1', s['port'])} for s in self.config['servers']]

    def start_server(self, key):
        server = next((s for s in self.config['servers'] if s['key'] == key), None)
        if server is None:
            return 'unknown server'
        if port_open('127.0.0.1', server['port']):
            return '%s is already running' % server['name']
        if self.fake:
            return 'fake: would start %s' % server['start']
        if not os.path.exists(server['start']):
            return 'no shortcut at %s' % server['start']
        os.startfile(server['start'])           # noqa: S606 (the lab's own Startup shortcut)
        return 'starting %s' % server['name']

    def restart_server(self, key):
        """C12: a server whose port is open but hangs. Stop what listens there, start again."""
        server = next((s for s in self.config['servers'] if s['key'] == key), None)
        if server is None:
            return 'unknown server'
        pids = listening_pids(server['port'])
        if self.fake:
            return 'fake: would stop %s and start %s' % (pids or 'nothing', server['name'])
        for pid in pids:
            subprocess.run(['taskkill', '/PID', str(pid), '/T', '/F'], capture_output=True,
                           timeout=15)
        deadline = time.time() + 15
        while port_open('127.0.0.1', server['port']) and time.time() < deadline:
            time.sleep(0.5)
        if port_open('127.0.0.1', server['port']):
            return '%s is still running after stopping %s (it came back by itself?)' % (
                server['name'], pids)
        return 'stopped %s; %s' % (pids or 'nothing', self.start_server(key))

    def reset_tracking(self):
        """Restart the LiDAR odometry's tracking (25 Nov: it shook 20 cm while standing still and
        trips stalled with "Failed to make progress"). Tested mid-trip: the device stays placed."""
        if self.fake:
            return 'fake: nothing to reset'
        host, wait_s = jetson_route(self.config)
        if not host:
            return 'the Jetson cannot be reached directly or through the link'
        remote = ('source /opt/ros/humble/setup.bash && timeout 15 ros2 service call '
                  '/icp_odometry/reset_odom std_srvs/srv/Empty >/dev/null 2>&1 && echo reset '
                  '|| echo "the reset got no answer (is the device started?)"')
        words = (['ssh', '-o', 'BatchMode=yes', '-o', 'ConnectTimeout=%d' % wait_s]
                 + host_words(self.config, host) + [remote])
        try:
            done = subprocess.run(words, stdin=subprocess.DEVNULL, capture_output=True,
                                  text=True, timeout=30,
                                  creationflags=getattr(subprocess, 'CREATE_NO_WINDOW', 0))
        except (OSError, subprocess.TimeoutExpired) as error:
            return 'ssh failed: %s' % error
        said = (done.stdout or done.stderr or '').strip().splitlines()[-1:] or ['no answer']
        if said[0] == 'reset':
            return 'Position tracking reset: the dot should hold still now'
        return said[0]

    def restart_link(self):
        """Restart the Jetson's internet link (its Cloudflare tunnel) over the direct route.

        23 Nov: after an access point change the tunnel came back under a dead name. Ending
        cloudflared makes the link service start a fresh one (and report it) in about a minute.
        """
        if self.fake:
            return 'fake: no link to restart'
        host, wait_s = jetson_route(self.config)
        if not host:
            return 'the Jetson cannot be reached directly or through the link'
        # "[c]" so the pattern does not match this command itself (it killed its own reply)
        remote = ('pkill -9 -f "apnvi_tools/[c]loudflared tunnel" && echo restarted '
                  '|| echo "no link program was running"')
        words = (['ssh', '-o', 'BatchMode=yes', '-o', 'ConnectTimeout=%d' % wait_s]
                 + host_words(self.config, host) + [remote])
        try:
            done = subprocess.run(words, stdin=subprocess.DEVNULL, capture_output=True,
                                  text=True, timeout=20,
                                  creationflags=getattr(subprocess, 'CREATE_NO_WINDOW', 0))
        except (OSError, subprocess.TimeoutExpired) as error:
            return 'ssh failed: %s' % error
        said = (done.stdout or done.stderr or '').strip().splitlines()[-1:] or ['no answer']
        if said[0] == 'restarted':
            return 'The internet link is starting again: a new one in about a minute'
        return said[0]

    def start_helper(self):
        """Start the Jetson's helper by hand over ssh (when its boot service is not installed)."""
        if self.fake:
            return 'fake: run  python ../jetson/helper.py --fake  yourself'
        script = self.config.get('helper_script',
                                 '~/apnvi_demo/src/apnvi-smart-cane/console/jetson/run_helper.sh')
        host, wait_s = jetson_route(self.config)
        if not host:
            return 'the Jetson cannot be reached (not on the hotspot, no remote link)'
        port = self.config['helper_port']
        # Look for a helper on OUR port: another helper (say the boot service on 8082 while we
        # test on 8083) does not count.
        remote = ('if ss -ltn | grep -q ":%d "; then echo running; else (nohup bash %s --port %d '
                  '> /tmp/console_helper_%d.log 2>&1 < /dev/null &); echo started; fi'
                  % (port, script, port, port))
        words = (['ssh', '-o', 'BatchMode=yes', '-o', 'ConnectTimeout=%d' % wait_s]
                 + host_words(self.config, host) + [remote])
        try:
            done = subprocess.run(words, stdin=subprocess.DEVNULL, capture_output=True,
                                  text=True, timeout=20,
                                  creationflags=getattr(subprocess, 'CREATE_NO_WINDOW', 0))
        except (OSError, subprocess.TimeoutExpired) as error:
            return 'ssh failed: %s' % error
        if done.returncode != 0:
            return 'ssh said: %s' % (done.stderr.strip() or done.returncode)
        what = 'already running' if 'running' in done.stdout else 'started'
        for _ in range(20):                 # it takes a few seconds to load ROS
            code, _status = http_json(self.helper + '/status', timeout=2.0)
            if code == 200:
                return 'helper %s on port %d and answering' % (what, port)
            time.sleep(0.5)
        return ('helper %s on port %d but it does not answer through the tunnel; '
                'its log is /tmp/console_helper_%d.log on the Jetson' % (what, port, port))

    def warm_model(self):
        """Load Qwen into the graphics card now, so the first question is quick."""
        if self.fake:
            return True, 'fake: not loaded'
        model = self.config['model']
        if not model:                   # the device's own choice: model:= in its settings file
            code, status = http_json(self.helper + '/status', timeout=2.0)
            for option in (status.get('settings') or []) if code == 200 else []:
                if option.startswith('model:='):
                    model = option.split(':=', 1)[1].strip()
        if not model:
            model = DEVICE_DEFAULT_MODEL
        if not model:
            return False, 'no Qwen model found in Ollama'
        code, reply = http_json('http://127.0.0.1:11434/api/generate',
                                {'model': model, 'prompt': 'Say OK.', 'stream': False,
                                 'keep_alive': '2h'}, timeout=180)
        return code == 200, ('%s loaded' % model) if code == 200 else reply.get('error', '?')

    def status(self):
        jetson = (jetson_route(self.config)[0] is not None if not self.fake
                  else port_open('127.0.0.1', self.config['helper_port'], 0.5))
        code, _ = http_json(self.helper + '/maps', timeout=2.0)
        # 23 Nov (Adel): the laptop opens the device page at the Jetson's address on AUS
        # wireless, which changes (four in one evening): the console shows the current one.
        page_address = '' if self.fake else ipv4(self.config['jetson_address'])
        return {'servers': self.servers(), 'jetson': jetson,
                'jetson_address': self.config['jetson_address'],
                'page_link': ('http://%s:%d' % (page_address, self.config['page_port'])
                              if page_address and jetson else ''),
                'tunnel': self.tunnel.state(), 'helper': code == 200,
                # the ssh tunnel listens even when the page is down: ask the page itself
                'page': http_json('http://127.0.0.1:%d/state' % self.config['page_port'],
                                  timeout=1.5)[0] == 200,
                'fake': self.fake, 'ready': self.ready_view()}

    # ---- Get ready --------------------------------------------------------------------------

    def ready_view(self):
        with self.lock:
            return None if self.ready is None else json.loads(json.dumps(self.ready))

    def begin_ready(self, options):
        with self.lock:
            if self.ready is not None and self.ready['state'] == 'running':
                return False
            names = [s['name'] for s in self.config['servers']] + [
                'Jetson switched on and reachable', 'Link to the Jetson', 'Qwen warmed up',
                'Device started', 'Camera, laser, wheels, grip', 'Map loaded, device placed',
                '"Ready" said']
            self.ready = {'state': 'running', 'started': time.time(), 'options': options,
                          'steps': [{'name': n, 'state': 'waiting', 'detail': '', 'fix': ''}
                                    for n in names]}
        threading.Thread(target=self.run_ready, args=(options,), daemon=True).start()
        return True

    def step(self, index, state, detail='', fix=''):
        with self.lock:
            self.ready['steps'][index].update(state=state, detail=detail, fix=fix)

    def run_ready(self, options):
        ok = self.ready_steps(options)
        with self.lock:
            self.ready['state'] = 'done' if ok else 'stopped'
            self.ready['ended'] = time.time()

    def wait_for(self, test, seconds, every=1.0):
        end = time.time() + seconds
        while time.time() < end:
            value = test()
            if value:
                return value
            time.sleep(every)
        return test()

    def ready_steps(self, options):
        i = 0
        for server in self.config['servers']:                 # 1 to 3: the PC's servers
            self.step(i, 'running', 'checking')
            if self.fake and not port_open('127.0.0.1', server['port']):
                self.step(i, 'warn', 'fake: down, not started', 'start_server:' + server['key'])
                i += 1
                continue
            if not port_open('127.0.0.1', server['port']):
                self.step(i, 'running', self.start_server(server['key']))
                if not self.wait_for(lambda: port_open('127.0.0.1', server['port']),
                                     self.config['server_wait_s']):
                    self.step(i, 'fail', 'port %d still closed' % server['port'],
                              'start_server:' + server['key'])
                    return False
            self.step(i, 'ok', 'port %d open' % server['port'])
            i += 1
        self.step(i, 'running', 'switch the Jetson on if it is off; waiting')   # Jetson
        if not self.wait_for(lambda: self.status_quick('jetson'), self.config['jetson_wait_s'],
                             2.0):
            self.step(i, 'fail', 'no answer from %s' % self.config['jetson_address'], 'jetson')
            return False
        host = jetson_route(self.config)[0] if not self.fake else None
        self.step(i, 'ok', self.config['jetson_address'] if host != self.config.get(
            'remote_ssh_host') else 'through its own tunnel (remote link)')
        i += 1
        self.step(i, 'running', 'opening the ssh tunnel')                     # tunnel + helper
        if not self.wait_for(lambda: http_json(self.helper + '/maps', timeout=2.0)[0] == 200,
                             20, 2.0):
            self.step(i, 'running', 'no helper yet: ' + self.start_helper())
        if not self.wait_for(lambda: http_json(self.helper + '/maps', timeout=2.0)[0] == 200,
                             45, 2.0):
            self.step(i, 'fail', 'the console helper on the Jetson does not answer (tunnel: %s)'
                      % self.tunnel.state(), 'helper')
            return False
        self.step(i, 'ok', self.tunnel.state())
        i += 1
        self.step(i, 'running', 'loading the model (up to 2 min the first time)')   # Qwen
        warmed, detail = self.warm_model()
        self.step(i, 'ok' if warmed else 'warn', detail, '' if warmed else 'warm')
        i += 1
        if not self.start_device(i, options):                              # device
            return False
        i += 1
        self.check_parts(i)                                                # health
        i += 1
        self.check_place(i)                                                # placed
        i += 1
        said = self.wait_for(lambda: self.said_ready(), 60, 2.0)           # "Ready"
        self.step(i, 'ok' if said else 'warn', 'heard on the device' if said else
                  'not said yet: open the device page and click Sound on', '' if said else
                  'page')
        return True

    def status_quick(self, what):
        if what == 'jetson':
            if self.fake:
                return port_open('127.0.0.1', self.config['helper_port'], 0.5)
            return jetson_route(self.config)[0] is not None
        return False

    def device_status(self):
        code, status = http_json(self.helper + '/status', timeout=3.0)
        return status if code == 200 else None

    def start_device(self, i, options):
        status = self.device_status()
        if status and status.get('running'):
            wanted = 'labels:=true'         # 23 Nov (Adel): the labeller is always on
            have = status.get('options') or []
            if wanted not in have:
                self.step(i, 'warn', 'already running, but not as chosen (%s). Press Restart in '
                          'the Device panel to start it as chosen' % ' '.join(have), 'restart')
            else:
                self.step(i, 'ok', 'already running (%s)' % ' '.join(have))
            return True
        self.step(i, 'running', 'starting (about 50 s)')
        code, reply = http_json(self.helper + '/device/start', options)
        if code != 200:
            self.step(i, 'fail', reply.get('error', 'could not start'), 'restart')
            return False

        def finished():
            s = self.device_status()
            job = (s or {}).get('job') or {}
            return job if job.get('state') in ('done', 'failed') else None
        job = self.wait_for(finished, 150, 2.0)
        if not job or job.get('state') != 'done':
            last = (job or {}).get('lines', [])[-1:] or ['no answer']
            self.step(i, 'fail', 'start did not finish: %s' % last[0], 'log')
            return False
        self.step(i, 'ok', 'started')
        return True

    def check_parts(self, i):
        self.step(i, 'running', 'waiting for every part')

        def missing():
            s = self.device_status() or {}
            health = (s.get('device') or {}).get('health') or {}
            return [p for p in HEALTH_PARTS if health.get(p) is not True]
        left = missing()
        end = time.time() + 60
        while left and time.time() < end:
            time.sleep(2.0)
            left = missing()
        if left:
            self.step(i, 'warn', 'not working yet: ' + ', '.join(left), 'restart')
        else:
            self.step(i, 'ok', 'all working')

    def check_place(self, i):
        self.step(i, 'running', 'waiting for the map')

        def place():
            d = (self.device_status() or {}).get('device') or {}
            return (d.get('map_name') or ''), (d.get('placement') or '')
        end = time.time() + 30
        name, state = place()
        while state != 'placed' and time.time() < end:
            time.sleep(2.0)
            name, state = place()
        if state == 'placed':
            self.step(i, 'ok', '%s, placed' % name)
        elif state == 'searching':
            self.step(i, 'warn', '%s loaded, still finding its place: push it a few metres from '
                      'the entrance' % name, 'replace')
        else:
            self.step(i, 'warn', 'map %r, place %r' % (name, state), 'replace')

    def said_ready(self):
        """True once the device said "Ready" since this run began (Jetson clock, not the PC's)."""
        status = self.device_status() or {}
        d = status.get('device') or {}
        offset = status.get('time', time.time()) - time.time()     # Jetson clock minus PC clock
        start = (self.ready or {}).get('started', 0) + offset
        return any(text == 'Ready' and when >= start - 120 for when, text in d.get('said', []))


def make_handler(hub):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *args):
            pass

        def send_body(self, code, body, kind):
            self.send_response(code)
            self.send_header('Content-Type', kind)
            self.send_header('Content-Length', str(len(body)))
            self.send_header('Cache-Control', 'no-store')
            self.end_headers()
            self.wfile.write(body)

        def send_json(self, data, code=200):
            self.send_body(code, json.dumps(data).encode('utf-8'), 'application/json')

        def body(self):
            size = min(int(self.headers.get('Content-Length') or 0), 10000)
            try:
                data = json.loads(self.rfile.read(size) or b'{}') if size else {}
            except ValueError:
                data = {}
            return data if isinstance(data, dict) else {}

        def do_GET(self):  # noqa: N802
            path = self.path.split('?')[0]
            if path in ('/', '/index.html'):
                with open(os.path.join(HERE, 'static', 'index.html'), 'rb') as f:
                    self.send_body(200, f.read(), 'text/html; charset=utf-8')
            elif path == '/api/pc/status':
                self.send_json(hub.status())
            elif path.startswith('/api/dev/'):
                code, data = http_json(hub.helper + '/' + path[len('/api/dev/'):], timeout=8.0)
                self.send_json(*helper_reply(code, data))
            else:
                self.send_json({'error': 'unknown address'}, 404)

        def do_POST(self):  # noqa: N802
            path = self.path.split('?')[0]
            body = self.body()
            if path == '/api/pc/start_server':
                self.send_json({'said': hub.start_server(body.get('key', ''))})
            elif path == '/api/pc/restart_server':
                self.send_json({'said': hub.restart_server(body.get('key', ''))})
            elif path == '/api/pc/start_helper':
                self.send_json({'said': hub.start_helper()})
            elif path == '/api/pc/reset_tracking':
                self.send_json({'said': hub.reset_tracking()})
            elif path == '/api/pc/restart_link':
                self.send_json({'said': hub.restart_link()})
            elif path == '/api/pc/warm':
                ok, detail = hub.warm_model()
                self.send_json({'ok': ok, 'said': detail})
            elif path == '/api/pc/ready':
                options = {'preset': body.get('preset', 'guided'),
                           'speed': body.get('speed', 0.2), 'map': body.get('map', '')}
                if hub.begin_ready(options):
                    self.send_json({'ok': True})
                else:
                    self.send_json({'error': 'Get ready is already running'}, 409)
            elif path.startswith('/api/dev/'):
                code, data = http_json(hub.helper + '/' + path[len('/api/dev/'):], body,
                                       timeout=40.0)
                self.send_json(*helper_reply(code, data))
            else:
                self.send_json({'error': 'unknown address'}, 404)

    return Handler


def main():
    parser = argparse.ArgumentParser(description='APNVI console on the PC (draft)')
    parser.add_argument('--fake', action='store_true',
                        help='no ssh and no starting; use a helper run with --fake here')
    parser.add_argument('--port', type=int, default=0)
    parser.add_argument('--helper-port', type=int, default=0, help='for testing')
    parser.add_argument('--helper-script', default='', help='for testing: run_helper.sh to start')
    args = parser.parse_args()
    config = load_config()
    if args.helper_port:
        config['helper_port'] = args.helper_port
    if args.helper_script:
        config['helper_script'] = args.helper_script
    port = args.port or config['port']
    hub = Hub(config, args.fake)
    server = ThreadingHTTPServer(('127.0.0.1', port), make_handler(hub))
    print('APNVI console: http://localhost:%d%s' % (port, '  (fake)' if args.fake else ''))
    sys.stdout.flush()
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
