"""
Desk tests for the console. No ROS, no Jetson, no network beyond this computer.

Run from the repository's top folder:
  python console/tests/test_console.py

What they check: the start options, the map-command guard (the tested map can't be written over),
what diagnose() offers for each known problem, the helper's web API against the pretend device,
the whole "Get ready" run of the PC hub against the pretend device, and the real ROS link's topics
and commands through the ROS stand-in (sim/rosstub).
What they can't check: anything on the real device.
"""

import json
import os
import socket
import sys
import threading
import time
import unittest
import urllib.request
from http.server import ThreadingHTTPServer

HERE = os.path.dirname(os.path.abspath(__file__))
CONSOLE = os.path.dirname(HERE)
sys.path.insert(0, os.path.join(CONSOLE, 'jetson'))
sys.path.insert(0, os.path.join(CONSOLE, 'pc'))

import helper  # noqa: E402
import hub  # noqa: E402
import links  # noqa: E402


def free_port():
    with socket.socket() as s:
        s.bind(('127.0.0.1', 0))
        return s.getsockname()[1]


def call(port, path, body=None):
    data = None if body is None else json.dumps(body).encode()
    request = urllib.request.Request('http://127.0.0.1:%d%s' % (port, path), data=data,
                                     method='GET' if body is None else 'POST')
    try:
        with urllib.request.urlopen(request, timeout=40) as reply:
            return reply.status, json.loads(reply.read())
    except urllib.error.HTTPError as error:
        return error.code, json.loads(error.read())


def serve(handler):
    port = free_port()
    server = ThreadingHTTPServer(('127.0.0.1', port), handler)
    threading.Thread(target=server.serve_forever, daemon=True).start()
    return port, server


def wait(test, secs=20):
    end = time.time() + secs
    while time.time() < end:
        if test():
            return True
        time.sleep(0.2)
    return False


class Pieces(unittest.TestCase):
    def test_start_options(self):
        self.assertEqual(helper.start_options('mapping', 0.3), ['0.3', 'labels:=true'])
        self.assertEqual(helper.start_options('guided', '0.5'), ['0.5', 'labels:=true'])
        self.assertEqual(helper.start_options('nonsense', 9), ['0.5', 'labels:=true'])
        self.assertEqual(helper.start_options('guided', 'x', 'ESB117 Live'),
                         ['0.2', 'labels:=true', 'map:=esb117_live'])

    def test_restart_uses_the_get_ready_choices(self):
        # 22 Nov (Adel): started as Guided, Mapping demo picked, Restart: still labels:=false
        last = ['0.3', 'labels:=false']
        self.assertEqual(helper.again_options({'preset': 'mapping', 'speed': '0.3'}, last),
                         ['0.3', 'labels:=true'])
        self.assertEqual(helper.again_options({}, last), last)     # nothing sent: as before
        self.assertEqual(helper.again_options({'preset': 'guided', 'speed': 0.4},
                                              ['0.3', 'labels:=true', 'map:=outside_2']),
                         ['0.4', 'labels:=true', 'map:=outside_2'])

    def test_dead_helper_said_plainly(self):
        self.assertEqual(hub.helper_reply(200, {'a': 1}), ({'a': 1}, 200))
        data, code = hub.helper_reply(0, {'error': '[WinError 10054] closed'})
        self.assertEqual(code, 502)
        self.assertIn('Press Fix on "Link to the Jetson"', data['error'])

    def test_pin_by_hand(self):
        ok, _ = helper.check_map_command({'cmd': 'pin', 'name': 'bottle', 'x': '1.5', 'y': -2})
        self.assertEqual((ok['x'], ok['y'], ok['sure']), (1.5, -2.0, True))
        self.assertIn('numbers', helper.check_map_command({'cmd': 'pin', 'name': 'b',
                                                           'x': 'left', 'y': 1})[1])
        self.assertIsNone(helper.check_map_command({'cmd': 'pin', 'name': 'b', 'x': 1e6,
                                                    'y': 1})[0])
        self.assertIsNone(helper.check_map_command({'cmd': 'save_place', 'name': ' '})[0])
        ok, _ = helper.check_map_command({'cmd': 'set_kind', 'name': 'b', 'kind': 'sign',
                                          'x': 0, 'y': 0})
        self.assertEqual(ok['kind'], 'sign')
        self.assertIn('a pin is a', helper.check_map_command({'cmd': 'pin', 'name': 'b',
                                                              'kind': 'cat', 'x': 0, 'y': 0})[1])

    def test_goal_text(self):
        self.assertEqual(helper.goal_text({'name': ' ESB117 sign '}), ('ESB117 sign', None))
        self.assertEqual(helper.goal_text({'x': 1.234, 'y': '-2'}), ('1.23 -2.00', None))
        self.assertIsNone(helper.goal_text({'x': 'here'})[0])
        self.assertIsNone(helper.goal_text({'x': 1e6, 'y': 0})[0])

    def test_rename_saved_map(self):
        import tempfile
        with tempfile.TemporaryDirectory() as root:
            named = os.path.join(root, 'named')
            os.makedirs(named)
            os.makedirs(os.path.join(root, 'work'))
            for name in ('lab.db', 'lab.session.json', 'hall.db'):
                open(os.path.join(named, name), 'w').close()
            with open(os.path.join(named, 'lab.json'), 'w') as handle:
                json.dump({'map': 'lab', 'places': {}}, handle)
            open(os.path.join(root, 'work', 'lab_work.db'), 'w').close()
            self.assertIn('already exists', helper.rename_saved_map(root, 'lab', 'Hall'))
            self.assertIn('no saved map', helper.rename_saved_map(root, 'nope', 'x'))
            self.assertIsNone(helper.rename_saved_map(root, 'lab', 'Lab B'))
            self.assertEqual(sorted(os.listdir(named)),
                             ['hall.db', 'lab_b.db', 'lab_b.json', 'lab_b.session.json'])
            self.assertEqual(os.listdir(os.path.join(root, 'work')), ['lab_b_work.db'])
            with open(os.path.join(named, 'lab_b.json')) as handle:
                self.assertEqual(json.load(handle)['map'], 'Lab B')

    def test_delete_saved_map_keeps_the_files(self):
        import tempfile
        with tempfile.TemporaryDirectory() as root:
            for folder, name in (('named', 'lab.db'), ('named', 'lab.json'),
                                 ('named', 'lab_two.db'), ('work', 'lab_work.db')):
                os.makedirs(os.path.join(root, folder), exist_ok=True)
                open(os.path.join(root, folder, name), 'w').close()
            self.assertEqual(helper.delete_saved_map(root, 'nothing'), [])
            moved = helper.delete_saved_map(root, 'Lab', when=0)
            self.assertEqual(sorted(os.path.basename(f) for f in moved),
                             ['lab.db', 'lab.json', 'lab_work.db'])
            self.assertEqual(os.listdir(os.path.join(root, 'named')), ['lab_two.db'])
            kept = os.listdir(os.path.join(root, 'deleted'))
            self.assertEqual(sorted(os.listdir(os.path.join(root, 'deleted', kept[0]))),
                             ['lab.db', 'lab.json', 'lab_work.db'])

    def test_map_guard(self):
        ok, why = helper.check_map_command({'cmd': 'save_map', 'name': 'ESB117 final'})
        self.assertIsNone(ok)
        self.assertIn('tested map', why)
        ok, _ = helper.check_map_command({'cmd': 'save_map', 'name': 'esb117_final',
                                          'force': True})
        self.assertEqual(ok['cmd'], 'save_map')
        self.assertIsNone(helper.check_map_command({'cmd': 'load_map', 'name': ''})[0])
        self.assertIsNone(helper.check_map_command({'cmd': 'rename_pin', 'name': 'a'})[0])
        # 23 Nov: never every pin of a name from the console
        self.assertIn('which pin', helper.check_map_command({'cmd': 'remove_pin',
                                                             'name': 'table'})[1])
        self.assertIn('which pin', helper.check_map_command({'cmd': 'rename_pin', 'name': 'table',
                                                             'to': 'desk'})[1])
        ok, _ = helper.check_map_command({'cmd': 'remove_pin', 'name': 'table', 'x': 1, 'y': 2})
        self.assertEqual((ok['x'], ok['y']), (1.0, 2.0))
        self.assertIsNone(helper.check_map_command({'cmd': 'drive', 'name': 'a'})[0])
        ok, _ = helper.check_map_command({'cmd': 'set_entrance'})
        self.assertEqual(ok, {'cmd': 'set_entrance', 'name': 'entrance'})

    def test_diagnose(self):
        state = links.State()
        self.assertEqual(links.diagnose(state.snapshot(), running=False), [])
        problems = links.diagnose(state.snapshot(), running=True)
        self.assertEqual([p['fix'] for p in problems], ['restart'])     # no health report
        state.put('health', {'camera': True, 'handle': False, 'place': True})
        state.put('grip_connected', False)
        state.put('placement', 'lost')
        state.put('guide', {'state': 'following', 'route_ok': False})
        fixes = [p['fix'] for p in links.diagnose(state.snapshot(), running=True)]
        self.assertEqual(fixes, ['reset_master', 'replace', 'clear'])

    def test_fit_from_notes(self):
        state = links.State()
        state.add('notes', 'clearly best of 6 places (71 %, next 57 %)')
        self.assertEqual(state.snapshot()['fit'], 71.0)
        # the map manager's own lines (placing.py, camera_guess.py, relocalizer.Fit), 19 Nov
        cases = {
            'searched the map, sure: Fit(x=1.20 y=0.40 yaw=10.0 deg score=0.78 '
            'runner_up=0.41)': 78.0,
            'not placed yet; outline 80 points, 2.1 m; following (1.0, 2.0) 55%, '
            'start spot (0.0, 0.0) 68%': 68.0,
            'camera guess seen twice, walls fit 74%': 74.0,
            'outline 80 points, 2.1 m': None,
        }
        for note, fit in cases.items():
            self.assertEqual(links.note_fit(note), fit, note)

    def test_tail_log_drops_known_noise(self):
        import tempfile
        lines = ['[nice-12] [ WARN] MsgConversion.cpp:3386::deskew_impl() A point has a stamp'] * 50
        lines += ['[map_manager_node]: placed', '[guide_node] [ERROR] no route to ESB117']
        lines += ['[esp32_reader-2] [WARN] Cannot open /dev/serial/by-id/stage1-esp32-not-plugged']
        with tempfile.NamedTemporaryFile('w', suffix='.log', delete=False) as f:
            f.write('\n'.join(lines) + '\n')
        try:
            out = helper.tail_log(f.name)
        finally:
            os.unlink(f.name)
        self.assertEqual(out['warnings'], ['[guide_node] [ERROR] no route to ESB117'])
        self.assertEqual(out['tail'][0], '[map_manager_node]: placed')

    def test_deaf_watch(self):
        watch = helper.DeafWatch(deaf_s=90)
        self.assertFalse(watch.deaf(True, 0, 0))        # starts the wait
        self.assertFalse(watch.deaf(True, 0, 60))
        self.assertTrue(watch.deaf(True, 0, 91))        # running, nothing for 91 s
        watch = helper.DeafWatch(deaf_s=90)
        watch.deaf(True, 0, 0)
        self.assertFalse(watch.deaf(True, 5, 91))       # messages came: the wait starts again
        self.assertFalse(watch.deaf(True, 5, 150))
        self.assertFalse(watch.deaf(False, 5, 400))     # device stopped: nothing to hear
        self.assertFalse(watch.deaf(True, 5, 401))
        self.assertFalse(watch.deaf(True, 5, 480))
        self.assertTrue(watch.deaf(True, 5, 492))

    def test_listening_pids(self):
        text = ('  Proto  Local Address          Foreign Address        State           PID\n'
                '  TCP    127.0.0.1:8090         0.0.0.0:0              LISTENING       4412\n'
                '  TCP    0.0.0.0:8090           0.0.0.0:0              LISTENING       4412\n'
                '  TCP    127.0.0.1:18090        0.0.0.0:0              LISTENING       999\n'
                '  TCP    127.0.0.1:8090         127.0.0.1:50122        ESTABLISHED     4412\n'
                '  TCP    [::]:8091              [::]:0                 LISTENING       5120\n')
        self.assertEqual(hub.listening_pids(8090, text), [4412])
        self.assertEqual(hub.listening_pids(8091, text), [5120])
        self.assertEqual(hub.listening_pids(11434, text), [])

    def test_ssh_hostname(self):
        self.assertIsInstance(hub.ssh_hostname('no-such-host-here'), str)

    def test_jetson_route_hotspot_first_then_remote_link(self):
        """Remote link: direct when the Jetson answers, else its fresh tunnel."""
        import tempfile
        listener = socket.socket()
        listener.bind(('127.0.0.1', 0))
        listener.listen(1)
        open_port = listener.getsockname()[1]
        listener.close()                                  # a port nothing listens on now
        with tempfile.TemporaryDirectory() as folder:
            kept = os.path.join(folder, 'jetson.json')
            config = dict(hub.DEFAULTS, jetson_address='127.0.0.1', jetson_file=kept)
            real_port_open = hub.port_open
            try:
                hub.port_open = lambda host, port, timeout=1.0: False      # not on the hotspot
                self.assertEqual(hub.jetson_route(config), (None, None))   # and never reported
                with open(kept, 'w') as f:
                    json.dump({'ssh': 'https://a.trycloudflare.com', 'at': time.time()}, f)
                self.assertEqual(hub.jetson_route(config), ('jetson-remote', 30))
                with open(kept, 'w') as f:
                    json.dump({'ssh': 'https://a.trycloudflare.com', 'at': time.time() - 600}, f)
                self.assertEqual(hub.jetson_route(config), (None, None))   # news too old
                hub.port_open = lambda host, port, timeout=1.0: True       # on the hotspot
                self.assertEqual(hub.jetson_route(config), ('jetson', 5))
                self.assertEqual(hub.jetson_route(dict(config, remote_ssh_host='')),
                                 ('jetson', 5))
            finally:
                hub.port_open = real_port_open
        self.assertFalse(hub.port_open('127.0.0.1', open_port, 0.3))


class HelperApi(unittest.TestCase):
    """The helper's web API against the pretend device."""

    @classmethod
    def setUpClass(cls):
        cls.state = links.State()
        cls.link = links.FakeLink(cls.state)
        cls.device = helper.Device(fake_link=cls.link)
        cls.port, cls.server = serve(helper.make_handler(cls.state, cls.link, cls.device))

    def test_flow(self):
        port = self.port
        code, body = call(port, '/api/map', {'cmd': 'new_map'})
        self.assertEqual(code, 409)                         # device not running yet
        self.assertEqual(call(port, '/api/device/start', {'preset': 'mapping'})[0], 200)
        self.assertEqual(call(port, '/api/device/stop', {})[0], 409)    # one job at a time
        self.assertTrue(wait(lambda: call(port, '/api/status')[1]['running']))
        self.assertTrue(wait(lambda: call(port, '/api/status')[1]['job']['state'] == 'done'))
        self.assertEqual(call(port, '/api/status')[1]['options'], ['0.3', 'labels:=true'])
        self.assertEqual(call(port, '/api/map', {'cmd': 'new_map'})[1]['reply'],
                         'Started a new map.')
        self.assertEqual(call(port, '/api/replace', {})[0], 409)        # nothing saved loaded
        self.assertEqual(call(port, '/api/map', {'cmd': 'save_map', 'name': 'esb117_final'})[0],
                         400)
        self.assertEqual(call(port, '/api/map', {'cmd': 'save_map', 'name': 'esb117_live'})[1]
                         ['reply'], 'Map saved as esb117_live')
        self.assertIn('esb117_live', call(port, '/api/maps')[1]['maps'])
        self.assertTrue(wait(lambda: call(port, '/api/status')[1]['device']
                             .get('map_name') == 'esb117_live'))
        self.assertEqual(call(port, '/api/replace', {})[1]['reply'], 'Loaded esb117_live')
        self.assertTrue(wait(lambda: call(port, '/api/status')[1]['device']
                             .get('placement') == 'placed', 30))
        said = [t for _, t in call(port, '/api/status')[1]['device']['said']]
        self.assertIn('Finding where you are. Keep walking.', said)
        self.assertIn('Found where you are.', said)
        self.assertEqual(call(port, '/api/signal', {'name': 'forward'})[0], 400)
        self.assertEqual(call(port, '/api/signal', {'name': 'left', 'secs': 99})[0], 200)
        self.assertEqual(call(port, '/api/stop', {})[0], 200)
        self.assertEqual(call(port, '/api/nothing', {})[0], 404)
        self.link.trouble = {'guide': 'guiding'}           # a trip is on: no map change
        self.assertTrue(wait(lambda: (call(port, '/api/status')[1]['device'].get('guide') or {})
                             .get('state') == 'guiding'))
        code, body = call(port, '/api/map', {'cmd': 'load_map', 'name': 'esb117_final'})
        self.assertEqual(code, 409)
        self.assertIn('STOP', body['error'])
        self.link.trouble = {}
        # pins and maps by hand
        self.assertEqual(call(port, '/api/map', {'cmd': 'pin', 'name': 'cup', 'x': 1, 'y': 2})[1]
                         ['reply'], 'Pinned cup (thing) at 1.0, 2.0')
        self.assertEqual(call(port, '/api/map', {'cmd': 'set_kind', 'name': 'cup', 'kind': 'sign',
                                                 'x': 1, 'y': 2})[1]['reply'],
                         'Pinned cup (sign) at 1.0, 2.0')
        # 23 Nov: removing a clicked pin takes that one off, not every pin of its name
        call(port, '/api/map', {'cmd': 'pin', 'name': 'cup', 'kind': 'thing', 'x': 5, 'y': 5})
        self.assertEqual(call(port, '/api/map', {'cmd': 'remove_pin', 'name': 'cup', 'x': 5.1,
                                                 'y': 5})[1]['reply'], 'Removed cup from the map')
        self.assertTrue(wait(lambda: [(p['x'], p['y']) for p in call(port, '/api/status')[1]
                                      ['device']['pins'] if p['name'] == 'cup'] == [(1, 2)]))
        self.assertEqual(call(port, '/api/goal', {'x': 1, 'y': 2})[1]['reply'],
                         'Trip asked: 1.00 2.00')
        self.assertEqual(call(port, '/api/goal', {})[0], 400)
        picture = call(port, '/api/map/picture')[1]
        dots = {p['name']: p for p in picture['view']['pins']}
        self.assertEqual(dots['cup']['kind'], 'sign')
        self.assertEqual((dots['cup']['col'], dots['cup']['row']), (352, 96))  # (1, 2) m
        self.assertEqual(call(port, '/api/map/rename_map', {'name': 'esb117_final',
                                                            'to': 'x'})[0], 409)
        self.assertEqual(call(port, '/api/map/rename_map', {'name': 'esb117_device',
                                                            'to': 'Old device'})[1]['reply'],
                         'esb117_device is now called old_device')
        self.assertIn('old_device', call(port, '/api/maps')[1]['maps'])
        self.assertEqual(call(port, '/api/map', {'cmd': 'save_place', 'name': 'desk'})[1]
                         ['reply'], 'Saved this place as desk')
        self.assertEqual(call(port, '/api/map/delete', {'name': 'esb117_final'})[0], 409)
        self.assertEqual(call(port, '/api/map/delete', {'name': 'esb117_live'})[0], 409)  # in use
        self.assertEqual(call(port, '/api/map/delete', {'name': 'old_device'})[0], 200)
        self.assertNotIn('old_device', call(port, '/api/maps')[1]['maps'])
        self.assertEqual(call(port, '/api/map/delete', {'name': 'old_device'})[0], 404)


class HubGetReady(unittest.TestCase):
    """The PC hub's whole Get ready run, against a pretend device on this computer."""

    def test_get_ready(self):
        state = links.State()
        link = links.FakeLink(state)
        helper_port, _ = serve(helper.make_handler(state, link, helper.Device(fake_link=link)))
        config = hub.load_config()
        config['helper_port'] = helper_port
        config['jetson_address'] = '127.0.0.1'
        for server in config['servers']:                   # pretend PC servers: open ports
            listener = socket.socket()
            listener.bind(('127.0.0.1', 0))
            listener.listen(5)
            server['port'] = listener.getsockname()[1]
            server['_keep'] = listener
        the_hub = hub.Hub(config, fake=True)
        hub_port, _ = serve(hub.make_handler(the_hub))
        self.assertEqual(call(hub_port, '/api/pc/ready', {'preset': 'guided'})[0], 200)
        self.assertEqual(call(hub_port, '/api/pc/ready', {'preset': 'guided'})[0], 409)
        self.assertTrue(wait(lambda: call(hub_port, '/api/pc/status')[1]['ready']['state']
                             != 'running', 120))
        ready = call(hub_port, '/api/pc/status')[1]['ready']
        states = [s['state'] for s in ready['steps']]
        self.assertEqual(ready['state'], 'done', ready)
        self.assertEqual(states, ['ok'] * len(states), ready)
        self.assertTrue(call(hub_port, '/api/dev/status')[1]['running'])     # through the hub


class RosLinkThroughStandIn(unittest.TestCase):
    """The real ROS link, with sim/rosstub standing in for ROS 2."""

    def test_topics_and_commands(self):
        rosstub = os.path.normpath(os.path.join(CONSOLE, '..', 'sim', 'rosstub'))
        sys.path.insert(0, rosstub)
        try:
            import rosstub_core
            from rclpy.node import Node
            from std_msgs.msg import Bool, String
        except ImportError as error:
            self.skipTest('rosstub not usable here: %s' % error)
        state = links.State()
        link = links.RosLink(state, '/nonexistent', spin=False)
        other = Node('test_sender')
        other.create_publisher(String, '/stage3/placement').publish(String(data='searching'))
        other.create_publisher(String, '/stage3/pins').publish(String(data=json.dumps(
            {'pins': [{'name': 'bin', 'kind': 'thing', 'x': 1, 'y': 2}], 'waiting': 2})))
        other.create_publisher(Bool, '/grip/connected').publish(Bool(data=False))
        other.create_publisher(String, '/stage4/health').publish(String(data=json.dumps(
            {'camera': True, 'handle': False})))
        other.create_publisher(String, '/voice/say').publish(String(data='Ready'))
        snap = state.snapshot()
        self.assertEqual(snap['placement'], 'searching')
        self.assertEqual(snap['pins'][0]['name'], 'bin')
        self.assertEqual(snap['pins_waiting'], 2)
        self.assertIs(snap['grip_connected'], False)
        self.assertEqual(snap['said'][-1][1], 'Ready')
        rosstub_core.PUBLISHED.clear()
        link.map_command({'cmd': 'load_map', 'name': 'esb117_live'})
        link.signal('near', 3)
        link.stop_moving()
        sent = [(topic, getattr(msg, 'data', None)) for topic, msg in rosstub_core.PUBLISHED]
        self.assertIn(('/stage3/command', '{"cmd": "load_map", "name": "esb117_live"}'), sent)
        self.assertIn(('/stage4/vibration_test', 'near 3.0'), sent)
        self.assertIn(('/stage4/goal', 'cancel'), sent)
        twist = [msg for topic, msg in rosstub_core.PUBLISHED if topic == '/cmd_vel_nav'][0]
        self.assertEqual((twist.linear.x, twist.angular.z), (0.0, 0.0))   # STOP sends only zero


if __name__ == '__main__':
    unittest.main(verbosity=2)
