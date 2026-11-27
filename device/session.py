#!/usr/bin/env python3
"""
Test-session tools for Stage 4 step 2 (and later steps). Run on the Jetson with ROS sourced.

  python3 session.py health              what runs, Stage 1 rate, position, master state
  python3 session.py where               where the device is on the map (x y metres, heading)
  python3 session.py drive 0.2 5         drive straight at 0.2 m/s for 5 s (through the hazard
                                         gate), then report how far it went and the real speed
  python3 session.py turn 0.5 3          turn on the spot at 0.5 rad/s (left) for 3 s
  python3 session.py goal "4 0"          take me to the map point 4 0 ("cancel" drops the route)
  python3 session.py ask continue        a follow-up, as if the user said "continue"
  python3 session.py ask other           any other question (drops a paused route)
  python3 session.py map save NAME       save the running map (map load NAME, map new)
  python3 session.py record              log everything to ~/apnvi_logs/session_<time>.log
                                         until Ctrl+C (the folder survives a restart)
  python3 session.py clear               empty both planners' obstacle maps (old camera marks)
  python3 session.py snap                save the camera's picture now to ~/apnvi_logs/snaps/
                                         (photos for tools/try_labels.py)
  python3 session.py say "where is the door"   a typed question, as if spoken; prints what the
                                         device says for 25 s ("take me to the door" drives!)
  python3 session.py pins                the map's pins (places, signs, things)
  python3 session.py pin remove NAME     take a pin off the map; pin rename OLD to NEW
  python3 session.py place NAME          "save this place as NAME" (pins where the device is)
  python3 session.py scan                "scan the room": the labeller looks all round for 60 s
  python3 session.py signal left 3       play one vibration signal for 3 s: careful, stopped,
                                         left, right, heard, arrived, help; motor1 / motor2
                                         run one grip motor (which side is it on?)
  python3 session.py ahead 1.5           is something 1.5 m ahead on the two planners' maps?
                                         (the camera layer check)
  python3 session.py reset               restart the master ESP32 over USB; only
                                         while the drive link is stopped (stop.sh first)

Nothing here sends wheel power directly: drive and turn send a speed on /cmd_vel_nav, the same
way the route follower does, so the hazard gate, the stops and the speed cap all still apply.
"""

import glob
import json
import math
import os
import sys
import time

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import Bool, Float32, String, UInt8

LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)


def heading_deg(pose):
    """The heading of a pose in degrees (0 = the map's x direction, + = to the left)."""
    q = pose.orientation
    return math.degrees(math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)))


class Tools(Node):
    """Listens to everything the tools need; keeps the newest of each."""

    def __init__(self):
        super().__init__('session_tools')
        self.t0 = time.time()
        self.now = {}
        self.counts = {}
        self.events = []
        self.listen(PoseStamped, '/stage3/position', 'position')
        self.listen(String, '/stage3/placement', 'placement', LATCHED)
        self.listen(String, '/stage3/map_name', 'map_name', LATCHED)
        self.listen(String, '/stage3/reply', 'reply', event=True)
        self.listen(String, '/hazard/level', 'level')
        self.listen(Float32, '/hazard/distance', 'distance')
        self.listen(Bool, '/stage4/hazard', 'blocked', LATCHED)
        self.listen(UInt8, '/stage4/vibration', 'vibration')
        self.vibe_ticks = [0, 0, 0]          # ticks this second: on, all, strongest
        self.create_subscription(UInt8, '/stage4/vibration', self.on_vibe, 50)
        self.listen(Path, '/stage4/route', 'route', LATCHED)
        self.listen(String, '/stage4/drive_status', 'status')
        self.listen(String, '/stage4/master', 'master', event=True)
        self.listen(String, '/stage4/guide_state', 'guide', LATCHED)
        self.listen(String, '/voice/say', 'say', event=True)
        self.listen(Twist, '/cmd_vel_nav', 'asked')
        self.listen(Twist, '/stage4/cmd_vel', 'gate_out')
        self.cmd = self.create_publisher(Twist, '/cmd_vel_nav', 10)
        self.goal_pub = self.create_publisher(String, '/stage4/goal', 10)
        self.request_pub = self.create_publisher(String, '/stage2/request', 10)
        self.map_pub = self.create_publisher(String, '/stage3/command', 10)
        self.listen(String, '/stage3/pins', 'pins', LATCHED)
        self.listen(String, '/stage4/vibration_signal', 'signal', event=True)
        self.listen(String, '/stage4/free_mode', 'free')
        self.listen(String, '/stage4/health', 'health')
        self.listen(String, '/stage2/labels', 'labels', event=True)
        self.listen(String, '/stage2/located', 'located', event=True)
        self.listen(String, '/stage2/status', 'stage2')
        self.listen(Odometry, '/odom', 'odom')
        self.question_pub = self.create_publisher(String, '/voice/question', 10)
        self.signal_pub = self.create_publisher(String, '/stage4/vibration_test', 10)

    def on_vibe(self, msg):
        self.vibe_ticks[0] += msg.data > 0
        self.vibe_ticks[1] += 1
        self.vibe_ticks[2] = max(self.vibe_ticks[2], msg.data)

    def listen(self, kind, topic, key, qos=20, event=False):
        def got(msg):
            self.now[key] = msg
            self.counts[key] = self.counts.get(key, 0) + 1
            if event:
                self.events.append((time.time(), key, msg.data))
        self.create_subscription(kind, topic, got, qos)

    def send(self, pub, msg):
        """Publish once, after waiting (up to 5 s) until someone listens: else it is lost."""
        end = time.time() + 5
        while pub.get_subscription_count() == 0 and time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.1)
        if pub.get_subscription_count() == 0:
            print('!! nobody listens on %s: is the device running?' % pub.topic_name)
        self.spin_for(0.3)
        pub.publish(msg)

    def spin_for(self, secs):
        end = time.time() + secs
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.05)

    def position(self):
        msg = self.now.get('position')
        if msg is None:
            return None
        p = msg.pose.position
        return p.x, p.y, heading_deg(msg.pose)

    def status(self):
        try:
            return json.loads(self.now['status'].data)
        except (KeyError, ValueError):
            return {}


def show_position(tools):
    where = tools.position()
    if where is None:
        return 'no position yet (is Stage 3 running?)'
    return 'x %.2f m, y %.2f m, heading %.0f deg' % where


def health(tools):
    tools.spin_for(2.0)                     # let the topics connect before counting
    tools.counts.clear()
    tools.spin_for(5.0)
    rate = {k: tools.counts.get(k, 0) / 5.0 for k in ('level', 'position', 'status')}
    s = tools.status()
    print('Stage 1 (hazard level): %.1f a second (want 10)' % rate['level'])
    print('Stage 3 (position):     %.1f a second (want 5)  ->  %s' % (
        rate['position'], show_position(tools)))
    placement = tools.now.get('placement')
    name = tools.now.get('map_name')
    print('Map: %s, %s' % (repr(name.data) if name else '?',
                          placement.data if placement else 'drawing a new map'))
    print('Master: connected %s, is master %s, ready %s, stops %s, grip %s, restarts %s' % (
        s.get('connected'), s.get('master'), s.get('ready'), s.get('stops'),
        'on' if s.get('handle') else 'NOT HEARD', s.get('resets', 0)))
    level, dist = tools.now.get('level'), tools.now.get('distance')
    print('Stage 1 now: %s, %s' % (level.data if level else '?',
                                  'nothing ahead' if dist is None or dist.data < 0
                                  else '%.2f m from the camera' % dist.data))
    guide = tools.now.get('guide')
    print('Guide: %s' % (guide.data if guide else '?'))
    parts = tools.now.get('health')
    print('Health: %s' % (parts.data if parts else 'the health program is not running'))
    pins = pin_list(tools)
    print('Pins on the map: %d%s' % (len(pins[0]), ' (and %d seen only once so far)' % pins[1]
                                     if pins[1] else ''))
    stage2 = tools.now.get('stage2')
    print('Stage 2: %s' % (stage2.data if stage2 else 'nothing heard yet (it speaks when asked)'))


def drive(tools, forward, turn, secs):
    end = time.time() + 5
    while tools.cmd.get_subscription_count() == 0 and time.time() < end:
        tools.spin_for(0.1)
    tools.spin_for(1.5)
    start = tools.position()
    print('start:', show_position(tools))
    t_start = time.time()
    blocked_seen = False
    while time.time() - t_start < secs:
        msg = Twist()
        msg.linear.x, msg.angular.z = forward, turn
        tools.cmd.publish(msg)
        tools.spin_for(0.1)
        b = tools.now.get('blocked')
        if b is not None and b.data and forward > 0 and not blocked_seen:
            blocked_seen = True
            print('  !! forward blocked by the hazard gate at %.1f s (Stage 1: %s)' % (
                time.time() - t_start, tools.now['level'].data if 'level' in tools.now else '?'))
    for _ in range(5):
        tools.cmd.publish(Twist())
        tools.spin_for(0.1)
    tools.spin_for(1.0)
    end = tools.position()
    print('end:  ', show_position(tools))
    s = tools.status()
    print('wheel power at the end: left %s%%, right %s%%, stops %s' % (
        s.get('left_pct'), s.get('right_pct'), s.get('stops')))
    if start and end:
        dx, dy = end[0] - start[0], end[1] - start[1]
        moved = math.hypot(dx, dy)
        along = dx * math.cos(math.radians(start[2])) + dy * math.sin(math.radians(start[2]))
        turned = (end[2] - start[2] + 180) % 360 - 180
        print('moved %.2f m (%.2f m along the way it faced: + = forward), turned %.0f deg' % (
            moved, along, turned))
        if forward:
            print('real speed about %.2f m/s for %.2f m/s asked (includes the 0.5 s start)' % (
                along / secs, forward))
        if turn:
            print('real turn about %.0f deg/s for %.0f deg/s asked' % (
                turned / secs, math.degrees(turn)))


def route_shape(path):
    """Sum up a route: (points, end x, end y, widest bend from the straight line in metres)."""
    pts = [(p.pose.position.x, p.pose.position.y) for p in path.poses]
    if len(pts) < 2:
        return (len(pts), 0.0, 0.0, 0.0)
    (x0, y0), (x1, y1) = pts[0], pts[-1]
    length = math.hypot(x1 - x0, y1 - y0) or 1e-6
    bend = max(abs((x1 - x0) * (y0 - y) - (x0 - x) * (y1 - y0)) / length for x, y in pts)
    return (len(pts), x1, y1, bend)


def record(tools):
    folder = os.path.expanduser('~/apnvi_logs')
    os.makedirs(folder, exist_ok=True)
    path = os.path.join(folder, time.strftime('session_%d%b_%H%M%S.log'))
    print('recording to', path, '(Ctrl+C to stop)')
    last = {}
    next_line = time.time()
    with open(path, 'w') as log:
        def write(text):
            line = '%s %s' % (time.strftime('%H:%M:%S'), text)
            log.write(line + '\n')
            log.flush()
            print(line, flush=True)

        def changed(key, value):
            if last.get(key) != value:
                last[key] = value
                return True
            return False
        try:
            while True:
                tools.spin_for(0.1)
                for _, key, text in tools.events:
                    if key in ('master', 'say', 'reply', 'signal', 'labels', 'located'):
                        write('%-7s %s' % (key, text))
                tools.events.clear()
                guide = tools.now.get('guide')
                if guide and changed('guide', guide.data):
                    write('guide   ' + guide.data)
                b = tools.now.get('blocked')
                if b and changed('blocked', b.data):
                    write('hazard  forward %s' % ('BLOCKED' if b.data else 'free'))
                r = tools.now.get('route')
                if r is not None:
                    n, ex, ey, bend = route_shape(r)
                    old = last.get('route_shape')
                    if old is None or n == 0 or (old[0] == 0) != (n == 0) or                             abs(bend - old[3]) > 0.15 or math.hypot(ex - old[1], ey - old[2]) > 0.3:
                        last['route_shape'] = (n, ex, ey, bend)
                        if changed('route_text', (n, round(ex, 1), round(ey, 1), round(bend, 1))):
                            write('route   %d points to (%.2f, %.2f), bends %.2f m off the '
                                  'straight line' % (n, ex, ey, bend))
                p = tools.now.get('placement')
                if p and changed('placement', p.data):
                    write('map     ' + p.data)
                free = tools.now.get('free')
                if free:
                    try:
                        why = json.loads(free.data)
                    except ValueError:
                        why = {}
                    if changed('free', (why.get('why'), why.get('turn_rps'))):
                        write('free    %s turn %s rad/s push %s m/s' % (
                            why.get('why'), why.get('turn_rps'), why.get('push_mps')))
                parts = tools.now.get('health')
                if parts and changed('health', parts.data):
                    write('health  ' + parts.data)
                pins = tools.now.get('pins')
                if pins and changed('pins', pins.data):
                    names, waiting = pin_list(tools)
                    write('pins    %d: %s%s' % (len(names), ', '.join(names),
                                               ' (+%d seen once)' % waiting if waiting else ''))
                s = tools.status()
                key = (s.get('left_pct'), s.get('right_pct'), tuple(s.get('stops', [])),
                       s.get('connected'), s.get('handle'))
                if s and changed('status', key):
                    write('wheels  left %s%% right %s%% stops %s connected %s grip %s' % key)
                if time.time() >= next_line:      # once a second: where, Stage 1, rates
                    next_line += 1.0
                    level, dist = tools.now.get('level'), tools.now.get('distance')
                    asked = tools.now.get('asked')
                    on, ticks, top = tools.vibe_ticks
                    tools.vibe_ticks = [0, 0, 0]
                    write('every1s %s | stage1 %s %s | vibe on %d/%d at %d | asked %s | rate '
                          'stage1 %d/s' % (
                              show_position(tools), level.data if level else '?',
                              '-' if dist is None else '%.2f' % dist.data, on, ticks, top,
                              '-' if asked is None else '%.2f %.2f' % (
                                  asked.linear.x, asked.angular.z),
                              tools.counts.get('level', 0)))
                    tools.counts['level'] = 0
        except KeyboardInterrupt:
            write('recording stopped')


def map_command(tools, words):
    cmd = {'save': 'save_map', 'load': 'load_map', 'new': 'new_map'}.get(words[0])
    if cmd is None:
        sys.exit('map save NAME | map load NAME | map new')
    tools.send(tools.map_pub, String(data=json.dumps({'cmd': cmd, 'name': ' '.join(words[1:])})))
    print('sent', cmd, ' '.join(words[1:]), '- waiting up to 30 s for the reply')
    end = time.time() + 30
    while time.time() < end:
        tools.spin_for(0.2)
        replies = [e for e in tools.events if e[1] == 'reply']
        if replies:
            print('Stage 3:', replies[-1][2])
            break
    tools.spin_for(2.0)
    p = tools.now.get('placement')
    print('placement:', p.data if p else '-', '|', show_position(tools))


def pin_list(tools):
    """The pins as (list of "name (kind) x y", how many are waiting for a second look)."""
    msg = tools.now.get('pins')
    if msg is None:
        return [], 0
    try:
        data = json.loads(msg.data)
    except ValueError:
        return [], 0
    names = ['%s (%s) %.1f %.1f' % (p['name'], p.get('kind', '?'), p['x'], p['y'])
             for p in data.get('pins', [])]
    return names, int(data.get('waiting', 0))


def show_pins(tools):
    tools.spin_for(2.0)
    names, waiting = pin_list(tools)
    name = tools.now.get('map_name')
    print('map %s: %d pins' % (repr(name.data) if name else '?', len(names)))
    for line in names:
        print('  ' + line)
    if waiting:
        print('  (%d more things were seen once; a second look pins them)' % waiting)


def pin_command(tools, words):
    if len(words) >= 2 and words[0] == 'remove':
        command = {'cmd': 'remove_pin', 'name': ' '.join(words[1:])}
    elif len(words) >= 4 and words[0] == 'rename' and 'to' in words[2:]:
        split = len(words) - 1 - words[::-1].index('to')
        command = {'cmd': 'rename_pin', 'name': ' '.join(words[1:split]),
                   'to': ' '.join(words[split + 1:])}
    else:
        sys.exit('pin remove NAME | pin rename OLD to NEW')
    tools.send(tools.map_pub, String(data=json.dumps(command)))
    tools.spin_for(2.0)
    for _, key, text in tools.events:
        if key == 'reply':
            print('Stage 3:', text)
    show_pins(tools)


def listen_for(tools, secs, keys=('say', 'reply', 'located', 'labels', 'signal')):
    """Print what the device says and does for a while."""
    end = time.time() + secs
    while time.time() < end:
        tools.spin_for(0.2)
        for _, key, text in tools.events:
            if key in keys:
                print('%s %-7s %s' % (time.strftime('%H:%M:%S'), key, text))
        tools.events.clear()


def say(tools, question):
    tools.events.clear()
    tools.send(tools.question_pub, String(data=question))
    print('asked: "%s" (listening for 25 s)' % question)
    listen_for(tools, 25)
    print('guide:', tools.now['guide'].data if 'guide' in tools.now else '?')


def scan(tools):
    tools.spin_for(1.0)
    before = len(pin_list(tools)[0])
    tools.events.clear()
    tools.send(tools.map_pub, String(data=json.dumps({'cmd': 'scan_room'})))
    print('scan started: turn the device slowly all the way round (65 s)')
    listen_for(tools, 65)
    print('pins before %d, now %d' % (before, len(pin_list(tools)[0])))
    show_pins(tools)


def signal(tools, words):
    name = words[0] if words else 'off'
    secs = float(words[1]) if len(words) > 1 else 3.0
    tools.send(tools.signal_pub, String(data='%s %s' % (name, secs)))
    print('playing "%s" for %.0f s' % (name, secs))
    tools.spin_for(secs + 0.5)


def ahead(tools, metres):
    """Look at a spot `metres` ahead of the wheels on the two planners' obstacle maps."""
    grids = {}
    for topic in ('/global_costmap/costmap', '/local_costmap/costmap'):
        tools.create_subscription(OccupancyGrid, topic,
                                  lambda m, t=topic: grids.__setitem__(t, m), LATCHED)
    end = time.time() + 6
    while time.time() < end and len(grids) < 2:
        tools.spin_for(0.2)
    odom = tools.now.get('odom')
    poses = {'map': tools.position()}
    if odom is not None:
        p = odom.pose.pose
        poses['odom'] = (p.position.x, p.position.y, heading_deg(p))
    for topic in ('/global_costmap/costmap', '/local_costmap/costmap'):
        grid = grids.get(topic)
        if grid is None:
            print('%s: no map received (is that planner running?)' % topic)
            continue
        pose = poses.get(grid.header.frame_id)
        if pose is None:
            print('%s: its frame is "%s" and the device\'s place in it is not known' % (
                topic, grid.header.frame_id))
            continue
        x = pose[0] + metres * math.cos(math.radians(pose[2]))
        y = pose[1] + metres * math.sin(math.radians(pose[2]))
        info = grid.info
        worst = -1
        for dx in (-0.2, -0.1, 0.0, 0.1, 0.2):
            for dy in (-0.2, -0.1, 0.0, 0.1, 0.2):
                col = int((x + dx - info.origin.position.x) / info.resolution)
                row = int((y + dy - info.origin.position.y) / info.resolution)
                if 0 <= col < info.width and 0 <= row < info.height:
                    worst = max(worst, grid.data[row * info.width + col])
        print('%s (frame %s): %.1f m ahead is %s (cost %d of 100)' % (
            topic, grid.header.frame_id, metres,
            'outside this map' if worst < 0 else 'BLOCKED' if worst >= 99
            else 'near something' if worst > 0 else 'free', worst))


def snap(tools):
    """Save the camera's colour picture as a JPEG (for trying the labeller's question)."""
    import cv2
    import numpy as np
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Image
    got = {}
    tools.create_subscription(Image, '/camera/camera/color/image_raw',
                              lambda m: got.setdefault('picture', m), qos_profile_sensor_data)
    end = time.time() + 5
    while 'picture' not in got and time.time() < end:
        tools.spin_for(0.1)
    if 'picture' not in got:
        sys.exit('no picture from the camera')
    m = got['picture']
    grid = np.frombuffer(m.data, dtype=np.uint8).reshape(m.height, m.step)
    grid = grid[:, :m.width * 3].reshape(m.height, m.width, 3)
    if m.encoding == 'rgb8':
        grid = grid[:, :, ::-1]
    folder = os.path.expanduser('~/apnvi_logs/snaps')
    os.makedirs(folder, exist_ok=True)
    path = os.path.join(folder, time.strftime('snap_%H%M%S.jpg'))
    cv2.imwrite(path, np.ascontiguousarray(grid))
    print('saved', path)


def reset_master():
    import serial
    ports = glob.glob('/dev/serial/by-id/*CP2102_USB*0001*')
    if not ports:
        sys.exit('master not found')
    s = serial.Serial(ports[0], 115200, timeout=0.2)
    s.dtr = False
    s.rts = True
    time.sleep(0.2)
    s.rts = False
    buf = b''
    end = time.time() + 4
    while time.time() < end:
        buf += s.read(500)
    s.write(b'who\n')
    time.sleep(0.5)
    buf += s.read(500)
    s.close()
    text = buf.decode('utf-8', 'replace')
    print(text[-700:])
    print('OK: the master answered' if 'who master' in text else 'NOT OK: no "who master"')


def main():
    args = sys.argv[1:]
    if not args:
        sys.exit(__doc__)
    if args[0] == 'reset':
        reset_master()
        return
    rclpy.init()
    tools = Tools()
    try:
        if args[0] == 'health':
            health(tools)
        elif args[0] == 'where':
            end = time.time() + 5
            while 'position' not in tools.now and time.time() < end:
                tools.spin_for(0.1)
            print(show_position(tools))
        elif args[0] == 'drive':
            drive(tools, float(args[1]), 0.0, float(args[2]))
        elif args[0] == 'turn':
            drive(tools, 0.0, float(args[1]), float(args[2]))
        elif args[0] == 'goal':
            tools.send(tools.goal_pub, String(data=' '.join(args[1:])))
            tools.spin_for(1.5)
            for _, key, text in tools.events:
                if key == 'say':
                    print('says:', text)
            print('guide:', tools.now['guide'].data if 'guide' in tools.now else '?')
        elif args[0] == 'ask':
            kind = {'continue': 'follow_up', 'other': 'other'}[args[1]]
            tools.send(tools.request_pub, String(data=json.dumps({'kind': kind})))
            tools.spin_for(1.5)
            for _, key, text in tools.events:
                if key == 'say':
                    print('says:', text)
            print('guide:', tools.now['guide'].data if 'guide' in tools.now else '?')
        elif args[0] == 'map':
            map_command(tools, args[1:])
        elif args[0] == 'clear':
            from nav2_msgs.srv import ClearEntireCostmap
            for name in ('/global_costmap/clear_entirely_global_costmap',
                         '/local_costmap/clear_entirely_local_costmap'):
                client = tools.create_client(ClearEntireCostmap, name)
                if client.wait_for_service(timeout_sec=3.0):
                    client.call_async(ClearEntireCostmap.Request())
                    tools.spin_for(1.0)
                    print('cleared', name)
                else:
                    print('not there:', name)
        elif args[0] == 'snap':
            snap(tools)
        elif args[0] == 'say':
            say(tools, ' '.join(args[1:]))
        elif args[0] == 'pins':
            show_pins(tools)
        elif args[0] == 'pin':
            pin_command(tools, args[1:])
        elif args[0] == 'place':
            tools.send(tools.map_pub, String(data=json.dumps(
                {'cmd': 'save_place', 'name': ' '.join(args[1:])})))
            listen_for(tools, 3, keys=('reply',))
        elif args[0] == 'scan':
            scan(tools)
        elif args[0] == 'signal':
            signal(tools, args[1:])
        elif args[0] == 'ahead':
            ahead(tools, float(args[1]) if len(args) > 1 else 1.5)
        elif args[0] == 'record':
            record(tools)
        else:
            sys.exit(__doc__)
    finally:
        tools.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
