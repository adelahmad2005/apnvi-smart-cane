"""
labeler_node: the map labels itself (Adel, 15 Nov).

As the device moves, this node looks at the camera picture now and then, asks Qwen (on the
campus PC) which signs, room names and landmarks are in it, places each one on the map from
the depth picture and the device's position (locate.py), and sends it to Stage 3's map manager
as a pin. The map manager counts the looks: a thing is shown and can be a goal once it was
seen twice. The pins are saved with the map and come back when it is loaded.

When it looks (labels.py): after the device moved 0.5 m or turned 20 degrees, and twice when
the user stops for 1 s (19 Nov: a sharp picture, and seen twice so the pin shows at once),
never more than once every 2 s, only while Stage 3 knows where the device is, and never while Stage 2 is busy
with the user's own question (the user comes first). Stage 1 is not touched: the camera
pictures are received only for the moment of a look (eye.py).

"Scan the room" (a voice command, or {"cmd": "scan_room"} on /stage3/command): it looks every
2 s for scan_s seconds without waiting for movement, while the user turns the device slowly
all the way round. Then it says how many new things it found and names up to five.

Listens: /stage3/position   PoseStamped  where the device is on the map
         /stage3/placement  String  placed / searching / lost ('' while a new map is drawn)
         /stage3/pins       String  the map's pins (for the scan's summary), latched
         /stage3/command    String  {"cmd": "scan_room"} starts a scan
         /stage2/status     String  listening / thinking / tracking / ready
         the colour and depth pictures and their camera infos, only during a look
Sends:   /stage3/command    String  {"cmd": "pin", "name", "kind", "x", "y", "from_x",
                                     "from_y", "sure": false} for each landmark
         /voice/say         String  the scan's sentences
         /stage2/labels     String  JSON after each look: seconds, how many were listed and
                                     how many were pinned (for the recorder and for testing)

Settings (--ros-args -p name:=value):
  pc             the campus PC's IP address (Qwen on port 11434)
  ollama_url     where Qwen runs, used when pc is empty; default http://localhost:11434
  model          default qwen3-vl:4b-instruct
  labels         false = never look by itself (a scan still works); default true
  moved_m (0.5), turned_deg (20), gap_s (2.0), scan_s (60), camera_back_m (0.70)
  color_topic, depth_topic, color_info_topic, depth_info_topic
"""

import json
import math
import threading
import time

from apnvi_stage1.hazard_node import image_to_depth_mm, NEWEST_PICTURE
from geometry_msgs.msg import PoseStamped
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String

from .eye import Eye
from .finder import ask_qwen, MODEL, OLLAMA_URL
from .labels import (LABEL_PROMPT, LOOK_GAP_S, LOOK_MOVED_M, read_labels, scan_summary,
                     should_look, still_look)
from .locate import CAMERA_BACK_M, lens_from_info, locate
from .stage2_node import (image_to_bgr, pin_command, PLACED_STATES, POSE_FRESH_S, urls_for_pc,
                          yaw_of)

SCAN_S = 60.0                   # how long "scan the room" keeps looking
READY_FOR_S = 1.0               # Stage 2 must have been ready this long before a look
PC_DOWN_S = 30.0                # after Qwen could not be reached, leave it alone this long
LOOK_TIMEOUT_S = 20.0           # give up on one look after this
LOOK_KEEP_S = 1.5               # keep receiving camera pictures this long around a look
SCAN_START = 'Scanning the room. Turn slowly all the way round'


class LabelerNode(Node):
    """Look, ask Qwen for landmarks, place them, pin them."""

    def __init__(self):
        super().__init__('labeler_node')
        self.declare_parameter('pc', '')
        self.declare_parameter('ollama_url', OLLAMA_URL)
        self.declare_parameter('model', MODEL)
        self.declare_parameter('labels', True)
        self.declare_parameter('moved_m', LOOK_MOVED_M)
        self.declare_parameter('turned_deg', 20.0)
        self.declare_parameter('gap_s', LOOK_GAP_S)
        self.declare_parameter('scan_s', SCAN_S)
        self.declare_parameter('camera_back_m', CAMERA_BACK_M)
        self.declare_parameter('color_topic', '/camera/camera/color/image_raw')
        self.declare_parameter('depth_topic', '/camera/camera/depth/image_rect_raw')
        self.declare_parameter('color_info_topic', '/camera/camera/color/camera_info')
        self.declare_parameter('depth_info_topic', '/camera/camera/depth/camera_info')
        self.url = self.get_parameter('ollama_url').value
        pc = str(self.get_parameter('pc').value).strip()
        if pc:
            self.url = urls_for_pc(pc)[0]
        self.model = self.get_parameter('model').value
        self.labels_on = bool(self.get_parameter('labels').value)
        self.moved_m = float(self.get_parameter('moved_m').value)
        self.turned_rad = math.radians(float(self.get_parameter('turned_deg').value))
        self.gap_s = float(self.get_parameter('gap_s').value)
        self.scan_s = float(self.get_parameter('scan_s').value)
        self.camera_back_m = float(self.get_parameter('camera_back_m').value)

        self.pose, self.pose_at, self.placement = None, -1e9, ''
        self.status, self.ready_since = 'ready', time.monotonic()
        self.last_pose, self.last_look = None, -1e9
        self.still_pose, self.still_since, self.looks_here = None, 0.0, 0   # the current stop
        self.looking = False
        self.pc_down_until = 0.0
        self.scan_until = 0.0                   # scanning while the clock is before this
        self.scan_before = []                   # the pin names when the scan started
        self.pin_names = []
        self.eye = Eye(self, {
            'colour': (Image, self.get_parameter('color_topic').value, NEWEST_PICTURE),
            'depth': (Image, self.get_parameter('depth_topic').value, NEWEST_PICTURE),
            'colour_info': (CameraInfo, self.get_parameter('color_info_topic').value,
                            NEWEST_PICTURE),
            'depth_info': (CameraInfo, self.get_parameter('depth_info_topic').value,
                           NEWEST_PICTURE)}, keep_s=LOOK_KEEP_S)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.command_pub = self.create_publisher(String, '/stage3/command', 10)
        self.say_pub = self.create_publisher(String, '/voice/say', 10)
        self.report_pub = self.create_publisher(String, '/stage2/labels', 10)
        self.create_subscription(PoseStamped, '/stage3/position', self.on_position, 10)
        self.create_subscription(String, '/stage3/placement', self.on_placement, latched)
        self.create_subscription(String, '/stage3/pins', self.on_pins, latched)
        self.create_subscription(String, '/stage3/command', self.on_command, 10)
        self.create_subscription(String, '/stage2/status', self.on_status, 10)
        self.create_timer(0.5, self.tick)
        self.get_logger().info('labeller ready: Qwen at %s, looking by itself: %s' % (
            self.url, 'yes' if self.labels_on else 'no (scan only)'))

    # ---- inputs ---------------------------------------------------------------------------

    def on_position(self, msg):
        position = msg.pose.position
        self.pose = (position.x, position.y, yaw_of(msg.pose.orientation))
        self.pose_at = time.monotonic()

    def on_placement(self, msg):
        self.placement = msg.data

    def on_pins(self, msg):
        try:
            self.pin_names = [pin['name'] for pin in json.loads(msg.data).get('pins', [])]
        except (ValueError, AttributeError, KeyError, TypeError):
            pass

    def on_status(self, msg):
        if msg.data == 'ready' and self.status != 'ready':
            self.ready_since = time.monotonic()
        self.status = msg.data

    def on_command(self, msg):
        try:
            command = json.loads(msg.data)
        except ValueError:
            return
        if isinstance(command, dict) and command.get('cmd') == 'scan_room':
            self.start_scan()

    def start_scan(self):
        if self.pose_now() is None:
            self.say_pub.publish(String(data="I don't know where I am on the map yet"))
            return
        self.scan_before = list(self.pin_names)
        self.scan_until = time.monotonic() + self.scan_s
        self.say_pub.publish(String(data=SCAN_START))
        self.get_logger().info('scan started (%.0f s)' % self.scan_s)

    def pose_now(self):
        """Give the device's (x, y, yaw) on the map, or None if Stage 3 does not know it."""
        if time.monotonic() - self.pose_at > POSE_FRESH_S or self.placement not in PLACED_STATES:
            return None
        return self.pose

    # ---- looking ----------------------------------------------------------------------------

    def tick(self):
        now = time.monotonic()
        scanning = now < self.scan_until
        if self.scan_until and not scanning and not self.looking:      # the scan just ended
            self.scan_until = 0.0
            threading.Timer(1.5, self.say_summary).start()     # let the last pins arrive
        if self.looking or now < self.pc_down_until or not (self.labels_on or scanning):
            return
        if self.status != 'ready' or now - self.ready_since < READY_FOR_S:
            return                              # the user's own question comes first
        pose = self.pose_now()
        stopped, self.still_pose, self.still_since, self.looks_here = still_look(
            pose, self.still_pose, self.still_since, self.looks_here, now)
        stopped = stopped and now - self.last_look >= self.gap_s
        if not stopped and not should_look(pose, self.last_pose, now, self.last_look, scanning,
                                           self.moved_m, self.turned_rad, self.gap_s):
            return
        self.eye.want()
        colour, depth = self.eye.get('colour', 0.3), self.eye.get('depth', 0.3)
        if colour is None or depth is None:
            return                              # the pictures come by the next tick
        if stopped:
            self.looks_here += 1
        self.looking = True
        self.last_pose, self.last_look = pose, now
        threading.Thread(target=self.look, args=(colour, depth, pose), daemon=True).start()

    def say_summary(self):
        self.say_pub.publish(String(data=scan_summary(self.scan_before, self.pin_names)))

    def look(self, colour, depth, pose):
        """One look: ask Qwen about this picture and pin what it lists (a worker thread)."""
        started = time.monotonic()
        report = {'listed': 0, 'pinned': 0, 'problem': ''}
        try:
            image = image_to_bgr(colour)
            depth_mm = image_to_depth_mm(depth)
            answer = ask_qwen(image, '', self.url, self.model, LOOK_TIMEOUT_S,
                              template=LABEL_PROMPT)
            height, width = image.shape[:2]
            labels = read_labels(answer, width, height)
            if labels is None:
                report['problem'] = 'could not understand the answer'
                return
            lenses = []
            for name in ('colour_info', 'depth_info'):
                info = self.eye.get(name, max_age_s=5.0)
                lenses.append(None if info is None else
                              lens_from_info(info.k, info.width, info.height))
            report['listed'] = len(labels)
            for label, kind, box in labels:
                located = locate(box, (width, height), depth_mm, pose, lenses[0], lenses[1],
                                 self.camera_back_m)
                if not located.found:
                    continue
                command = pin_command(label, located, 'thing' if kind == 'movable' else kind,
                                      sure=False)
                command['movable'] = kind == 'movable'     # kept for this session only
                self.command_pub.publish(String(data=json.dumps(command)))
                report['pinned'] += 1
        except OSError as problem:              # the PC or the network: leave it alone a while
            self.pc_down_until = time.monotonic() + PC_DOWN_S
            report['problem'] = 'cannot reach Qwen: %s' % problem
        except Exception as problem:            # never let one look kill the node
            report['problem'] = str(problem)
        finally:
            report['seconds'] = round(time.monotonic() - started, 2)
            self.report_pub.publish(String(data=json.dumps(report)))
            if report['problem']:
                self.get_logger().warn('look failed: %s' % report['problem'],
                                       throttle_duration_sec=10.0)
            else:
                self.get_logger().info('look: %d listed, %d pinned, %.1f s' % (
                    report['listed'], report['pinned'], report['seconds']))
            self.looking = False


def main(args=None):
    """Start the labeller and keep it running until Ctrl+C."""
    rclpy.init(args=args)
    node = LabelerNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
