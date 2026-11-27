"""
Turn Stage 3's map into a small picture for the 8081 page, only while someone watches.

Listens: /map (the map from RTAB-Map) and the frame map -> base_link (where the device is),
         /stage3/pins (the map's named places, signs and things, from the map manager),
         /stage3/map_zoom (from the page, 17 Nov): JSON {"zoom": 1 to 8, "x", "y"},
                          draw that much closer in round the map point (x, y); zoom 1 with no
                          x, y = the whole map. The page's drag moves (x, y), at any zoom.
                          Goes back to the whole map when nobody watches.
Sends:   /stage3/map_image  the picture, as a JPEG (CompressedImage), when the map or the
                            device moved, at most `rate` a second, and once a second at rest
         /stage3/map_view   the box the picture shows, as JSON {"x_left","y_top","span","size"}
                            in metres, so a click on the picture can be turned into a map point;
                            also "zoom" (how much closer than the whole map) and "pins" (each
                            dot: name, kind, col, row in pixels, x, y in metres), so the page
                            can say what a tapped dot is.
The Stage 2 page listens to /stage3/map_image only while a browser is open. Nothing is drawn
or encoded while nobody listens. The node stays subscribed to /map: RTAB-Map then keeps the
map up to date (a 0.3 ms job), so the first picture after someone opens the page is current.

Settings (--ros-args -p name:=value):
  rate   pictures per second at most, default 10 (the LiDAR sweeps 10 a second)
"""

import json
import math
import time

import cv2
import numpy as np
from nav_msgs.msg import OccupancyGrid
import rclpy
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener

from .map_picture import draw_map, MAX_ZOOM, SIZE, view_box

JPEG_QUALITY = 70
MAP_FRAME = 'map'
DEVICE_FRAME = 'base_link'
POSE_MAX_AGE_S = 1.0            # a position older than this is not drawn
HEARTBEAT_S = 1.0               # at rest, still send a picture this often (so the page knows)


def yaw_of(q):
    """Give the way the device faces (radians, 0 = along x, anticlockwise) from a quaternion."""
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def read_zoom(text):
    """Turn the page's zoom request into (zoom, centre or None); (1.0, None) if it is bad."""
    try:
        asked = json.loads(text)
        zoom = min(max(float(asked.get('zoom', 1)), 1.0), MAX_ZOOM)
        if not math.isfinite(zoom):
            return 1.0, None
        centre = (float(asked['x']), float(asked['y'])) if 'x' in asked else None
        if centre is not None and not all(math.isfinite(v) for v in centre):
            centre = None
        return zoom, centre
    except (ValueError, TypeError, AttributeError, KeyError):
        return 1.0, None


def view_words(box, zoom, shown):
    """Make the /stage3/map_view JSON: the box shown, the zoom, and where each pin's dot is."""
    return json.dumps({'x_left': box[0], 'y_top': box[1], 'span': box[2], 'size': SIZE,
                       'zoom': round(zoom, 2), 'pins': shown})


class MapViewNode(Node):
    """Keep the newest map, and draw it with the device on it for whoever is listening."""

    def __init__(self):
        super().__init__('map_view_node')
        self.declare_parameter('rate', 10.0)
        self.grid, self.resolution, self.origin = None, 0.05, (0.0, 0.0)
        self.map_number = 0                     # goes up with every new map
        self.sent = None                        # what the last picture showed: (map, pose)
        self.sent_time, self.watchers = 0.0, 0
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(OccupancyGrid, '/map', self.on_map, latched)
        self.pins = []
        self.create_subscription(String, '/stage3/pins', self.on_pins, latched)
        self.zoom, self.centre = 1.0, None      # what the page asked for: 1 = the whole map
        self.create_subscription(String, '/stage3/map_zoom', self.on_zoom, 10)
        self.image_pub = self.create_publisher(CompressedImage, '/stage3/map_image', 1)
        self.view_pub = self.create_publisher(String, '/stage3/map_view', 1)
        self.create_timer(1 / float(self.get_parameter('rate').value), self.draw)

    def on_map(self, msg):
        """Keep the newest map as numbers: rows of cells, row 0 at the lowest y."""
        self.grid = np.array(msg.data, dtype=np.int8).reshape(msg.info.height, msg.info.width)
        self.resolution = msg.info.resolution
        self.origin = (msg.info.origin.position.x, msg.info.origin.position.y)
        self.map_number += 1

    def on_pins(self, msg):
        """Keep the newest pins; a change redraws the picture."""
        try:
            self.pins = list(json.loads(msg.data).get('pins', []))
        except (ValueError, AttributeError):
            return
        self.map_number += 1

    def on_zoom(self, msg):
        """Draw closer in (or back out) as the page asks; the next picture shows it."""
        self.zoom, self.centre = read_zoom(msg.data)
        self.map_number += 1

    def pose(self):
        """Give the device's (x, y, yaw) on the map, or None if it is not known (yet)."""
        try:
            found = self.tf_buffer.lookup_transform(MAP_FRAME, DEVICE_FRAME, rclpy.time.Time())
        except TransformException:
            return None
        age = self.get_clock().now() - rclpy.time.Time.from_msg(found.header.stamp)
        if age > Duration(seconds=POSE_MAX_AGE_S):
            return None
        move, turn = found.transform.translation, found.transform.rotation
        return move.x, move.y, yaw_of(turn)

    def draw(self):
        if self.image_pub.get_subscription_count() == 0 or self.grid is None:
            self.watchers = 0
            self.zoom, self.centre = 1.0, None  # the next viewer starts with the whole map
            return                              # nobody watching, or no map yet: no work
        pose = self.pose()
        shown = (self.map_number, None if pose is None else
                 (round(pose[0], 3), round(pose[1], 3), round(pose[2], 2)))
        now = time.monotonic()
        news = self.watchers == 0 or shown != self.sent       # a new viewer, or something moved
        self.watchers = self.image_pub.get_subscription_count()
        if not news and now - self.sent_time < HEARTBEAT_S:
            return
        self.sent, self.sent_time = shown, now
        picture, box, shown = draw_map(self.grid, self.resolution, self.origin, pose,
                                       self.pins, self.zoom, self.centre)
        zoom = 1.0
        if self.zoom > 1.0:                     # how much closer it really is (there is a limit)
            zoom = view_box(self.grid, self.resolution, self.origin, pose)[2] / box[2]
        ok, encoded = cv2.imencode('.jpg', picture, [cv2.IMWRITE_JPEG_QUALITY, JPEG_QUALITY])
        if not ok:
            return
        image = CompressedImage()
        image.header.stamp = self.get_clock().now().to_msg()
        image.header.frame_id = MAP_FRAME
        image.format = 'jpeg'
        image.data = encoded.tobytes()
        self.image_pub.publish(image)
        self.view_pub.publish(String(data=view_words(box, zoom, shown)))


def main(args=None):
    """Start the map picture node, until Ctrl+C."""
    rclpy.init(args=args)
    node = MapViewNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
