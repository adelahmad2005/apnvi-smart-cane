# Is the "obstacle" the floor? Print the camera sweep across the whole view, and estimate the camera's
# downward tilt from the depth picture's bottom rows (where the floor is).
import math, time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan, Image, CameraInfo
rclpy.init(); n = Node('floor_check'); got = {}
n.create_subscription(LaserScan, '/camera/obstacle_scan', lambda m: got.__setitem__('s', m), 10)
n.create_subscription(Image, '/camera/camera/depth/image_rect_raw', lambda m: got.__setitem__('d', m), qos_profile_sensor_data)
n.create_subscription(CameraInfo, '/camera/camera/depth/camera_info', lambda m: got.__setitem__('i', m), qos_profile_sensor_data)
t = time.time()
while time.time() - t < 4 and len(got) < 3:
    rclpy.spin_once(n, timeout_sec=0.2)
s = got.get('s')
if s:
    row = []
    for i in range(0, len(s.ranges), 10):
        r = s.ranges[i]
        row.append('%+.0f:%s' % (math.degrees(s.angle_min + i * s.angle_increment),
                                 'inf' if math.isinf(r) else ('nan' if math.isnan(r) else '%.2f' % r)))
    print('sweep across the view (deg: m from the lens):\n  ' + '  '.join(row))
d, info = got.get('d'), got.get('i')
if d and info:
    img = np.frombuffer(d.data, dtype=np.uint16).reshape(d.height, d.width).astype(float) / 1000.0
    fy, cy = info.k[4], info.k[5]
    cx, fx = info.k[2], info.k[0]
    tilts = []
    for v in range(int(d.height * 0.75), d.height, 4):          # bottom quarter: floor
        col = img[v, int(cx) - 40:int(cx) + 40]
        z = np.median(col[col > 0.2]) if (col > 0.2).any() else np.nan
        if not np.isfinite(z):
            continue
        down = (v - cy) / fy                    # slope below the lens axis
        # level camera at height h sees the floor at z = h / down; tilted down by a: h = z*(down+tan a)
        tilts.append((z, down))
    if tilts:
        z = np.array([a for a, _ in tilts]); dn = np.array([b for _, b in tilts])
        # fit h and tan(a): z*down + z*tan(a) = h  ->  least squares on h - z*tan(a) = z*down
        A = np.c_[np.ones_like(z), -z]
        h, tan_a = np.linalg.lstsq(A, z * dn, rcond=None)[0]
        print('floor seen in the bottom of the picture: camera height %.2f m, tilted down %.1f deg '
              '(from %d rows, floor %.2f to %.2f m ahead)' % (h, math.degrees(math.atan(tan_a)), len(z),
                                                              z.min(), z.max()))
