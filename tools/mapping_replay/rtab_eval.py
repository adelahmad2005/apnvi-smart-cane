# After a replay: where does the map (rtabmap_map frame) put the device at the end, vs the start?
import math, time
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from nav_msgs.msg import Odometry
import tf2_ros

rclpy.init()
n = Node('rtab_eval')
buf = tf2_ros.Buffer()
listener = tf2_ros.TransformListener(buf, n)   # keep it, or it stops listening
s = {'at': 0.0, 'count': 0, 'first': None}
def on(m):
    s['at'] = time.time()
    s['count'] += 1
n.create_subscription(Odometry, '/odom', on, 50)
def xyyaw(t):
    q = t.transform.rotation
    return (t.transform.translation.x, t.transform.translation.y,
            math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)))


def pose():
    # The map's correction (rtabmap_map -> odom) is stamped with today's clock, the movement
    # (odom -> base_link) with the recording's clock: look each up on its own and join them.
    try:
        a = xyyaw(buf.lookup_transform('rtabmap_map', 'odom', Time()))
        b = xyyaw(buf.lookup_transform('odom', 'base_link', Time()))
    except Exception:
        return None
    x = a[0] + b[0] * math.cos(a[2]) - b[1] * math.sin(a[2])
    y = a[1] + b[0] * math.sin(a[2]) + b[1] * math.cos(a[2])
    return x, y, math.degrees(a[2] + b[2])
start = time.time()
while time.time() - start < 600:
    rclpy.spin_once(n, timeout_sec=0.2)
    if s['first'] is None and s['count'] > 20:
        s['first'] = pose()
    if s['count'] and time.time() - s['at'] > 5:
        break
end_wait = time.time() + 6                 # let the last map update and its correction land
while time.time() < end_wait:
    rclpy.spin_once(n, timeout_sec=0.2)
f, l = s['first'] or (0.0, 0.0, 0.0), pose()
if l is None:
    print('no map pose')
else:
    err = math.hypot(l[0] - f[0], l[1] - f[1])
    turn = (l[2] - f[2] + 180) % 360 - 180
    print('MAP end error %.2f m, heading error %.0f deg (end %.2f %.2f %.0f)' % (
        err, turn, l[0], l[1], l[2]), flush=True)
