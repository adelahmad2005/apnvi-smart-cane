# Watch /odom during a replay; when it has been quiet for 5 s, print where the lap ended.
import math, sys, time
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry

rclpy.init()
n = Node('odom_eval')
s = {'first': None, 'last': None, 'path': 0.0, 'prev': None, 'count': 0, 'nulls': 0, 'at': 0.0,
     'track': []}
def yaw(q):
    return math.degrees(math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)))
def on(m):
    s['at'] = time.time()
    p = m.pose.pose.position
    q = m.pose.pose.orientation
    if q.w == 0 and q.x == 0 and q.y == 0 and q.z == 0:
        s['nulls'] += 1
        return
    here = (p.x, p.y, yaw(q))
    s['count'] += 1
    if s['first'] is None:
        s['first'] = here
    if s['prev'] is not None:
        s['path'] += math.hypot(here[0] - s['prev'][0], here[1] - s['prev'][1])
    s['prev'] = s['last'] = here
    if s['count'] % 50 == 0:
        s['track'].append(here)
n.create_subscription(Odometry, '/odom', on, 50)
start = time.time()
while time.time() - start < 600:
    rclpy.spin_once(n, timeout_sec=0.2)
    if s['count'] and time.time() - s['at'] > 5:
        break
f, l = s['first'], s['last']
with open(sys.argv[1] if len(sys.argv) > 1 else '/dev/null', 'w') as out:
    for t in s['track']:
        out.write('%.2f %.2f %.1f\n' % t)
if f is None:
    print('no odometry at all')
else:
    err = math.hypot(l[0] - f[0], l[1] - f[1])
    turn = (l[2] - f[2] + 180) % 360 - 180
    print('end error %.2f m, heading error %.0f deg, path %.1f m, %d poses, %d lost' % (
        err, turn, s['path'], s['count'], s['nulls']), flush=True)
