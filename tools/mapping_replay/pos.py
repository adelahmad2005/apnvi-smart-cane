# Print where the device is now: raw movement tracking (odom) and the map's view (map).
# Used for the push test (15 Nov): read it at the start mark, push, read it again.
#   source /opt/ros/humble/setup.bash; source ~/apnvi_ws/install/setup.bash
#   python3 /tmp/pos.py | tee /tmp/push_start.txt
# Needs the device running (e.g. `bash start.sh 0.3 drive:=false` in the session tools).
import math, time, rclpy
from rclpy.time import Time
import tf2_ros
rclpy.init(); n = rclpy.create_node('pos_once')
buf = tf2_ros.Buffer(); tl = tf2_ros.TransformListener(buf, n)


def get(parent):
    t = buf.lookup_transform(parent, 'base_link', Time())
    q = t.transform.rotation
    return (t.transform.translation.x, t.transform.translation.y,
            math.degrees(math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))))


end = time.time() + 3; out = {}
while time.time() < end:
    rclpy.spin_once(n, timeout_sec=0.1)
    for p in ('odom', 'map'):
        try:
            out[p] = get(p)
        except Exception:
            pass
for p in ('odom', 'map'):
    print('%-4s x %.3f  y %.3f  heading %.1f' % ((p,) + out[p]) if p in out else p + ': not available')
