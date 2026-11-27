"""
Free mode test (16 Nov): free mode steering, the hazard gate and the vibration program together.

Made-up LiDAR and camera sweeps and a made-up push, with rosstub/ standing in for ROS. It checks
the rule that makes free mode safe: the device turns only while it is pushed, and what reaches
the wheels never has any forward speed. How to run it: see README.md in this folder.
"""
import json, time, math
import numpy as np
import rosstub_core as R
from std_msgs.msg import String, Bool, Float32
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import apnvi_stage4.free_mode_node as F
from apnvi_stage4.free_mode import quaternion_matrix
from apnvi_stage4.hazard_gate_node import HazardGateNode
from apnvi_stage4.vibration_node import VibrationNode
n = F.FreeModeNode(); g = HazardGateNode(); v = VibrationNode()
# frames as the launch sets them: LiDAR upside down facing back, camera forward, both 0.7 m behind
n.frames = {'laser': (quaternion_matrix(0, 0, 1, 0) @ quaternion_matrix(1, 0, 0, 0), (-0.7, 0, 0.77)),
            'camera_link': (np.eye(3), (-0.7, 0, 0.86))}
def lidar_empty():
    return LaserScan(header=R.Header(frame_id='laser'), angle_min=-math.pi, angle_increment=math.radians(1), ranges=[float('inf')] * 360)
def camera_box(dist, bearing_deg=0.0):
    # the camera sweep: -46..+46 deg, 0.5 deg beams; a box at `dist` from the lens
    r = [float('inf')] * 185
    mid = int((bearing_deg + 46) / 0.5)
    for i in range(mid - 6, mid + 7): r[i] = dist
    return LaserScan(header=R.Header(frame_id='camera_link'), angle_min=math.radians(-46), angle_increment=math.radians(0.5), ranges=r)
state = {'x': 0.0}
def odom(v):
    state['x'] += v * 0.1
    o = Odometry(); o.pose.pose.position.x = state['x']; n.on_odom(o); v_node_odom(o)
def v_node_odom(o): v.on_odom(o)
def run(seconds, speed, cam, idle=True):
    out = []
    gs = String(data=json.dumps({'state': 'idle' if idle else 'guiding'}))
    for _ in range(int(seconds * 10)):
        n.on_guide(gs); g.on_guide(gs); v.on_guide(gs)
        n.on_sweep('lidar', lidar_empty()); n.on_sweep('camera', cam)
        g.on_level(String(data='clear')); v.on_level(String(data='clear'))
        odom(speed); n.tick(); v.tick(); time.sleep(0.1)
        sent = g.pubs['/stage4/cmd_vel'].sent
        out.append((sent[-1].linear.x, round(sent[-1].angular.z, 2)) if sent else None)
    return out
box = camera_box(1.9, 6.0)      # 1.9 m from the lens = 1.07 m from the front, a little left
r = run(1.0, 0.0, box); print('nobody pushes  :', r[-1], n.steering.why); assert all(x in (None, (0.0, 0.0)) for x in r)
r = run(1.0, 0.4, box); print('pushed at a box:', r[-1], n.steering.why); assert r[-1] == (0.0, -0.4)
print('vibration signal:', v.pubs['/stage4/vibration_signal'].sent[-1].data, 'motor', v.pubs['/stage4/vibration_motor'].sent[-1].data)
assert v.pubs['/stage4/vibration_signal'].sent[-1].data == 'right'
r = run(0.3, 0.0, box); print('let go         :', r[-1], n.steering.why); assert r[-1] == (0.0, 0.0)
g.pubs['/stage4/cmd_vel'].sent.clear()
r = run(1.0, 0.4, box, idle=False); print('while guiding  :', r[-1]); assert all(x is None for x in r)
assert all(m.linear.x == 0.0 for m in n.pubs['/cmd_vel_free'].sent), 'free mode asked for forward speed'
# the gate throws away any forward speed on the free channel
g.on_guide(String(data=json.dumps({'state': 'idle'}))); t = Twist(); t.linear.x = 0.5; t.angular.z = 0.3; g.on_free(t)
assert g.pubs['/stage4/cmd_vel'].sent[-1].linear.x == 0.0 and g.pubs['/stage4/cmd_vel'].sent[-1].angular.z == 0.3
print('OK')
