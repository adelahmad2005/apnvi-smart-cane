"""
Whole-chain test (16 Nov): the programs of Stages 2 to 4 wired together on an ordinary computer.

No ROS, no camera, no LiDAR and no AI are needed: rosstub/ stands in for ROS (it passes each
message straight to whoever listens), the camera and depth pictures are made up, and a made-up
AI "sees" a door, a sign that says ESB117 and, later, a water bottle. It shows that the
programs hand the right messages to each other. It cannot show that the hardware behaves.
How to run it: see README.md in this folder. `docs/results.md` lists these 11 steps.
"""
import json, time, math, tempfile, threading
import numpy as np
import rosstub_core as R
from std_msgs.msg import String, Bool, Float32
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
maps = tempfile.mkdtemp()
R.Node.overrides = {'maps_dir': maps, 'sound_out': 'browser', 'voice': True}    # settings for every program
import apnvi_stage2.stage2_node as S2
import apnvi_stage2.labeler_node as LB
import apnvi_stage2.asking as A
from apnvi_stage2.finder import FindResult, RouteResult
from apnvi_stage3.map_manager_node import MapManagerNode
from apnvi_stage4.guide_node import GuideNode
from apnvi_stage4.vibration_node import VibrationNode
from apnvi_stage4.health_node import HealthNode

# the fake world: a door 3 m ahead-left, a sign ESB117 ahead-right, a bottle put down later
WORLD = {'door': (450, 420, 550, 580), 'ESB117': (700, 100, 780, 160)}
state = {'bottle': False, 'x': 0.0, 'y': 0.0, 'yaw': 0.0}
def fake_qwen(image, q, url, model, timeout, template=None, **k):
    return json.dumps([{'bbox_2d': list(WORLD['door']), 'label': 'door', 'kind': 'thing'},
                       {'bbox_2d': list(WORLD['ESB117']), 'label': 'ESB117', 'kind': 'sign'}])
LB.ask_qwen = fake_qwen
def fake_find(image, q, url=None, model=None, **k):
    if 'bottle' in q and state['bottle']:
        return FindResult(True, (600, 320, 680, 400), 'water bottle', 0.4, '', '[]', ((600, 320, 680, 400),))
    return FindResult(False, seconds=0.4, answer='[]')
S2.find = fake_find
A.answer_typed.__defaults__ = (fake_find,) + A.answer_typed.__defaults__[1:]
S2.route = lambda q, url=None, model=None: RouteResult('find', 0.1, '')

mm = MapManagerNode(); s2 = S2.Stage2Node(); lb = LB.LabelerNode(); g = GuideNode(); v = VibrationNode(); h = HealthNode()
said, sounds = [], []
mm.subs.append(('/voice/say', lambda m: said.append(m.data)))
mm.subs.append(('/voice/out', lambda m: sounds.append(json.loads(m.data))))
img = Image(height=720, width=1280, encoding='rgb8', step=3840, data=np.zeros((720, 1280, 3), np.uint8).tobytes())
depth = np.full((480, 848), 3000, np.uint16)
dmsg = Image(height=480, width=848, encoding='16UC1', is_bigendian=0, step=848 * 2, data=depth.tobytes())
pos_pub = R.Pub('/stage3/position'); lvl = R.Pub('/hazard/level'); cam = R.Pub('/sensor/status'); place = R.Pub('/stage3/placement')
def step(n=1, dt=0.05):
    for _ in range(n):
        p = PoseStamped(); p.pose.position.x = state['x']; p.pose.position.y = state['y']
        p.pose.orientation.z = math.sin(state['yaw'] / 2); p.pose.orientation.w = math.cos(state['yaw'] / 2)
        pos_pub.publish(p); mm.last_pose = (state['x'], state['y'], state['yaw'])
        lvl.publish(String(data='clear')); cam.publish(String(data='camera_ok'))
        s2.on_picture(img); s2.eye.take('depth', dmsg); lb.eye.take('colour', img); lb.eye.take('depth', dmsg)
        lb.tick(); h.tick(); v.tick(); g.on_tick(); time.sleep(dt)
def ask(q, wait=12):
    s2.on_typed(String(data=q)); step(wait)
def new(): 
    out = list(said); said.clear(); return out
place.publish(String(data=''))
print(' 1 start                :', (step(3), new())[1])
lb.ready_since -= 5; step(4); state['x'] = 0.6; lb.last_look -= 5; step(6)
pins = json.loads(mm.pubs['/stage3/pins'].sent[-1].data)
print(' 2 after two looks      :', [(p['name'], p['kind']) for p in pins['pins']])
assert {p['name'] for p in pins['pins']} == {'door', 'ESB117'}
ask('take me to the door')
print(' 3 take me to the door  :', new(), '| goal', g.guide.summary()['goal'])
assert g.guide.state == 'guiding'
goal = g.guide.goal; state['x'], state['y'] = goal.x - 0.3, goal.y; step(4)
print(' 4 at the goal          :', new(), '| grip:', v.pubs['/stage4/vibration_signal'].sent[-1].data)
assert g.guide.state == 'idle'
ask('take me to the water bottle')
print(' 5 bottle not in view   :', new())
state['bottle'] = True
ask('take me to the water bottle')
print(' 6 bottle in view       :', new())
assert g.guide.state == 'guiding'
ask('continue'); print(' 7 continue             :', new())
ask('save this place as my desk'); print(' 8 save this place      :', new())
ask('take me to my desk'); print(' 9 take me to my desk   :', new())
ask('scan the room', 4); print('10 scan the room        :', new())
ask('forget the door'); print('11 forget the door      :', new(), '| pins left:',
                              [p['name'] for p in json.loads(mm.pubs['/stage3/pins'].sent[-1].data)['pins']])
errors = [(n.node_name, t) for n in (mm, s2, lb, g, v, h) for level, t in n.get_logger().lines
          if level == 'error']
print('errors reported by the programs:', errors)
assert not errors
