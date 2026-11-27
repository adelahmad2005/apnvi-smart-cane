"""Stand-in for ROS 2 so the nodes can be imported and driven in tests here (not in the repo)."""
import importlib.abc, importlib.machinery, sys, types, time as _time


class Msg:
    """A message whose fields appear when used; numbers default to 0.0 via explicit classes."""
    _defaults = {}

    def __init__(self, *args, **kw):
        for k, v in type(self)._defaults.items():
            self.__dict__[k] = v() if callable(v) else v
        self.__dict__.update(kw)

    def __call__(self, *a, **k):
        return Msg()

    def __getattr__(self, name):
        if name.startswith('__'):
            raise AttributeError(name)
        v = Msg()
        self.__dict__[name] = v
        return v

    def __repr__(self):
        return '%s(%s)' % (type(self).__name__, self.__dict__)


def mk(name, **defaults):
    return type(name, (Msg,), {'_defaults': defaults})


Stamp = mk('Stamp', sec=0, nanosec=0)
Header = mk('Header', frame_id='', stamp=Stamp)
Vector3 = mk('Vector3', x=0.0, y=0.0, z=0.0)
Quaternion = mk('Quaternion', x=0.0, y=0.0, z=0.0, w=1.0)
Pose = mk('Pose', position=Vector3, orientation=Quaternion)
KNOWN = {
    'String': mk('String', data=''), 'Bool': mk('Bool', data=False),
    'UInt8': mk('UInt8', data=0), 'Float32': mk('Float32', data=0.0),
    'Empty': mk('Empty'),
    'Twist': mk('Twist', linear=Vector3, angular=Vector3),
    'Vector3': Vector3, 'Quaternion': Quaternion, 'Pose': Pose, 'Point': Vector3,
    'PoseStamped': mk('PoseStamped', header=Header, pose=Pose),
    'PoseWithCovariance': mk('PoseWithCovariance', pose=Pose),
    'TwistWithCovariance': mk('TwistWithCovariance', twist=lambda: KNOWN['Twist']()),
    'Odometry': mk('Odometry', header=Header, pose=lambda: KNOWN['PoseWithCovariance'](),
                   twist=lambda: KNOWN['TwistWithCovariance']()),
    'LaserScan': mk('LaserScan', header=Header, angle_min=0.0, angle_max=0.0,
                    angle_increment=0.0, range_min=0.0, range_max=0.0, ranges=list),
    'Image': mk('Image', header=Header, height=0, width=0, encoding='', step=0, data=b''),
    'CompressedImage': mk('CompressedImage', header=Header, format='', data=b''),
    'CameraInfo': mk('CameraInfo', header=Header, k=lambda: [0.0] * 9, width=0, height=0),
    'Path': mk('Path', header=Header, poses=list),
    'Header': Header,
}


class StubModule(types.ModuleType):
    def __getattr__(self, name):
        if name.startswith('__'):
            raise AttributeError(name)
        cls = KNOWN.get(name) or mk(name)
        setattr(self, name, cls)
        return cls


PREFIXES = ('std_msgs', 'geometry_msgs', 'sensor_msgs', 'nav_msgs', 'rcl_interfaces',
            'std_srvs', 'nav2_msgs', 'action_msgs', 'rtabmap_msgs', 'tf2_ros', 'cv_bridge',
            'builtin_interfaces', 'tf2_msgs', 'visualization_msgs')


class Finder(importlib.abc.MetaPathFinder, importlib.abc.Loader):
    def find_spec(self, name, path, target=None):
        if name.split('.')[0] in PREFIXES:
            return importlib.machinery.ModuleSpec(name, self, is_package=True)
        return None

    def create_module(self, spec):
        m = StubModule(spec.name)
        m.__path__ = []
        return m

    def exec_module(self, module):
        pass


sys.meta_path.append(Finder())

# ---- a fake node ---------------------------------------------------------------------------
PUBLISHED = []          # (topic, message) of everything published, in order
NODES = []


class Pub:
    def __init__(self, topic):
        self.topic, self.sent = topic, []

    def publish(self, msg):
        self.sent.append(msg)
        PUBLISHED.append((self.topic, msg))
        for node in NODES:
            for topic, cb in node.subs:
                if topic == self.topic:
                    cb(msg)

    def get_subscription_count(self):
        return 1


class Logger:
    def __init__(self):
        self.lines = []

    def _log(self, level, text, **kw):
        self.lines.append((level, text))

    def info(self, t, **kw): self._log('info', t)
    def warn(self, t, **kw): self._log('warn', t)
    warning = warn
    def error(self, t, **kw): self._log('error', t)
    def debug(self, t, **kw): self._log('debug', t)


class Param:
    def __init__(self, value):
        self.value = value


class Clock:
    def now(self):
        t = types.SimpleNamespace()
        t.nanoseconds = int(_time.time() * 1e9)
        t.to_msg = lambda: Stamp(sec=int(_time.time()), nanosec=0)
        return t


class Node:
    overrides = {}          # parameter values for the next node made

    def __init__(self, name='node', **kw):
        self.node_name = name
        self.params, self.subs, self.pubs, self.timers = {}, [], {}, []
        self._logger = Logger()
        self._overrides = dict(Node.overrides)
        NODES.append(self)

    def declare_parameter(self, name, value=None):
        self.params[name] = Param(self._overrides.get(name, value))
        return self.params[name]

    def get_parameter(self, name):
        return self.params[name]

    def create_publisher(self, typ, topic, qos=10):
        self.pubs[topic] = Pub(topic)
        return self.pubs[topic]

    def create_subscription(self, typ, topic, cb, qos=10):
        self.subs.append((topic, cb))
        return (topic, cb)

    def destroy_subscription(self, sub):
        if sub in self.subs:
            self.subs.remove(sub)

    def create_timer(self, period, cb):
        self.timers.append((period, cb))
        return types.SimpleNamespace(cancel=lambda: None)

    def create_client(self, typ, name):
        return types.SimpleNamespace(srv_name=name, wait_for_service=lambda timeout_sec=0: False)

    def get_logger(self):
        return self._logger

    def get_clock(self):
        return Clock()

    def destroy_node(self):
        if self in NODES:
            NODES.remove(self)
