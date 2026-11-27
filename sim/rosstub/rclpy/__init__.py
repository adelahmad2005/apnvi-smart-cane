import rosstub_core  # noqa
from . import node, executors, qos, time, duration, action  # noqa
def init(args=None): pass
def shutdown(): pass
def ok(): return True
def spin(node): pass
def spin_once(node, timeout_sec=0): pass
