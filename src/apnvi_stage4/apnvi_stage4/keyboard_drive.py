"""
keyboard_drive: send speeds from the keyboard, for testing (wheels off the ground first!).

  ros2 run apnvi_stage4 keyboard_drive                 speeds through the hazard gate
  ros2 run apnvi_stage4 keyboard_drive --ros-args -p power:=true
                         raw wheel power (for measuring the speed at the power cap); the
                         drive link must run with test_power:=true

Speed mode keys:  w faster forward   x slower / back   a turn left   d turn right
Power mode keys:  w +10% both wheels  x -10% both       a / d  shift power to one side
Both modes:       space or s = stop at once   q = stop and quit
Any other key also stops. Speeds are sent 10 times a second until changed.
The master's stop rules still apply (red, green, handle quiet, Jetson quiet, rear).

Sends: /cmd_vel_nav (Twist) in speed mode, /stage4/test_power (String "l r") in power mode.
"""

import select
import sys
import termios
import tty

from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

SPEED_STEP = 0.05               # m/s per key press
TURN_STEP = 0.1                 # rad/s per key press
POWER_STEP = 10                 # percent per key press
TOP_SPEED = 0.5
TOP_TURN = 1.0


def clamp(value, low, high):
    """Keep value between low and high."""
    return max(low, min(high, value))


def next_speeds(key, forward, turn):
    """Speed mode: the new (forward, turn) after one key."""
    if key == 'w':
        return clamp(round(forward + SPEED_STEP, 2), -TOP_SPEED, TOP_SPEED), turn
    if key == 'x':
        return clamp(round(forward - SPEED_STEP, 2), -TOP_SPEED, TOP_SPEED), turn
    if key == 'a':
        return forward, clamp(round(turn + TURN_STEP, 2), -TOP_TURN, TOP_TURN)
    if key == 'd':
        return forward, clamp(round(turn - TURN_STEP, 2), -TOP_TURN, TOP_TURN)
    return 0.0, 0.0                             # space, s, q and anything else: stop


def next_power(key, left, right):
    """Power mode: the new (left %, right %) after one key."""
    if key == 'w':
        return clamp(left + POWER_STEP, -100, 100), clamp(right + POWER_STEP, -100, 100)
    if key == 'x':
        return clamp(left - POWER_STEP, -100, 100), clamp(right - POWER_STEP, -100, 100)
    if key == 'a':
        return clamp(left - POWER_STEP, -100, 100), clamp(right + POWER_STEP, -100, 100)
    if key == 'd':
        return clamp(left + POWER_STEP, -100, 100), clamp(right - POWER_STEP, -100, 100)
    return 0, 0


class KeyboardDrive(Node):
    """Keep sending the chosen speeds 10 times a second."""

    def __init__(self):
        super().__init__('keyboard_drive')
        self.declare_parameter('power', False)
        self.power_mode = bool(self.get_parameter('power').value)
        self.a, self.b = 0, 0                   # (forward, turn) or (left %, right %)
        if self.power_mode:
            self.pub = self.create_publisher(String, '/stage4/test_power', 10)
        else:
            self.pub = self.create_publisher(Twist, '/cmd_vel_nav', 10)
        self.create_timer(0.1, self.send)

    def key(self, key):
        step = next_power if self.power_mode else next_speeds
        self.a, self.b = step(key, self.a, self.b)
        unit = ('left %d%%  right %d%%' if self.power_mode
                else 'forward %.2f m/s  turn %.2f rad/s')
        print('\r' + unit % (self.a, self.b) + '        ', end='', flush=True)

    def send(self):
        if self.power_mode:
            self.pub.publish(String(data='%d %d' % (self.a, self.b)))
        else:
            msg = Twist()
            msg.linear.x, msg.angular.z = float(self.a), float(self.b)
            self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = KeyboardDrive()
    print(__doc__)
    old = termios.tcgetattr(sys.stdin) if sys.stdin.isatty() else None
    try:
        if old is not None:
            tty.setcbreak(sys.stdin.fileno())
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.02)
            if select.select([sys.stdin], [], [], 0)[0]:
                key = sys.stdin.read(1).lower()
                node.key(key)
                if key == 'q':
                    break
    except KeyboardInterrupt:
        pass
    finally:
        node.a, node.b = 0, 0
        for _ in range(3):
            node.send()                         # leave the wheels stopped
        if old is not None:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        print()
