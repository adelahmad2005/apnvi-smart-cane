"""
drive_link_node: the only program that talks to the master ESP32 (the wheels and the grip).

Finds the master by its /dev/serial/by-id name (never ttyUSB0: the numbers swap), checks that
it answers "who master", then about 10 times a second sends the two wheel powers and the
vibration strength. The master stops by itself when it is not safe (see esp32_master.ino);
this program also sends 0 0 while the master reports any stop.

A master that comes up stuck: after a power-on the master sometimes talks at the
wrong speed and answers nothing readable. If "who" is not answered WHO_GIVE_UP_S after the
port was opened, this program restarts the master through the USB lines (the same pulse as
`session.py reset`), up to MAX_RESETS times, then keeps asking and says so in the status.

Listens: /stage4/cmd_vel     Twist   forward speed (linear.x, m/s) and turn rate (angular.z,
                                     rad/s), from the hazard gate
         /stage4/vibration   UInt8   grip vibration strength 0 to 100, from the vibration node
         /stage4/vibration_motor UInt8  which grip motor vibrates, 1 or 2
Sends:   /grip/red           Bool    red grip button held (the talk button), latched
         /grip/green         Bool    green grip button on (the emergency stop), latched
         /grip/connected     Bool    the grip's radio is heard (False = handle lost), latched
         /stage4/stops       String  JSON list of the master's stop reasons now, latched:
                                     red, green, handle, jetson, rear
         /stage4/master      String  every report line from the master (stop/clear reasons...)
         /stage4/drive_status String JSON twice a second: connected, ready, stops, wheel
                                     power and expected wheel turns a minute

Settings (--ros-args -p name:=value):
  port              the master's port; '' = look in /dev/serial/by-id for port_match
  port_match        part of the master's by-id name (default: the CP2102 with serial 0001)
  rate_hz           how often to send (default 10)
  wheel_spacing_m   default 0.367 (measured 13 Nov)
  wheel_diameter_m  default 0.074 (measured 13 Nov)
  speed_at_cap_mps  real wheel speed at 100% power (default 0.64, measured 13 Nov in the air)
  max_speed_mps     top speed (default 0.5)
  max_turn_rps      top turn rate (default 1.0)
  allow_reverse     allow backing up (default false: the user walks behind)
  rear_check        switch the master's "nobody behind" stop on (default false: not wired)
  dry_run           true = never open a port; print what would be sent (for testing)
  test_power        true = also take raw wheel power on /stage4/test_power (String
                    "<left> <right>", -100..100) for measuring the speed at the cap; the
                    master's stops still apply. Default false
"""

import glob
import json
import os
import threading
import time

from geometry_msgs.msg import Twist
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import Bool, String, UInt8

from .drive import DriveLink, DriveSettings

BAUD = 115200
BY_ID = '/dev/serial/by-id'
MASTER_MATCH = 'CP2102_USB_to_UART_Bridge_Controller_0001'
BOOT_WAIT_S = 2.0               # opening the port restarts the master: wait before talking
RETRY_S = 2.0                   # port gone: try again this often
WHO_GIVE_UP_S = 5.0             # no "who master" this long after opening: restart the master
MAX_RESETS = 3                  # restarts tried before giving up (it keeps asking "who")
RESET_PULSE_S = 0.2


def should_reset(now, opened_at, is_master, resets_done):
    """Say if the master should be restarted now: it is open, silent too long, tries left."""
    return (not is_master) and resets_done < MAX_RESETS and now - opened_at >= WHO_GIVE_UP_S


def find_port(match, folder=BY_ID):
    """Return the by-id path whose name contains match, or '' if none or more than one."""
    found = sorted(p for p in glob.glob(os.path.join(folder, '*')) if match in os.path.basename(p))
    return found[0] if len(found) == 1 else ''


class SerialLink:
    """
    The USB line to the master: open, read lines in a thread, write lines.

    opener(path) returns an object with readline(), write(bytes) and close(), like
    serial.Serial; the tests pass a fake one.
    """

    def __init__(self, opener, on_line, log=print):
        self.opener = opener
        self.on_line = on_line
        self.log = log
        self.port = None
        self.lock = threading.Lock()
        self.reader = None

    def open(self, path):
        """Open the port and start reading. Returns True if it opened."""
        try:
            port = self.opener(path)
        except (OSError, ValueError) as problem:
            self.log('cannot open %s: %s' % (path, problem))
            return False
        with self.lock:
            self.port = port
        self.reader = threading.Thread(target=self.read_loop, args=(port,), daemon=True)
        self.reader.start()
        return True

    def is_open(self):
        """Say if the port is open."""
        return self.port is not None

    def read_loop(self, port):
        """Hand every line to on_line until the port fails or closes."""
        while self.port is port:
            try:
                raw = port.readline()
            except (OSError, ValueError, TypeError) as problem:
                self.log('USB read failed: %s' % problem)
                self.close()
                return
            if raw:
                self.on_line(raw.decode('ascii', 'replace').strip())

    def write(self, lines):
        """Send lines. Returns False (and closes) if the port failed."""
        with self.lock:
            port = self.port
        if port is None:
            return False
        try:
            port.write(''.join(line + '\n' for line in lines).encode('ascii'))
            return True
        except (OSError, ValueError, TypeError) as problem:
            self.log('USB write failed: %s' % problem)
            self.close()
            return False

    def reset_board(self, pulse_s=RESET_PULSE_S, sleep=time.sleep):
        """
        Restart the ESP32 through the USB lines (RTS pulls its EN pin low). True if it could.

        The same pulse as `session.py reset` (13 Nov): DTR off, RTS on for a moment, RTS off.
        """
        with self.lock:
            port = self.port
        if port is None:
            return False
        try:
            port.dtr = False
            port.rts = True
            sleep(pulse_s)
            port.rts = False
            return True
        except (OSError, ValueError, TypeError, AttributeError) as problem:
            self.log('could not restart the master over USB: %s' % problem)
            return False

    def close(self):
        """Close the port (safe to call twice)."""
        with self.lock:
            port, self.port = self.port, None
        if port is not None:
            try:
                port.close()
            except (OSError, ValueError, TypeError):
                pass


def open_serial(path):
    """
    Open the real port (pyserial), for this program only.

    exclusive=True locks the port, so a second drive link (one left over from an old launch)
    cannot open it too. It fails, says so, and keeps trying every few seconds.
    """
    import serial
    return serial.Serial(path, BAUD, timeout=0.2, write_timeout=0.2, exclusive=True)


class DriveLinkNode(Node):
    """Send wheel power and vibration to the master; publish what it reports."""

    def __init__(self, opener=open_serial, **kwargs):
        super().__init__('drive_link_node', **kwargs)
        defaults = DriveSettings()
        self.declare_parameter('port', '')
        self.declare_parameter('port_match', MASTER_MATCH)
        self.declare_parameter('rate_hz', 10.0)
        for name in ('wheel_spacing_m', 'wheel_diameter_m', 'speed_at_cap_mps',
                     'max_speed_mps', 'max_turn_rps', 'min_power_pct', 'turn_gain',
                     'pivot_power_pct', 'pivot_kick_pct', 'pivot_kick_s'):
            self.declare_parameter(name, float(getattr(defaults, name)))
        self.declare_parameter('allow_reverse', False)
        self.declare_parameter('rear_check', False)
        self.declare_parameter('dry_run', False)
        self.declare_parameter('test_power', False)
        settings = DriveSettings(
            **{name: float(self.get_parameter(name).value) for name in (
                'wheel_spacing_m', 'wheel_diameter_m', 'speed_at_cap_mps', 'max_speed_mps',
                'max_turn_rps', 'min_power_pct', 'turn_gain', 'pivot_power_pct',
                'pivot_kick_pct', 'pivot_kick_s')},
            allow_reverse=bool(self.get_parameter('allow_reverse').value))
        self.link = DriveLink(settings)
        self.rear_check = bool(self.get_parameter('rear_check').value)
        self.dry_run = bool(self.get_parameter('dry_run').value)
        self.lock = threading.Lock()
        self.serial = SerialLink(opener, self.on_line, self.get_logger().warn)
        self.opened_at = 0.0
        self.next_try = 0.0
        self.last_who = 0.0
        self.was_open = False
        self.resets_done = 0            # restarts of a stuck master since it last answered

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.red_pub = self.create_publisher(Bool, '/grip/red', latched)
        self.green_pub = self.create_publisher(Bool, '/grip/green', latched)
        self.handle_pub = self.create_publisher(Bool, '/grip/connected', latched)
        self.stops_pub = self.create_publisher(String, '/stage4/stops', latched)
        self.report_pub = self.create_publisher(String, '/stage4/master', 20)
        self.status_pub = self.create_publisher(String, '/stage4/drive_status', 5)
        self.create_subscription(Twist, '/stage4/cmd_vel', self.on_cmd_vel, 10)
        self.create_subscription(UInt8, '/stage4/vibration', self.on_vibration, 10)
        self.create_subscription(UInt8, '/stage4/vibration_motor', self.on_vibration_motor, 10)
        if bool(self.get_parameter('test_power').value):
            self.create_subscription(String, '/stage4/test_power', self.on_test_power, 10)
            self.get_logger().warn('test_power is on: raw wheel power accepted')
        self.create_timer(1.0 / float(self.get_parameter('rate_hz').value), self.tick)
        self.create_timer(0.5, self.publish_status)
        self.publish_stops()
        self.get_logger().info(
            'drive link: spacing %.2f m, wheel %.2f m, %.2f m/s at the power cap, top %.2f m/s'
            ' (speed at the cap measured in the air; check it on the floor)%s' % (
                settings.wheel_spacing_m, settings.wheel_diameter_m, settings.speed_at_cap_mps,
                settings.max_speed_mps, ', DRY RUN: no port opened' if self.dry_run else ''))

    # ---- inputs -------------------------------------------------------------------------

    def on_cmd_vel(self, msg):
        with self.lock:
            self.link.set_speed(msg.linear.x, msg.angular.z, time.monotonic())

    def on_test_power(self, msg):
        try:
            left, right = (float(v) for v in msg.data.split())
        except ValueError:
            return
        with self.lock:
            self.link.set_test_power(left, right, time.monotonic())

    def on_vibration(self, msg):
        with self.lock:
            self.link.set_vibration(msg.data, time.monotonic())

    def on_vibration_motor(self, msg):
        with self.lock:
            self.link.set_vibration_motor(msg.data)

    def on_line(self, line):
        """Handle a report line from the master (called from the reading thread)."""
        with self.lock:
            events = self.link.on_line(line)
        for event in events:
            kind = event[0]
            if kind == 'alive':
                continue                        # once a second: only in the status
            self.report_pub.publish(String(data=line))
            if kind == 'red':
                self.red_pub.publish(Bool(data=event[1]))
            elif kind == 'green':
                self.green_pub.publish(Bool(data=event[1]))
            elif kind == 'handle':
                self.handle_pub.publish(Bool(data=event[1]))
            elif kind in ('stop', 'clear', 'ready'):
                self.publish_stops()
                if kind == 'stop':
                    self.get_logger().info('master stopped the wheels: %s' % event[1])
            elif kind == 'master':
                self.resets_done = 0
                self.get_logger().info('the master answered')
                self.serial.write(['rear on' if self.rear_check else 'rear off'])

    # ---- the 10-a-second loop -------------------------------------------------------------

    def tick(self):
        now = time.monotonic()
        with self.lock:
            lines = self.link.lines(now)
        if self.dry_run:
            self.get_logger().info(' | '.join(lines), throttle_duration_sec=1.0)
            return
        if not self.serial.is_open():
            if self.was_open:                   # the reading thread found it broken
                self.was_open = False
                self.lost()
            self.connect(now)
            return
        if now - self.opened_at < BOOT_WAIT_S:
            return                              # the master is restarting
        if not self.link.master.is_master:
            if should_reset(now, self.opened_at, False, self.resets_done):
                self.resets_done += 1           # stuck after power-on: restart it
                self.get_logger().warn('the master does not answer: restarting it over USB '
                                       '(try %d of %d)' % (self.resets_done, MAX_RESETS))
                self.serial.reset_board()
                # Open the port again from fresh (16 Nov): after a USB replug ModemManager probes
                # the board and changes the port's settings under us; we then hear nothing.
                self.serial.close()
                self.was_open = False           # on purpose: not a lost link
                self.next_try = now + BOOT_WAIT_S
                if self.resets_done == MAX_RESETS:
                    self.get_logger().error('the master still does not answer after %d '
                                            'restarts: check its USB and power' % MAX_RESETS)
                return
            if now - self.last_who >= 1.0:      # never drive a board that is not the master
                self.last_who = now
                self.serial.write(['who'])
            return
        if not self.serial.write(lines):
            self.was_open = False
            self.lost()

    def connect(self, now):
        if now < self.next_try:
            return
        self.next_try = now + RETRY_S
        path = self.get_parameter('port').value or find_port(
            self.get_parameter('port_match').value)
        if not path:
            self.get_logger().warn('master ESP32 not found in %s (match "%s")' % (
                BY_ID, self.get_parameter('port_match').value), throttle_duration_sec=10.0)
            return
        with self.lock:
            self.link.master.reset()
            self.link.master.is_master = False
        if self.serial.open(path):
            self.was_open = True
            self.opened_at = now
            self.get_logger().info('opened %s; waiting for the master to start' % path)
            self.publish_stops()

    def lost(self):
        self.get_logger().warn('USB link to the master lost; the master stops by itself')
        with self.lock:
            self.link.master.reset()
            self.link.master.is_master = False
        self.handle_pub.publish(Bool(data=False))
        self.publish_stops()

    # ---- outputs ----------------------------------------------------------------------------

    def publish_stops(self):
        with self.lock:
            stops = sorted(self.link.master.stops)
            if not self.link.master.ready and not stops:
                stops = ['waiting']             # stops are gone, waiting for "ready"
        self.stops_pub.publish(String(data=json.dumps(stops)))

    def publish_status(self):
        with self.lock:
            status = json.loads(self.link.status(self.serial.is_open()))
        status['resets'] = self.resets_done     # restarts of a stuck master
        self.status_pub.publish(String(data=json.dumps(status)))

    def destroy_node(self):
        if self.serial.is_open():
            self.serial.write(['d 0 0', 'vibe 0'])
            self.serial.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = DriveLinkNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
