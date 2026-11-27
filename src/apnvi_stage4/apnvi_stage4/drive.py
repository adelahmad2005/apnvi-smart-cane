"""
The drive link's thinking, with no ROS and no real USB port, so it can be tested anywhere.

What it does:
  1. Turns the route follower's two numbers (forward speed in m/s, turn rate in rad/s) into
     two wheel speeds, caps them, and turns those into wheel power for the master ESP32
     (percent of the master's power cap, -100..100, + = forward).
  2. Reads the master's report lines (buttons, stops and their reasons, handle lost).
  3. Decides the line to send about 10 times a second: "d <left> <right>" and "vibe <0-100>".
     While the master reports any stop, or before it says "ready", it sends "d 0 0".

Wheel speeds (stage-design.md Stage 4, section 4, step 5):
  left  = forward - turn x half the wheel spacing
  right = forward + turn x half the wheel spacing
A positive turn rate makes the right wheel faster, so the device curves left.

No encoders: the master cannot measure wheel speed, so power is set from
speed_at_cap_mps, the real wheel speed at 100% power (measured 13 Nov, wheels in the air).

On the floor (15 Nov, ESB117) speed does not follow power in a straight line: below 45% the
motors only hum, and 45% / 62% / 78% gave about 0.06 / 0.14 / 0.31 m/s (100%: about 0.54, from
the line through the last two). So each wheel's speed is turned into power through that measured
table (power_table), in between by straight lines. A wheel asked for less than half the slowest
speed in the table is left at 0 (the device then pivots on it). Raw test power is never changed.
With an empty table it falls back to a straight line from speed_at_cap_mps, lifted to at least
min_power_pct.

Turning on the spot (23 Nov, ESB117, measured with the LiDAR, device on the floor, free mode off):
45% did not move, 50% 19 deg/s, 53% 26, 57% 28, 60% 33 (40 on a trip), 80% 90 to 107 deg/s. With
turn_gain the asked rate did not match: 0.4 rad/s (23 deg/s) gave 60%, and the follower's small
first turn asks gave 45%, which does not move, so it stood 10 s and then spun at 80%. Now every
turn on the spot (forward under PIVOT_FORWARD_MPS, any program) gets pivot_power_pct on each wheel,
about 22 deg/s, whatever rate was asked. Turns while driving forward are unchanged. 19:25 trip:
52% turned 31 to 37 deg/s with the walker following (the test had the handle held), so 50%.
"""

from dataclasses import dataclass, field
import json
import math

STOP_REASONS = ('red', 'green', 'handle', 'jetson', 'rear')

# (wheel speed m/s, power %) on the floor, 15 Nov, ESB117, both wheels, device and battery on it.
# Distance / time from the LiDAR, the 0.5 s smooth start taken off. 100% is from the line.
FLOOR_TABLE = ((0.057, 45), (0.142, 62), (0.31, 78), (0.54, 100))
PIVOT_FORWARD_MPS = 0.02        # slower than this forward counts as turning on the spot
PIVOT_MIN_RPS = 0.02            # a turn asked below this on the spot is no turn


@dataclass
class DriveSettings:
    """Numbers for the base, measured 13 Nov (Adel)."""

    wheel_spacing_m: float = 0.367    # measured 13 Nov (Adel): 325 mm inside + 42 mm tyre width
    wheel_diameter_m: float = 0.074   # measured 13 Nov (Adel): about 74 mm across
    speed_at_cap_mps: float = 0.64    # 13 Nov, in the air: 27.5 turns / 10 s x pi x 0.074 m
    max_speed_mps: float = 0.5        # top speed (Adel, 11 Nov; may change)
    max_turn_rps: float = 1.0         # top turn rate, radians a second
    min_power_pct: float = 45.0       # 15 Nov, on the floor: less only hums (35, 40 did not move)
    power_table: tuple = FLOOR_TABLE  # speed -> power on the floor; () = straight line instead
    # 15 Nov a spin on the floor gave 48 deg for 86 asked, so 1.8. Since de1ffcf turns on the
    # spot get pivot_power_pct, so the gain only scaled turns while driving: 23 Nov it doubled
    # every correction and the device zigzagged (85/0 then 0/85 each second). Now 1.0.
    turn_gain: float = 1.0
    pivot_power_pct: float = 50.0     # 23 Nov: on the spot (52% turned 31-37 deg/s); 0 = off
    # 23 Nov, EB2's floor: 50% turned only 11 deg/s (19 in ESB117) and with the walker behind it
    # stood still at the start of a turn. Each turn on the spot starts at pivot_kick_pct for
    # pivot_kick_s to get the wheels going, then pivot_power_pct (0 = no kick).
    pivot_kick_pct: float = 58.0
    pivot_kick_s: float = 0.5
    allow_reverse: bool = False       # the user walks behind: no backing up unless testing
    cmd_timeout_s: float = 0.3        # no new speed for this long: wheels to 0
    vibe_timeout_s: float = 0.5       # no new vibration strength for this long: vibration off


def clamp(value, low, high):
    """Keep value between low and high."""
    return max(low, min(high, value))


def capped_command(forward, turn, settings):
    """Cap the forward speed and turn rate (no reversing unless allowed)."""
    top = settings.max_speed_mps
    forward = clamp(forward, -top if settings.allow_reverse else 0.0, top)
    turn = clamp(turn, -settings.max_turn_rps, settings.max_turn_rps)
    return forward, turn


def wheel_speeds(forward, turn, spacing_m):
    """Two wheel speeds in m/s, (left, right), from forward speed and turn rate."""
    half = spacing_m / 2.0
    return forward - turn * half, forward + turn * half


def wheel_power(forward, turn, settings):
    """
    Wheel power for the master: (left %, right %), whole numbers in -100..100.

    The command is capped first. If a wheel would still need more than full power, both wheels
    are scaled down together, so the device keeps the same curve, only slower.
    """
    forward, turn = capped_command(forward, turn, settings)
    pivot = settings.pivot_power_pct
    if pivot > 0 and abs(forward) < PIVOT_FORWARD_MPS:
        if abs(turn) < PIVOT_MIN_RPS:
            return 0, 0
        power = int(round(clamp(pivot, 0, 100)))
        return (-power, power) if turn > 0 else (power, -power)
    left, right = wheel_speeds(forward, turn * settings.turn_gain, settings.wheel_spacing_m)
    table = settings.power_table
    full = max(table[-1][0] if table else settings.speed_at_cap_mps, 1e-6)
    biggest = max(abs(left), abs(right))
    if biggest > full:
        left, right = left * full / biggest, right * full / biggest
    return speed_to_power(left, settings), speed_to_power(right, settings)


def speed_to_power(speed, settings):
    """One wheel's speed in m/s to its power in % (sign kept), through the floor table."""
    size = abs(speed)
    table = settings.power_table
    if not table:
        full = max(settings.speed_at_cap_mps, 1e-6)
        return lift(int(round(clamp(100.0 * speed / full, -100, 100))), settings.min_power_pct)
    slowest, lowest_power = table[0]
    if size < slowest / 2.0:
        return 0
    if size <= slowest:
        power = lowest_power
    elif size >= table[-1][0]:
        power = table[-1][1]
    else:
        for (v1, p1), (v2, p2) in zip(table, table[1:]):
            if size <= v2:
                power = p1 + (p2 - p1) * (size - v1) / (v2 - v1)
                break
    return int(round(math.copysign(clamp(power, 0, 100), speed)))


def lift(power_pct, min_pct):
    """Raise a small wheel power to the lowest one that moves the device; drop a tiny one to 0."""
    size = abs(power_pct)
    if size == 0 or size >= min_pct:
        return power_pct
    if size < min_pct / 2.0:
        return 0
    return int(round(math.copysign(min_pct, power_pct)))


def wheel_rpm(power_pct, settings):
    """Give the expected wheel turns a minute at this power (to check by eye, in the air)."""
    speed = settings.speed_at_cap_mps * power_pct / 100.0
    return 60.0 * speed / (math.pi * max(settings.wheel_diameter_m, 1e-6))


def vibe_line(strength, motor=1):
    """
    Make the vibration command for the master: "vibe 70", or "vibe 70 2" for the second motor.

    An older master program reads only the strength and runs motor 1.
    """
    line = 'vibe %d' % int(round(clamp(strength, 0, 100)))
    return line + ' %d' % int(motor) if int(motor) in (2, 3) else line   # 3 = both (17 Nov)


def drive_line(left_pct, right_pct):
    """Make the wheel command for the master."""
    return 'd %d %d' % (left_pct, right_pct)


@dataclass
class MasterState:
    """What the master ESP32 has reported, read line by line."""

    is_master: bool = False           # answered "who master"
    ready: bool = False               # said "ready": driving allowed after a stop
    red: bool = False
    green: bool = False
    handle_up: bool = False           # the grip's radio is heard
    stops: set = field(default_factory=lambda: {'handle', 'jetson'})   # as at its start
    last_alive: str = ''

    def feed(self, line):
        """
        Take one line from the master and return a list of events.

        Each event is a tuple:
          ('red', bool), ('green', bool), ('handle', bool), ('stop', reason),
          ('clear', reason), ('ready',), ('master',), ('alive', line), ('other', line)
        """
        words = line.strip().split()
        if not words:
            return []
        text = ' '.join(words).lower()
        events = []
        if text == 'who master' or text.startswith('who master.'):
            self.is_master = True
            events.append(('master',))
        elif words[0].lower() == '===':
            self.reset()                      # it restarted
            events.append(('other', text))
        elif text in ('btn red down', 'btn red up'):
            self.red = text.endswith('down')
            events.append(('red', self.red))
        elif text in ('btn green on', 'btn green off'):
            self.green = text.endswith('on')
            events.append(('green', self.green))
        elif text in ('slave connected', 'slave lost'):
            self.handle_up = text.endswith('connected')
            events.append(('handle', self.handle_up))
        elif len(words) == 2 and words[0].lower() in ('stop', 'clear') \
                and words[1].lower() in STOP_REASONS:
            reason = words[1].lower()
            if words[0].lower() == 'stop':
                self.stops.add(reason)
                self.ready = False
                events.append(('stop', reason))
            else:
                self.stops.discard(reason)
                events.append(('clear', reason))
        elif text == 'ready':
            self.ready = True
            events.append(('ready',))
        elif words[0].lower() == 'alive':
            self.last_alive = text
            events.extend(self.sync_alive(text.split()))
            events.append(('alive', text))
        else:
            events.append(('other', text))
        return events

    def sync_alive(self, words):
        """
        Catch up from the once-a-second line "alive <s> s, slave <up|down> stops <list|none> ...".

        The stop and clear lines only come when something changes. If the drive link connects
        to a master that was already running, it missed them (for example "clear handle" from
        long ago), so the alive line is what tells it the master's real stops (13 Nov).
        """
        events = []
        if 'slave' in words[:-1]:
            up = words[words.index('slave') + 1] == 'up'
            if up != self.handle_up:
                self.handle_up = up
                events.append(('handle', up))
        if 'stops' in words[:-1]:
            listed = words[words.index('stops') + 1]
            now = set() if listed == 'none' else set(listed.split(','))
            now &= set(STOP_REASONS)
            for reason in sorted(now - self.stops):
                self.stops.add(reason)
                self.ready = False
                events.append(('stop', reason))
            for reason in sorted(self.stops - now):
                self.stops.discard(reason)
                events.append(('clear', reason))
        return events

    def reset(self):
        """Forget what the master said: it restarted, so all of it is old."""
        self.ready = False
        self.red = self.green = self.handle_up = False
        self.stops = {'handle', 'jetson'}

    def may_drive(self):
        """Say if the master would drive: it is the master, no stop, and ready."""
        return self.is_master and self.ready and not self.stops and not self.red \
            and not self.green


class DriveLink:
    """
    The drive link without ROS: give it speeds and report lines, ask it what to send.

    Times are seconds from any clock that only goes forward (the node uses time.monotonic).
    """

    def __init__(self, settings=None):
        self.settings = settings or DriveSettings()
        self.master = MasterState()
        self.forward = 0.0
        self.turn = 0.0
        self.speed_at = None              # when the last speed came in
        self.vibe = 0
        self.vibe_motor = 1               # which of the grip's two motors
        self.vibe_at = None
        self.test_power = (0, 0)          # raw wheel power for measuring (test_power mode)
        self.test_at = None
        self.last_power = (0, 0)
        self.pivot_since = None           # when the turn on the spot now going began (its kick)
        self.pivot_way = 0
        # After green (the emergency stop): wheels stay at 0 until green is off AND the speed
        # asked for has been 0 once, so a speed that is still being sent can't restart them
        # (Adel, 12 Nov).
        self.green_hold = False

    def set_speed(self, forward, turn, now):
        """Take a new forward speed (m/s) and turn rate (rad/s) from the route follower."""
        self.forward, self.turn, self.speed_at = float(forward), float(turn), now

    def set_test_power(self, left_pct, right_pct, now):
        """Take raw wheel power for measuring the speed at the cap. Still obeys every stop."""
        self.test_power = (int(clamp(left_pct, -100, 100)), int(clamp(right_pct, -100, 100)))
        self.test_at = now

    def set_vibration(self, strength, now):
        """Take a new vibration strength, 0 to 100."""
        self.vibe, self.vibe_at = int(clamp(strength, 0, 100)), now

    def set_vibration_motor(self, motor):
        """Pick the grip motor for the vibration: 1, 2, or 3 for both."""
        self.vibe_motor = int(motor) if int(motor) in (2, 3) else 1

    def on_line(self, line):
        """Take a line from the master. Returns its events (see MasterState.feed)."""
        return self.master.feed(line)

    def power(self, now):
        """Give the wheel power to send now, (left %, right %)."""
        green_now = self.master.green or 'green' in self.master.stops
        if green_now:
            self.green_hold = True
        wanted = self.wanted_power(now)
        if self.green_hold and not green_now and wanted == (0, 0):
            self.green_hold = False
        if not self.master.may_drive() or self.green_hold:
            return 0, 0
        return wanted

    def wanted_power(self, now):
        """Give the wheel power asked for (test power or speed), before any stop."""
        timeout = self.settings.cmd_timeout_s
        if self.test_at is not None and now - self.test_at <= timeout:
            return self.test_power
        if self.speed_at is None or now - self.speed_at > timeout:
            self.pivot_since = None
            return 0, 0
        power = wheel_power(self.forward, self.turn, self.settings)
        return self.kick(power, now)

    def kick(self, power, now):
        """Start each turn on the spot at pivot_kick_pct for pivot_kick_s (23 Nov, EB2)."""
        s = self.settings
        pivot = int(round(s.pivot_power_pct))
        if pivot <= 0 or s.pivot_kick_pct <= 0 or power not in ((-pivot, pivot), (pivot, -pivot)):
            self.pivot_since, self.pivot_way = None, 0
            return power
        way = 1 if power[1] > 0 else -1
        if self.pivot_since is None or way != self.pivot_way:
            self.pivot_since, self.pivot_way = now, way
        if now - self.pivot_since < s.pivot_kick_s:
            strong = int(round(clamp(s.pivot_kick_pct, 0, 100)))
            return (-strong, strong) if way > 0 else (strong, -strong)
        return power

    def vibration(self, now):
        """Give the vibration strength to send now (0 when the last one is too old)."""
        fresh = self.vibe_at is not None and now - self.vibe_at <= self.settings.vibe_timeout_s
        return self.vibe if fresh else 0

    def lines(self, now):
        """Give the lines to send to the master on this tick (about 10 times a second)."""
        self.last_power = self.power(now)
        return [drive_line(*self.last_power), vibe_line(self.vibration(now), self.vibe_motor)]

    def status(self, connected):
        """Make a JSON line for the 8081 page and for testing: what the drive is doing."""
        left, right = self.last_power
        return json.dumps({
            'connected': connected,
            'master': self.master.is_master,
            'ready': self.master.ready,
            'stops': sorted(self.master.stops),
            'red': self.master.red,
            'green': self.master.green,
            'green_hold': self.green_hold,
            'handle': self.master.handle_up,
            'left_pct': left,
            'right_pct': right,
            'left_rpm': round(wheel_rpm(left, self.settings), 1),
            'right_rpm': round(wheel_rpm(right, self.settings), 1),
        })
