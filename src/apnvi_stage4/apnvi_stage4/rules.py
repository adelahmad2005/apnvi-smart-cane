"""
The hazard gate's rule and the grip's vibration signals, with no ROS.

Plain Python, so both can be tested anywhere.

Hazard gate (Adel, 11 Nov): a Stage 1 hazard sets the forward speed to zero; the route stays.
The turn rate is passed on, so the route follower can still turn the device in place.
If Stage 1 goes quiet, forward speed is zero too: no warnings means no driving forward.

Vibration (Adel, 11 Nov): the handle vibrates ONLY when getting close to objects,
stronger the closer. Off when nothing is near. Nothing for turns or arrival.

Distances from the device's front (15 Nov): Stage 1 measures from the camera, which
sits about 70 cm behind the wheels (13 Nov), so the front of the device is already
camera_to_front_m ahead of it. Both rules take that off first: the stop is 0.5 m from the
device's front, and the vibration starts 1.5 m from it. Stage 1's own "stop" level still stops
too (it can only be later than this, never earlier).

Vibration meanings (Adel, 15 Nov night: "adding meaning to vibrations is very important").
Few signals, each with one meaning, told apart by rhythm and by which of the grip's
two motors runs. Silent means all is fine. vibration_signal() picks one for each tenth of a
second:
  careful      something ahead, 0.5 to 1.0 m from the front: two short pulses, pause, repeats
  stopped      forward is blocked: steady for 3 s, then one pulse a second
  going left / going right
               the device is turning the user that way (round something, or a turn of the
               route): quick ticks on that side's motor. If the two motors are not on two
               sides of the grip (two_sides false): one tick for left, three for right
  heard you    the talk button was pressed: one short tick
  arrived      three long pulses, once
  need help    it cannot find a way, or does not know where it is: a long pulse every 2 s
They run while the device is guiding, and also with no trip while the user pushes it
(free mode). Standing still with no trip: silent, except heard you.
An older master program ignores the motor number and plays everything on motor 1; the
rhythms alone still tell the signals apart.
"""

from dataclasses import dataclass

HAZARD_LEVELS = ('stop',)       # Stage 1 levels that stop forward driving
LEVEL_TIMEOUT_S = 0.5           # Stage 1 sends 10 times a second; quiet this long = unsafe
STOP_M = 0.5                    # stop when something is this close to the device's front
# camera to the base's front edge: about 0.83 m (Adel, 15 Nov: 0.82 to 0.85)
CAMERA_TO_FRONT_M = 0.83

# Vibration only when it matters (Adel, 15 Nov): two patterns while guiding, else silent.
CAREFUL_M = 1.0                 # (old rule, vibration_pattern) "careful" this close
CAREFUL_STRENGTH = 70
STOPPED_STRENGTH = 100
STOPPED_STEADY_S = 3.0          # "stopped" buzzes steadily this long, then one pulse a second
# Patterns in ticks of 0.1 s (the drive link sends 10 times a second):
CAREFUL_TICKS = (12, {0, 1, 4, 5})      # pulse 0.2 s, gap 0.2 s, pulse 0.2 s, quiet 0.6 s
STOPPED_LATER_TICKS = (10, {0, 1})      # after the steady part: a 0.2 s pulse each second

# 17 Nov redesign (Adel): the grip only gives the repeated "what is around me now" feeling;
# arrived, heard and help are spoken (they were also signals before). Three signals, none to learn:
#   near    soft pulses on both motors, faster as the thing gets closer (like a parking sensor)
#   stopped strong steady buzz on both motors (then a pulse a second, after STOPPED_STEADY_S)
#   left / right  something close coming in on that side: pulses on that side's motor only
NEAR_M = 1.5                    # "near" starts this far from the device's front
NEAR_STRENGTH = 55              # soft
NEAR_SLOW_TICKS = 10            # a pulse every 1.0 s at NEAR_M...
NEAR_FAST_TICKS = 2             # ...every 0.2 s at the stop distance
SIDE_TICKS = (4, {0, 1})        # left / right: 0.2 s on, 0.2 s off, on that side's motor
SIDE_STRENGTH = 80
TURNING_RPS = 0.3               # a turn rate asked for above this counts as "turning you"
TURNING_FOR_S = 0.3             # (kept for the recorder; turns are felt through the handle)
PUSHED_MPS = 0.15               # no trip: the user is pushing the device faster than this
HELP_AFTER_S = 5.0              # no route for this long while guiding (spoken, not a signal)
MAIN_MOTOR, OTHER_MOTOR = 1, 2
BOTH_MOTORS = 3                 # "vibe N 3": both grip motors at once (master, 17 Nov)

VIBE_START_M = 1.5              # (old rule) start vibrating this far from the device's front
VIBE_FULL_M = 0.5               # full strength at the stop distance
VIBE_MIN = 30                   # weakest vibration that is still felt (to tune)
VIBE_MAX = 100


def from_front(distance_m, camera_to_front_m=CAMERA_TO_FRONT_M):
    """
    Turn Stage 1's distance (from the camera) into a distance from the device's front.

    None or a negative number (nothing in range) stays None. Never below 0.
    """
    if distance_m is None or distance_m < 0:
        return None
    return max(0.0, distance_m - camera_to_front_m)


def gate(forward, turn, level, level_age_s, front_m=None, hazard_levels=HAZARD_LEVELS,
         timeout_s=LEVEL_TIMEOUT_S, stop_m=STOP_M):
    """
    Return (forward, turn, blocked) after the hazard gate.

    level: Stage 1's newest level (clear, notice, warning, stop) or None if none yet.
    level_age_s: seconds since that level came in.
    front_m: the nearest thing ahead, in metres from the device's front (from_front), or None.
    """
    blocked = level is None or level_age_s > timeout_s or level in hazard_levels \
        or (front_m is not None and front_m <= stop_m)
    if blocked and forward > 0:
        forward = 0.0
    return forward, turn, blocked


# 17 Nov (lab A3): at a sharp corner the camera saw a desk under 0.5 m and the device stood still.
# It flickered between stopped and careful at the edge of the stop distance, and the route's small
# turn, with the forward speed taken off, was too weak to move the wheels.
RELEASE_M = 0.15                # once stopped, drive on only when this much further than stop_m
RELEASE_S = 1.0                 # ...and only after it has looked clear this long
LIDAR_QUIET_S = 0.5             # no LiDAR sweep this long: the wheels stop (23 Nov)
# 23 Nov (Adel): turns on the spot were too quick for the walker to follow: 0.6 -> 0.4.
PIVOT_RPS = 0.4                 # stopped, the route bends: turn on the spot at least this fast
PIVOT_MIN_RPS = 0.05            # a turn asked for below this is "straight ahead": just wait


def hold_blocked(blocked_now, was_blocked, clear_since, now, release_s=RELEASE_S):
    """
    Keep the gate shut until it has looked clear for release_s. Return (blocked, clear_since).

    blocked_now: the gate's answer for this moment (with the release margin while stopped).
    clear_since: when it began to look clear while still held shut, or None.
    """
    if blocked_now:
        return True, None
    if not was_blocked:
        return False, None
    if clear_since is None:
        return True, now
    if now - clear_since >= release_s:
        return False, None
    return True, clear_since


SPIN_FORWARD_MPS = 0.05         # slower than this forward, the follower is turning on the spot
SPIN_MIN_RPS = 0.2              # a turn at least this fast picks the way
SPIN_ANY_RPS = 0.05             # slower than this is "no turn"
SPIN_KEEP_S = 2.0               # no turn the kept way this long: the next may pick its way again
SPIN_GIVE_S = 1.0               # the other way asked this long: the follower means it, follow it


class TurnLatch:
    """
    Keep the way of a turn on the spot until the device drives on (18 Nov, lab).

    With the goal straight behind, Nav2's follower chose left, then right, then left again
    (about 179 degrees either way) and the device wiggled on the spot for two minutes by the
    bin. The first way chosen is kept while it turns on the spot; driving forward, or no turn
    that way for SPIN_KEEP_S, lets the next turn choose again.

    18 Nov evening, two more lab cases:
    - on the way to ESB117 the follower asked left for 20 s with short blips right in between;
      the blips (under SPIN_MIN_RPS) went to the wheels and the device stood jerking.
    - by a desk the follower asked left once, then right (away from the desk) for 20 s; flipping
      every turn to the kept way drove it into the desk.
    So a turn the other way is not flipped: the device waits (turn 0). If the other way is still
    asked after SPIN_GIVE_S, the follower means it and that becomes the kept way.
    release() drops the way (the guide's own turn when getting unstuck is sent as asked).
    """

    def __init__(self):
        self.release()

    def release(self):
        self.way, self.at, self.other_since = 0, -1e9, None

    def apply(self, now, forward, turn):
        """Give the turn to send: as asked, or 0 for a short turn against the kept way."""
        if abs(forward) >= SPIN_FORWARD_MPS or now - self.at > SPIN_KEEP_S:
            self.release()
        if abs(forward) >= SPIN_FORWARD_MPS or abs(turn) < SPIN_ANY_RPS:
            return turn
        way = 1 if turn > 0 else -1
        if self.way == 0:
            if abs(turn) >= SPIN_MIN_RPS:
                self.way, self.at = way, now
            return turn
        if way == self.way:
            self.at, self.other_since = now, None
            return turn
        if self.other_since is None:
            self.other_since = now
        if now - self.other_since >= SPIN_GIVE_S:
            self.way, self.at, self.other_since = way, now, None
            return turn
        return 0.0


def lidar_quiet(now, scan_at, quiet_s=LIDAR_QUIET_S):
    """Say if the LiDAR has sent no sweep for quiet_s (or none yet: scan_at is None)."""
    return scan_at is None or now - scan_at > quiet_s


def pivot_when_blocked(forward_asked, turn, blocked, pivot_rps=PIVOT_RPS,
                       min_rps=PIVOT_MIN_RPS):
    """
    Give the turn rate to send when forward driving is blocked.

    If the route wants to go forward and bends (a corner), turn on the spot the way it bends, fast
    enough for the wheels to move, until the way ahead is clear. Straight ahead (a person in the
    way) or nothing asked: the turn is passed on unchanged and the device waits.
    """
    if not blocked or forward_asked <= 0 or abs(turn) < min_rps:
        return turn
    return turn if abs(turn) >= pivot_rps else (pivot_rps if turn > 0 else -pivot_rps)


def vibration_pattern(guiding, blocked, front_m, blocked_for_s, t_s, careful_m=CAREFUL_M):
    """
    Grip vibration strength 0..100 for this moment (called 10 times a second).

    guiding: a trip is on (not idle, not paused). Otherwise silent.
    blocked: the hazard gate stops forward driving now. "Stopped": steady for STOPPED_STEADY_S,
             then a 0.2 s pulse each second.
    front_m: the nearest thing in the path, metres from the device's front, or None.
             Within careful_m (and not blocked): "careful", two short pulses then quiet.
    blocked_for_s: how long it has been blocked. t_s: any clock that only goes forward.
    """
    if not guiding:
        return 0
    tick = int(round(t_s * 10))
    if blocked:
        if blocked_for_s < STOPPED_STEADY_S:
            return STOPPED_STRENGTH
        cycle, on = STOPPED_LATER_TICKS
        return STOPPED_STRENGTH if tick % cycle in on else 0
    if front_m is not None and front_m <= careful_m:
        cycle, on = CAREFUL_TICKS
        return CAREFUL_STRENGTH if tick % cycle in on else 0
    return 0


@dataclass
class VibeInput:
    """What the vibration node knows at this moment (see vibration_signal)."""

    guiding: bool = False           # a trip is on (not idle, not paused)
    moving: bool = False            # no trip, but the user pushes the device (free mode)
    blocked: bool = False           # forward is blocked (the hazard gate's rule)
    blocked_for_s: float = 0.0
    front_m: float = None           # nearest thing ahead, metres from the device's front
    turn_rps: float = 0.0           # turn rate the wheels are asked for now, + = left
    turning_for_s: float = 0.0      # how long it has been above TURNING_RPS that way
    need_help: bool = False         # no way to the goal, or the device is lost
    heard_ago_s: float = None       # seconds since the talk button went down (None: long ago)
    arrived_ago_s: float = None     # seconds since "You have arrived"
    side: str = None                # Stage 1: something coming in from 'left' or 'right' now


def _on(pattern, tick):
    cycle, ticks = pattern
    return tick % cycle in ticks


def near_period(front_m, near_m=NEAR_M, stop_m=STOP_M):
    """Ticks (0.1 s) from one "near" pulse to the next: slow far away, fast close up."""
    share = min(max((front_m - stop_m) / max(near_m - stop_m, 1e-6), 0.0), 1.0)
    return int(round(NEAR_FAST_TICKS + share * (NEAR_SLOW_TICKS - NEAR_FAST_TICKS)))


def vibration_signal(now, t_s, near_m=NEAR_M, two_sides=True, left_motor=MAIN_MOTOR):
    """
    Give (strength 0..100, motor 1, 2 or 3 = both, name) for this tenth of a second.

    now: a VibeInput. t_s: any clock that only goes forward. two_sides: the grip's two motors
    sit on its left and right (left_motor says which is on the left). name is '', near,
    stopped, left or right (for the recorder and the tests). Silent unless a trip is on or the
    user is pushing. Order: stopped, then something coming in from a side, then near.
    """
    tick = int(round(t_s * 10))
    if not (now.guiding or now.moving):
        return 0, BOTH_MOTORS, ''
    if now.blocked:
        if now.blocked_for_s < STOPPED_STEADY_S:
            return STOPPED_STRENGTH, BOTH_MOTORS, 'stopped'
        on = _on(STOPPED_LATER_TICKS, tick)
        return (STOPPED_STRENGTH if on else 0), BOTH_MOTORS, 'stopped'
    if now.side in ('left', 'right'):
        if two_sides:
            right_motor = OTHER_MOTOR if left_motor == MAIN_MOTOR else MAIN_MOTOR
            motor = left_motor if now.side == 'left' else right_motor
        else:
            motor = BOTH_MOTORS
        return (SIDE_STRENGTH if _on(SIDE_TICKS, tick) else 0), motor, now.side
    if now.front_m is not None and now.front_m <= near_m:
        on = tick % near_period(now.front_m, near_m) == 0
        return (NEAR_STRENGTH if on else 0), BOTH_MOTORS, 'near'
    return 0, BOTH_MOTORS, ''


SIGNAL_NAMES = ('near', 'stopped', 'left', 'right')


def signal_input(name, elapsed_s):
    """
    Make the VibeInput that plays one named signal, for trying the signals by hand.

    elapsed_s: seconds since the test began. "near" comes in from 1.5 m to the stop distance
    over 4 s, so the pulses can be felt getting faster. An unknown name gives silence.
    """
    if name == 'near':
        return VibeInput(guiding=True, front_m=max(NEAR_M - elapsed_s * 0.25, STOP_M))
    if name == 'stopped':
        return VibeInput(guiding=True, blocked=True, blocked_for_s=elapsed_s)
    if name in ('left', 'right'):
        return VibeInput(guiding=True, side=name)
    return VibeInput()


def vibration_strength(distance_m, start_m=VIBE_START_M, full_m=VIBE_FULL_M,
                       weakest=VIBE_MIN, strongest=VIBE_MAX):
    """
    Grip vibration strength 0..100 for Stage 1's distance to the nearest thing ahead.

    distance_m: metres, or a negative number / None when nothing is in range.
    0 when nothing is closer than start_m; from `weakest` at start_m up to `strongest` at
    full_m and closer.
    """
    if distance_m is None or distance_m < 0 or distance_m > start_m:
        return 0
    if distance_m <= full_m:
        return int(strongest)
    share = (start_m - distance_m) / (start_m - full_m)
    return int(round(weakest + share * (strongest - weakest)))
