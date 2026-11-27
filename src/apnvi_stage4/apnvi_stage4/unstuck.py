"""
Getting unstuck on a guided trip, with no ROS, so it can be tested anywhere.

Adel, 17 Nov (lab A2, A3): at desk corners and facing desks the camera stops the device and it
stands still. The LiDAR (77 cm high) does not see desks, so the route keeps leading into them.
Now, when a trip has been stopped for wait_s and nothing the LiDAR sees is straight ahead, the
device gets itself free:
  1. It says which way it will turn ("Finding a way round. Turning left") and waits say_s.
  2. It turns on the spot that way at turn_rps (the grip gives going left / going right) until
     the camera and the LiDAR both see the way ahead clear for clear_m.
  3. It stands until the hazard gate lets it go, then drives straight about drive_m (through
     the hazard gate as always: the camera stops it).
  4. The route takes over again (the guide re-plans at once).
It never drives backwards. It stops at once and the try ends when the trip is paused (red,
green), Stage 1 goes quiet, the master reports any stop, or the LiDAR or the movement
tracking goes old.

Which side: the side that is clear soonest, from free mode's clear_heading (LiDAR and camera
points, 10 to 60 degrees). Both sides alike, or neither clear: the side with more room at 90
degrees; no difference: toward the goal.
A turn that finds nothing clear within max_turn_rad: the next try turns the other way.

23 Nov: no route (the planner fails) is a hold-up too, as when the camera stops the device.

When it does not try:
  - something the LiDAR sees straight ahead within person_m: most likely a person, who will
    leave (or a chair back). For the first person_wait_s of a hold-up it waits (23 Nov: then it
    tries anyway, a wall never leaves) and talks as before: "Something is in the way. Waiting",
    then after still_s "The way is still blocked. Turn me a little to the left or right".
  - max_tries tries already in this hold-up: it says "The way is still blocked. Turn me a
    little..." and the user turns it. A hold-up ends after free_s without being stopped.

Unstuck.step() is called 10 times a second. It gives (command, actions):
  command   (forward, turn) to send to the wheels now, or None: the route follower drives
  actions   ('hold',)    stop the route follower and re-planning: this program drives now
            ('resume',)  hand back to the route: re-plan now
            ('say', words)
"""

from dataclasses import dataclass
import math

from .free_mode import clear_heading, FreeSettings, lane_nearest
from .guide import BLOCKED, BLOCKED_SAY_S, STILL_BLOCKED, STILL_BLOCKED_S

TURNING = 'Finding a way round. Turning %s'


@dataclass
class UnstuckSettings:
    """Numbers for getting unstuck (all to tune on the floor)."""

    wait_s: float = BLOCKED_SAY_S   # stopped this long before a try (3 s since 23 Nov)
    still_s: float = STILL_BLOCKED_S    # no try (a person stays): ask to be turned after this
    # 23 Nov (Adel): the door frame within 1 m was taken for a person and waited for 17 s.
    person_m: float = 0.6           # the LiDAR sees something this close straight ahead: wait
    # 23 Nov (Adel, a corner): a wall ahead never walks away; a person does within seconds.
    person_wait_s: float = 6.0      # ...but only this long into a hold-up, then try anyway
    say_s: float = 1.5              # after the sentence, before the wheels turn
    # 23 Nov (Adel): 0.6 was too quick to follow; 0.4 still gets about 60 % wheel power.
    turn_rps: float = 0.4           # the hazard gate's pivot rate
    max_turn_rad: float = math.radians(90)
    max_turn_s: float = 8.0
    clear_m: float = 1.2            # "way ahead clear": nothing this close to the front
    confirm_s: float = 0.2          # ...seen clear this long (23 Nov: it coasts 9-15 deg after)
    settle_s: float = 3.0           # the hazard gate must let go within this long
    drive_m: float = 0.5
    drive_mps: float = 0.15
    drive_s: float = 6.0
    max_tries: int = 2
    retry_s: float = 1.0            # a turn found nothing: the other way after this pause
    free_s: float = 8.0             # driving this long without a stop ends the hold-up


@dataclass
class Seen:
    """What the guide node knows at this moment."""

    guiding: bool = False           # a trip is on (not idle, not paused)
    blocked: bool = False           # the hazard gate stops forward driving now
    stage1_ok: bool = False         # Stage 1's level is fresh
    camera_stop: bool = False       # Stage 1's level is "stop"
    camera_m: float = None          # nearest thing ahead (camera), metres from the front
    master_stop: bool = False       # the master reports a stop (red, green, handle, ...)
    points: object = None           # LiDAR and camera points (N x 2, device frame), None = old
    lidar: object = None            # LiDAR points only, None = old
    yaw: float = None               # heading from the movement tracking, None = old
    x: float = None
    y: float = None
    goal_side: int = 0              # the goal is to the left (+1), right (-1), unknown (0)
    no_route: bool = False          # the planner finds no route now (23 Nov)


def angle_between(a, b):
    """Give the size of the turn from heading a to heading b, in radians (0..pi)."""
    return abs(math.atan2(math.sin(b - a), math.cos(b - a)))


def person_ahead(lidar, lanes, person_m):
    """Say if the LiDAR sees something straight ahead within person_m of the front."""
    if lidar is None:
        return False
    seen = lane_nearest(lidar, 0.0, lanes)
    return seen is not None and seen <= person_m


def pick_side(points, lanes, goal_side=0):
    """Give the side to turn to: +1 left, -1 right."""
    heading = clear_heading(points, lanes)
    if heading is not None and lane_nearest(points, -heading, lanes) is not None:
        return 1 if heading > 0 else -1             # only this side is clear that soon
    room = {}
    for side in (1, -1):
        seen = lane_nearest(points, side * math.pi / 2, lanes)
        room[side] = lanes.lane_m if seen is None else seen
    if abs(room[1] - room[-1]) > 0.1:
        return 1 if room[1] > room[-1] else -1
    return -1 if goal_side < 0 else 1


def way_clear(seen, lanes, clear_m):
    """Say if both the camera and the LiDAR see nothing within clear_m straight ahead."""
    if seen.camera_stop or (seen.camera_m is not None and seen.camera_m < clear_m):
        return False
    ahead = lane_nearest(seen.points, 0.0, lanes)
    return ahead is None or ahead >= clear_m


class Unstuck:
    """The decision, step by step. See the top of this file."""

    def __init__(self, settings=None, lanes=None):
        self.s = settings or UnstuckSettings()
        self.lanes = lanes or FreeSettings()
        self.tries = 0
        self.phase = 'off'              # off, announce, turn, gap, settle, drive
        self.phase_at = 0.0
        self.why = 'off'                # for the log: why the last try ended
        self.side, self.budget = 1, self.s.max_turn_rad
        self.last_yaw = self.start_xy = None
        self.turned = 0.0               # radians turned in this try (adds up past 180)
        self.reset_hold_up()

    @property
    def active(self):
        """Say if this program drives the wheels now (the route follower is held)."""
        return self.phase != 'off'

    def reset_hold_up(self):
        self.blocked_since = self.free_since = None
        self.said_wait = self.said_still = False
        self.failed = None              # (side, radians turned) of a turn that found nothing
        self.clear_since = None

    def can_move(self, seen):
        return seen.guiding and seen.stage1_ok and not seen.master_stop \
            and seen.points is not None and seen.yaw is not None and seen.x is not None

    def step(self, now, seen):
        """Give (command, actions) for this moment. See the top of this file."""
        if not seen.guiding:
            was = self.active
            self.phase, self.tries = 'off', 0
            self.reset_hold_up()
            return ((0.0, 0.0) if was else None), []
        if self.active:
            return self.run(now, seen)
        return self.watch(now, seen)

    # ---- not driving: watch for a hold-up -------------------------------------------------

    def watch(self, now, seen):
        s = self.s
        # 23 Nov (Adel, a desk corner): close to a corner the camera marked, the planner found no
        # route (the device's own spot counted as touching) while the camera saw 3.7 m clear
        # ahead; nothing blocked forward driving, so no try came and it stood for minutes. No
        # route is a hold-up too (the guide does the talking for it: "I can't find a way...").
        stuck = seen.blocked or seen.no_route
        if not stuck:
            self.blocked_since = None
            self.said_wait = self.said_still = False
            if self.free_since is None:
                self.free_since = now
            if now - self.free_since >= s.free_s:
                self.tries, self.failed = 0, None
            return None, []
        self.free_since = None
        if self.blocked_since is None:
            self.blocked_since = now
        held = now - self.blocked_since
        if held < s.wait_s:
            return None, []
        person = held < s.person_wait_s and person_ahead(seen.lidar, self.lanes, s.person_m)
        if self.tries < s.max_tries and self.can_move(seen) and not person:
            return self.start(now, seen)
        actions = []
        if not seen.blocked:
            return None, actions            # no route only: the guide says so
        if self.tries >= s.max_tries and not self.said_still:
            self.said_still = True
            actions.append(('say', STILL_BLOCKED))
        elif not self.said_wait and not self.said_still:
            self.said_wait = True
            actions.append(('say', BLOCKED))
        elif held >= s.still_s and not self.said_still:
            self.said_still = True
            actions.append(('say', STILL_BLOCKED))
        return None, actions

    def start(self, now, seen):
        """Begin a try: hold the route, say which way, stand still a moment."""
        self.tries += 1
        budget = self.s.max_turn_rad
        if self.failed is not None:
            self.side = -self.failed[0]
            budget += self.failed[1]        # back through where it came from
        else:
            self.side = pick_side(seen.points, self.lanes, seen.goal_side)
        self.failed = None
        self.budget = budget
        self.phase, self.phase_at = 'announce', now
        words = TURNING % ('left' if self.side > 0 else 'right')
        return (0.0, 0.0), [('hold',), ('say', words)]

    # ---- driving ------------------------------------------------------------------------

    def end(self, now, why, resume=True):
        self.phase, self.why = 'off', why
        self.blocked_since = self.free_since = self.clear_since = None
        return (0.0, 0.0), ([('resume',)] if resume else [])

    def run(self, now, seen):
        s = self.s
        if not self.can_move(seen):
            return self.end(now, 'stopped')
        since = now - self.phase_at
        if self.phase == 'announce':
            if since >= s.say_s:
                self.phase, self.phase_at, self.last_yaw = 'turn', now, seen.yaw
                self.clear_since, self.turned = None, 0.0
                return (0.0, self.side * s.turn_rps), []
            return (0.0, 0.0), []
        if self.phase == 'turn':
            return self.turn(now, seen, since)
        if self.phase == 'gap':
            if since < s.retry_s:
                return (0.0, 0.0), []
            if self.tries < s.max_tries and (seen.blocked or seen.no_route) \
                    and not (now - (self.blocked_since or now) < s.person_wait_s
                             and person_ahead(seen.lidar, self.lanes, s.person_m)):
                command, actions = self.start(now, seen)
                return command, [a for a in actions if a[0] != 'hold']
            return self.end(now, 'nothing clear')
        if self.phase == 'settle':
            if not seen.blocked:
                self.phase, self.phase_at, self.start_xy = 'drive', now, (seen.x, seen.y)
                return (s.drive_mps, 0.0), []
            if since >= s.settle_s:
                return self.end(now, 'gate kept it stopped')
            return (0.0, 0.0), []
        # drive
        moved = math.hypot(seen.x - self.start_xy[0], seen.y - self.start_xy[1])
        if seen.blocked:
            return self.end(now, 'stopped while driving')
        if moved >= s.drive_m or since >= s.drive_s:
            return self.end(now, 'done')
        return (s.drive_mps, 0.0), []

    def turn(self, now, seen, since):
        s = self.s
        if way_clear(seen, self.lanes, s.clear_m):
            if self.clear_since is None:
                self.clear_since = now
            if now - self.clear_since >= s.confirm_s:
                self.phase, self.phase_at = 'settle', now
                return (0.0, 0.0), []
        else:
            self.clear_since = None
        self.turned += angle_between(self.last_yaw, seen.yaw)
        self.last_yaw = seen.yaw
        if self.turned >= self.budget or since >= s.max_turn_s:
            self.failed = (self.side, self.turned)
            self.phase, self.phase_at = 'gap', now
            return (0.0, 0.0), []
        return (0.0, self.side * s.turn_rps), []
