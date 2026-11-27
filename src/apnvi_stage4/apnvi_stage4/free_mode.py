"""
Free mode steering (the "redirection"), with no ROS, so it can be tested anywhere.

Adel, 15 Nov: with no trip on, the user pushes the device like a walker, and the
device steers the user away from what is in the way, so the handle pulls them round it.

The rule that makes it safe: **in free mode the device never drives forward. It only turns.**
The two wheels get equal and opposite power, so all forward movement comes from the user.
If the user lets go, the device stops moving forward, the "is the user pushing" test fails,
and the turning stops too. It cannot run away.

What it does, 10 times a second (Steering.step):
  1. Is the user pushing? Forward speed, measured from the LiDAR's movement tracking, must be
     at least push_mps for push_for_s (and stay above keep_mps while it turns).
  2. What is ahead? All points the LiDAR and the camera see, in the device's own frame
     (x forward from the wheels' centre, y to the left). The lane is the strip in front of the
     device: from its front edge to lane_m ahead, half_width_m to each side (the device and
     the user behind it).
  3. Lane clear: no turning. In a narrow gap (something close on both sides) it turns gently
     toward the middle.
  4. Lane blocked: it tries headings 10, 20 ... 60 degrees to each side and turns toward the
     smallest one whose lane is clear (the side with more room wins a tie). It keeps to that
     side until the lane ahead is clear, so it never flips left-right-left.
  5. No clear heading: no turning. The grip gives "stopped" (Stage 1's rule); the user turns
     round themselves.
  6. One turn is at most max_turn_rad or max_turn_s; then it rests rest_s.
"""

from collections import deque
from dataclasses import dataclass
import math

import numpy as np


@dataclass
class FreeSettings:
    """Numbers for free mode steering (all to tune on the floor)."""

    push_mps: float = 0.15          # the user is pushing: forward speed at least this
    keep_mps: float = 0.10          # while turning, it carries on above this
    push_for_s: float = 0.3         # pushed this long before the first turn
    front_m: float = 0.13           # the base's front edge, ahead of the wheels' centre
    lane_m: float = 1.5             # how far ahead of the front edge it looks
    half_width_m: float = 0.45      # half the lane's width (the device, and the user behind)
    turn_rps: float = 0.4           # turn rate round an obstacle
    centre_rps: float = 0.2         # top turn rate when centring in a narrow gap
    centre: bool = True
    gap_side_m: float = 0.8         # "narrow gap": something this close on both sides
    centre_dead_m: float = 0.15     # sides this equal: no centring
    headings_deg: tuple = (10, 20, 30, 40, 50, 60)
    max_turn_rad: float = math.radians(60)
    max_turn_s: float = 3.0
    rest_s: float = 1.0
    clear_for_s: float = 0.5        # the lane must stay clear this long to forget the side
    max_range_m: float = 4.0        # points further than this are not looked at


def steering_allowed(state):
    """
    Say if free mode may steer, from the guide's state (its /stage4/guide_state as a dict).

    18 Nov (lab): steering went off while the user pushed with a trip paused (red) or waiting
    for the device to find its place. In both the device does not drive itself, so free mode
    steers: no trip, a paused trip, or a trip still waiting for its place.
    """
    name = state.get('state')
    return name in ('idle', 'paused') or (name == 'guiding' and state.get('placed') is False)


def rotate(points, angle):
    """Turn points (N x 2) by angle (radians, anticlockwise) about the wheels' centre."""
    if len(points) == 0:
        return points
    c, s = math.cos(angle), math.sin(angle)
    return points @ np.array([[c, s], [-s, c]])


def sweep_points(ranges, angle_min, angle_increment, turn, shift, max_range_m=4.0,
                 min_range_m=0.05):
    """
    Turn a sweep into points in the device's frame: an N x 2 array of (x forward, y left).

    turn: the sensor's 3 x 3 rotation in the device's frame; shift: its (x, y, z) position.
    Both come from the frames the launch file sets (the LiDAR is mounted upside down and
    facing back, so its sweep cannot be used as it comes). inf, nan and far readings are
    dropped.
    """
    ranges = np.asarray(ranges, dtype=np.float64)
    angles = angle_min + angle_increment * np.arange(len(ranges))
    good = np.isfinite(ranges) & (ranges >= min_range_m) & (ranges <= max_range_m)
    if not good.any():
        return np.zeros((0, 2))
    ranges, angles = ranges[good], angles[good]
    local = np.stack([ranges * np.cos(angles), ranges * np.sin(angles),
                      np.zeros(len(ranges))], axis=1)
    there = local @ np.asarray(turn, dtype=np.float64).T + np.asarray(shift, dtype=np.float64)
    return there[:, :2]


def quaternion_matrix(x, y, z, w):
    """Give the 3 x 3 rotation of a quaternion (as in a ROS transform)."""
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def lane_nearest(points, heading, settings):
    """
    Give the distance (from the front edge) to the nearest point in the lane at this heading.

    heading: radians, + = to the left of where the device faces now. None = the lane is clear.
    """
    if len(points) == 0:
        return None
    turned = rotate(points, -heading)
    ahead = turned[:, 0] - settings.front_m
    inside = (ahead > 0) & (ahead <= settings.lane_m) & \
        (np.abs(turned[:, 1]) <= settings.half_width_m)
    if not inside.any():
        return None
    return float(ahead[inside].min())


def side_room(points, settings):
    """
    Give (left, right): how far the nearest thing is on each side of the lane, in metres.

    Looked at beside the lane's first metre; None for a side with nothing within gap_side_m.
    """
    if len(points) == 0:
        return None, None
    ahead = points[:, 0] - settings.front_m
    beside = (ahead > -0.3) & (ahead <= 1.0)
    y = points[beside, 1]
    left = y[(y > settings.half_width_m) & (y <= settings.half_width_m + settings.gap_side_m)]
    right = -y[(y < -settings.half_width_m) & (y >= -settings.half_width_m
                                               - settings.gap_side_m)]
    return (float(left.min()) if len(left) else None,
            float(right.min()) if len(right) else None)


def clear_heading(points, settings, keep_side=0):
    """
    Give the heading to turn to (radians, + = left), or None if nothing within reach is clear.

    The smallest heading whose lane is clear. Both sides clear at the same size: the side
    whose lane further round stays clear longer. keep_side (+1 left, -1 right): only that side.
    """
    for degrees in settings.headings_deg:
        angle = math.radians(degrees)
        sides = [side for side in (1, -1) if keep_side in (0, side)
                 and lane_nearest(points, side * angle, settings) is None]
        if len(sides) == 1:
            return sides[0] * angle
        if len(sides) == 2:
            wider = math.radians(settings.headings_deg[-1])
            room = {}
            for side in sides:
                seen = lane_nearest(points, side * wider, settings)
                room[side] = settings.lane_m if seen is None else seen
            return (1 if room[1] >= room[-1] else -1) * angle
    return None


class PushMeter:
    """
    How fast the user pushes the device forward, from its positions over the last moments.

    The LiDAR's movement tracking gives the position about 10 times a second. The speed is
    the distance moved along the way the device faces, over the last window_s. Turning on
    the spot is not forward movement, so it reads about 0.
    """

    def __init__(self, window_s=0.4, stale_s=0.6):
        self.window_s, self.stale_s = window_s, stale_s
        self.track = deque()

    def add(self, t, x, y, yaw):
        self.track.append((t, x, y, yaw))
        while len(self.track) > 2 and t - self.track[1][0] >= self.window_s:
            self.track.popleft()

    def speed(self, now):
        """Give the forward speed in m/s (+ = forward); 0 with too little or old data."""
        if len(self.track) < 2 or now - self.track[-1][0] > self.stale_s:
            return 0.0
        t0, x0, y0, _ = self.track[0]
        t1, x1, y1, yaw = self.track[-1]
        if t1 - t0 < 0.15:
            return 0.0
        return ((x1 - x0) * math.cos(yaw) + (y1 - y0) * math.sin(yaw)) / (t1 - t0)

    def yaw(self):
        """Give the newest heading, or None."""
        return self.track[-1][3] if self.track else None


class Steering:
    """Free mode's decision, step by step. Give it what is seen; it gives a turn rate."""

    def __init__(self, settings=None):
        self.settings = settings or FreeSettings()
        self.pushing_since = None       # when the speed first went above push_mps
        self.side = 0                   # the side it is going round on: +1 left, -1 right
        self.clear_since = None
        self.turn_started = None        # (time, yaw) when this turn began
        self.rest_until = 0.0
        self.why = 'idle'               # for the status line: idle, clear, round, centre,
        #                                 no_way, rest, not_pushed

    def stop(self, why):
        self.turn_started = None
        self.why = why
        return 0.0

    def step(self, now, speed, yaw, points):
        """
        Give the turn rate in rad/s (+ = left) for this moment. Forward speed is always 0.

        now: seconds, any clock that only goes forward. speed: the user's push (m/s forward).
        yaw: the device's heading (radians) or None. points: what is seen (N x 2, device frame).
        """
        s = self.settings
        turning = self.turn_started is not None
        if speed < (s.keep_mps if turning else s.push_mps):
            self.pushing_since = None
            self.side, self.clear_since = 0, None
            return self.stop('not_pushed')
        if self.pushing_since is None:
            self.pushing_since = now
        if now - self.pushing_since < s.push_for_s:
            return self.stop('not_pushed')
        if now < self.rest_until:
            return self.stop('rest')
        if lane_nearest(points, 0.0, s) is None:            # nothing ahead
            if self.clear_since is None:
                self.clear_since = now
            if now - self.clear_since >= s.clear_for_s:
                self.side = 0
            self.turn_started = None
            return self.centre(points)
        self.clear_since = None
        heading = clear_heading(points, s, self.side)
        if heading is None:
            self.side = 0
            return self.stop('no_way')
        self.side = 1 if heading > 0 else -1
        if self.turn_started is None:
            self.turn_started = (now, yaw)
        began, yaw_then = self.turn_started
        turned = 0.0 if yaw is None or yaw_then is None else abs(
            math.atan2(math.sin(yaw - yaw_then), math.cos(yaw - yaw_then)))
        if now - began >= s.max_turn_s or turned >= s.max_turn_rad:
            self.rest_until = now + s.rest_s
            return self.stop('rest')
        self.why = 'round'
        return self.side * s.turn_rps

    def centre(self, points):
        """With the lane clear: in a narrow gap, turn gently toward the middle."""
        s = self.settings
        self.why = 'clear'
        if not s.centre:
            return 0.0
        left, right = side_room(points, s)
        if left is None or right is None or abs(left - right) <= s.centre_dead_m:
            return 0.0
        self.why = 'centre'
        return max(-s.centre_rps, min(s.centre_rps, 0.5 * (left - right)))
