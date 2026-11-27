"""
When to believe the camera's "I have been here before" (18 Nov evening).

With camera_map:=true, RTAB-Map recognises a place by sight and sends where the device is on
the saved map (/rtabmap/localization_pose). The map manager turns that into an anchor: where
the movement tracker's frame (odom) sits on the map. This file decides whether to place the
device there.

Why not believe every guess: on 31 Oct the camera matched views of desks seen from far away to
the wrong spot (block3-results-31oct.md). Why not ask the LiDAR alone: in a big open area (the
outside of EB2, 18 Nov) there are too few walls to confirm anything; the LiDAR search never got
a clear win. So a guess places the device when
  - the LiDAR sweep fits the saved walls there clearly (wall_fit >= placed_fit), or
  - two guesses agree (the same anchor within agree_m and agree_deg, the second at most keep_s
    after the first) and the walls do not speak against it (wall_fit >= min_fit).
A wrong match is rarely made twice the same way; a right one is.
"""

import math

PLACED_FIT = 0.7        # as the LiDAR search: a fit this good is enough on its own
MIN_FIT = 0.3           # two agreeing guesses still need this much: the walls must not object
AGREE_M = 0.5           # two anchors this close...
AGREE_DEG = 15.0        # ...and this little turned agree
KEEP_S = 60.0           # an older guess is forgotten
QUIET_S = 90.0          # the camera said nothing this long: the LiDAR may place on its own


def anchors_agree(first, second, agree_m=AGREE_M, agree_deg=AGREE_DEG):
    """True if two anchors (x, y, yaw) are about the same."""
    turn = math.atan2(math.sin(first[2] - second[2]), math.cos(first[2] - second[2]))
    return (math.hypot(first[0] - second[0], first[1] - second[1]) <= agree_m
            and abs(math.degrees(turn)) <= agree_deg)


class CameraGuesses:
    """Collect the camera's guesses; say when one is good enough to place the device."""

    def __init__(self, placed_fit=PLACED_FIT, min_fit=MIN_FIT, keep_s=KEEP_S, quiet_s=QUIET_S):
        self.placed_fit, self.min_fit, self.keep_s = placed_fit, min_fit, keep_s
        self.quiet_s = quiet_s
        self.last = None                # (anchor, when) of the last guess the walls allowed
        self.newest = None              # (anchor, when) of the newest guess of any kind
        self.heard_at = None            # when the camera last guessed (kept across loads)
        self.note = ''

    def forget(self, new_map=False):
        self.last = None
        if new_map:
            self.newest = None          # a guess on the old map says nothing about this one

    def heard(self, anchor, now):
        """Note every guess the camera makes (before it is judged)."""
        self.newest, self.heard_at = (anchor, now), now

    def agrees(self, anchor, now):
        """True if the camera's newest guess (at most keep_s old) is about this anchor."""
        return (self.newest is not None and now - self.newest[1] <= self.keep_s
                and anchors_agree(anchor, self.newest[0]))

    def lets_lidar_place(self, anchor, now):
        """
        May the LiDAR alone place the device here? (19 Nov, ESB117.)

        Twice the LiDAR search was sure of a place 3 to 4 m off (84% against 66%) while the
        camera was right. So while the camera is working, a LiDAR win counts only where the
        camera's newest guess agrees. With no camera (camera_map off), or after quiet_s with no
        guess at all, the LiDAR decides alone as before.
        """
        if self.heard_at is None or now - self.heard_at > self.quiet_s:
            return True
        return self.agrees(anchor, now)

    def add(self, anchor, wall_fit, now):
        """
        Take one guess. Return why it places the device ('walls fit' or 'seen twice'), or None.

        anchor: odom on the map, (x, y, yaw). wall_fit: how well the LiDAR sweep fits the saved
        walls with that anchor (0..1). now: seconds, any clock that only goes forward.
        """
        if wall_fit >= self.placed_fit:
            self.note = 'camera guess, walls fit %.0f%%' % (100 * wall_fit)
            self.last = None
            return 'walls fit'
        if wall_fit < self.min_fit:
            self.note = 'camera guess refused: walls fit only %.0f%%' % (100 * wall_fit)
            return None
        before = self.last
        self.last = (anchor, now)
        if before is not None and now - before[1] <= self.keep_s and anchors_agree(before[0],
                                                                                    anchor):
            self.note = 'camera guess seen twice, walls fit %.0f%%' % (100 * wall_fit)
            self.last = None
            return 'seen twice'
        self.note = 'camera guess kept, waiting for a second one (walls fit %.0f%%)' % (
            100 * wall_fit)
        return None
