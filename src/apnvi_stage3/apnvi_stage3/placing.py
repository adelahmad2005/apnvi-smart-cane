"""
Find the device on a loaded map while it is walked: walk-and-match (17 Nov, Adel).

One LiDAR sweep in ESB117 often fits two places (the room looks alike turned round). So while
the device is not placed, the sweeps of the last few metres of walking are put together into
one wider outline, using the movement tracker (odom) for where each was taken. The wider
outline is fitted to the saved map with the same fit as one sweep (relocalizer.py). Several
possible places are followed as the device moves; the ones that stop fitting are dropped. The
device is placed only on a clear win: a fit of `placed_fit` and a lead of `gap` over every
other place, and over the best answer elsewhere of the latest whole-map search (17 Nov: a
place followed alone won at 71% with no rival followed). The more the user walks, the
surer it gets.

The start spot: the room's entrance once it is marked on the map ("save this place as
entrance", standing in the doorway facing in), else the spot where the map was started.
23 Nov (Adel: "indoors it should not need the entrance"): with an entrance marked, the spot
where the map was started (START in ESB117) is a second start spot (`also_start`). Before, the
entrance replaced it, so a device started on START searched the whole room (76 s). Each is
one of the places from the first check on, tried as "the device stands there now" until it
has walked NEAR_START_M. It does not win a tie: it must win clearly like any other place
(17 Nov: started beside the taped spot, a 60% fit there placed the device wrong).

CPU (the Jetson is busy): a sweep is kept only every STEP_M of movement, older sweeps are
thinned, the whole map is searched only when the outline has grown (not while standing still),
and the coarse search uses at most COARSE_POINTS points.
No ROS in this file, so it can be tested with no hardware.
"""

import math

import numpy as np

from .relocalizer import APART_M, Candidates, combine, frame_offset, inverse

SEARCH_EVERY = 3                # search the whole map at most every this many checks
NEAR_START_M = 1.0              # walked less than this: may still stand on the start spot
CANDIDATE_FIT = 0.6             # a search answer this good is followed as a possible place


def turn_between(a, b):
    """Give the turn from heading a to heading b, -pi to pi."""
    return math.atan2(math.sin(b - a), math.cos(b - a))


class Outline:
    """
    The sweeps of the last few metres of walking, put together as one wider outline.

    Each kept sweep remembers where the LiDAR was in the tracker's frame. The outline is given
    in the frame of the LiDAR now, with the start of every beam, so the fit checks each beam
    along its own way (relocalizer.Relocalizer.score).
    """

    STEP_M = 0.5                # keep a sweep every this much movement...
    TURN_DEG = 30.0             # ...or this much turn
    KEEP_M = 4.0                # keep the sweeps of about this much walking
    MOST_SWEEPS = 10            # and never more than this many
    OLD_POINTS = 30             # points kept of each older sweep (the newest keeps all)

    def __init__(self):
        self.sweeps = []        # (LiDAR pose in the tracker's frame, points), oldest first

    def add(self, pose, points):
        """Keep this sweep if the LiDAR moved STEP_M or turned TURN_DEG. True if kept."""
        if self.sweeps:
            last = self.sweeps[-1][0]
            if (math.hypot(pose[0] - last[0], pose[1] - last[1]) < self.STEP_M
                    and abs(turn_between(last[2], pose[2])) < math.radians(self.TURN_DEG)):
                return False
        count = len(points[0])
        if count > self.OLD_POINTS:
            points = points[:, np.linspace(0, count - 1, self.OLD_POINTS).astype(int)]
        self.sweeps.append((tuple(pose), points))
        while len(self.sweeps) > self.MOST_SWEEPS or (
                len(self.sweeps) > 2 and self.walked(1) > self.KEEP_M):
            self.sweeps.pop(0)
        return True

    def walked(self, first=0):
        """Metres walked between kept sweep number `first` and the newest."""
        poses = [pose for pose, _points in self.sweeps[first:]]
        return sum(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(poses, poses[1:]))

    def points(self, pose, newest):
        """
        Give the newest sweep with the kept ones, in the frame of the LiDAR at `pose`.

        Rows: x, y, how much of a miss on free floor (as Relocalizer.sweep_points), and where
        the beam started (x, y). A kept sweep taken at about this pose is the newest one again
        and is left out.
        """
        rows = [np.vstack([newest[:3], np.zeros((2, len(newest[0])))])]
        to_now = inverse(pose)
        for where, kept in self.sweeps:
            x, y, yaw = combine(to_now, where)
            if math.hypot(x, y) < self.STEP_M / 2 and abs(yaw) < math.radians(self.TURN_DEG / 2):
                continue
            cos, sin = math.cos(yaw), math.sin(yaw)
            rows.append(np.vstack([x + cos * kept[0] - sin * kept[1],
                                   y + sin * kept[0] + cos * kept[1], kept[2],
                                   np.full(len(kept[0]), x), np.full(len(kept[0]), y)]))
        return np.hstack(rows)


class Placer:
    """
    Not placed yet: one check a second, until one place clearly wins.

    finder: the Relocalizer of the loaded map. start: (x, y, yaw) of the device on the map at
    the start spot (the entrance, facing in). Settings as the map manager's: placed_fit and
    gap for a followed place, sure_fit (and gap) for one search answer.
    """

    def __init__(self, finder, start=(0.0, 0.0, 0.0), placed_fit=0.7, sure_fit=0.75, gap=0.1,
                 entrance=False, also_start=None):
        self.finder, self.start = finder, tuple(start)
        self.also_start = None if also_start is None else tuple(also_start)
        self.entrance = entrance        # start is an entrance the user marked (19 Nov)
        self.placed_fit, self.sure_fit, self.gap = placed_fit, sure_fit, gap
        self.outline = Outline()
        self.candidates = Candidates()
        self.checks = 0
        self.searched_at = None     # the check of the last whole-map search
        self.grown = True           # the outline grew since that search
        self.rivals = []            # (shift, fit) of that search's answers
        self.walked_m = 0.0
        self.last_seen = None
        self.note = ''              # what the last check saw, for the log

    def step(self, points, seen, laser_on_device, asked=False):
        """
        One check. Returns (shift of the tracker's frame on the map, why) once placed, else None.

        points: the newest sweep (Relocalizer.sweep_points). seen: where the LiDAR is in the
        tracker's frame. laser_on_device: where the LiDAR sits on the device. asked: the user
        was asked to take the device to the start spot.
        """
        if self.last_seen is not None:
            self.walked_m += math.hypot(seen[0] - self.last_seen[0], seen[1] - self.last_seen[1])
        self.last_seen = seen
        self.grown = self.outline.add(seen, points) or self.grown
        wide = self.outline.points(seen, points)
        at_start = combine(self.start, laser_on_device)
        if self.checks == 0:
            self.candidates.add_start(frame_offset(at_start, seen), at=at_start,
                                      marked=self.entrance)
            if self.also_start is not None:
                at_also = combine(self.also_start, laser_on_device)
                self.candidates.add_start(frame_offset(at_also, seen), at=at_also, marked=False)
        near_start = asked or self.walked_m < NEAR_START_M
        self.candidates.update(self.finder, wide, seen, at_start if near_start else None)
        found = None
        if self.grown and (self.searched_at is None
                           or self.checks - self.searched_at >= SEARCH_EVERY):
            found = self.finder.find(wide)
            self.searched_at, self.grown = self.checks, False
            if found is not None:
                self.rivals = [(frame_offset(answer[:3], seen), answer[3]) for answer in
                               [(found.x, found.y, found.yaw, found.score)] + found.others]
        self.checks += 1
        if found is not None:
            rivals = [Candidates.average(place) for place in self.candidates.places
                      if len(place['fits']) >= Candidates.NEEDED and math.hypot(
                          *np.subtract(combine(place['shift'], seen)[:2], (found.x, found.y)))
                      >= APART_M]
            rival = max([found.runner_up] + rivals)
            if found.score >= self.sure_fit and found.score - rival >= self.gap:
                self.note = 'searched the map, sure: %s' % found
                return (frame_offset((found.x, found.y, found.yaw), seen),
                        'found by searching the map (%s)' % found)
            self.candidates.add_answers([(found.x, found.y, found.yaw, found.score)]
                                        + found.others, seen, CANDIDATE_FIT)
        following = []
        for place in self.candidates.places:
            x, y, _yaw = combine(place['shift'], seen)
            following.append('%s(%.1f, %.1f) %.0f%%' % (
                'start spot ' if place['start'] else '', x, y, 100 * Candidates.average(place)))
        self.note = '%s; outline %d points, %.1f m; following %s' % (
            'searched the map, not sure: %s' % found if found is not None else 'not placed yet',
            len(wide[0]), self.outline.walked(), ', '.join(following))
        return self.candidates.winner(self.placed_fit, self.gap, start_wins=asked,
                                      rivals=self.rivals, seen=seen, start_best=self.entrance)
