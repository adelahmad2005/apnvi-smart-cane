"""
Find where the LiDAR is on a saved map, from one sweep, with no clue about where it started.

The saved map is turned into a "distance to the nearest wall" picture. Every spot on the free
floor and every turn (every 5 degrees) is tried: the sweep is placed there and scored. The best
few are then refined in small steps. The answer comes with how sure it is: its score, and how
much better it is than the best answer somewhere else (a room with two identical ends gives
two good answers).

The score (16 Nov night): a sweep point within a few centimetres of a wall is a hit. A point
that lands outside the known floor (it went through a wall) is a full miss. A point that lands
on free floor is something the map does not hold: far away it is a full miss, but close by
(under NEAR_M) it is most likely a person, a chair or a desk edge, and counts only NEAR_MISS of
a miss. Before this every such point was a full miss: from a spot between the desks, where
more than half the sweep is furniture 1.5 to 2.5 m away that the map does not hold, the right
spot scored about 40% and the device was never placed.
A sweep can also be several sweeps put together (placing.py, 17 Nov): each point then carries
where its beam started, so a beam from an older spot is checked along its own way.
No ROS in this file, so it can be tested with no hardware.
"""

import math

import cv2
import numpy as np

OCCUPIED_ABOVE = 50             # a map cell this likely a wall (0 to 100) is a wall
MATCH_M = 0.10                  # a sweep point within this of a wall is a hit
COARSE_MATCH_M = 0.25           # ...and within this in the coarse search (20 cm, 5 deg steps)
NEAR_M = 2.5                    # a point nearer than this that lands on free floor is most
NEAR_MISS = 0.2                 # likely not on the map (a person, a chair): this much of a miss
FAR_MISS = 0.6                  # a far point on free floor: this much of a miss (when the pole
#                                 leans forward the LiDAR sees the floor 5 m ahead as a wall)
SAMPLE_POINTS = 120             # use this many points of the sweep (spread evenly)
COARSE_POINTS = 150             # the coarse search uses at most this many points (CPU)
COARSE_STEP_M = 0.20            # coarse search: a spot every 20 cm...
COARSE_TURN_DEG = 5.0           # ...and a turn every 5 degrees
REFINE_STEP_M = 0.03
REFINE_TURN_DEG = 1.0
KEEP_BEST = 8                   # refine this many of the best coarse answers
WIDE_M = 0.8                    # the wide refine looks this far around a spot...
WIDE_TURN_DEG = 15.0            # ...and this far either way in turn
APART_M = 1.0                   # answers closer than this to a better one are the same answer
RANGE_MIN_M = 1.0               # nearer points are skipped: the person pushing, the chair
RANGE_MAX_M = 10.0              # far walls tell one end of a room from the other (was 6)
HIT, FLOOR, BEYOND = 0, 1, 2    # where a sweep point lands: on a wall, on free floor, outside
ON_THE_WAY = (1 / 3, 2 / 3)     # a beam is also checked at these parts of its length: in a wall
#                                 or off the known floor there, the point is a full miss


class Fit:
    """Where the sweep fits: map position x, y (metres), turn yaw (radians) and the scores."""

    def __init__(self, x, y, yaw, score, runner_up, others=()):
        self.x, self.y, self.yaw = x, y, yaw
        self.score = score              # share of points that hit a wall, 0 to 1
        self.runner_up = runner_up      # best score of an answer more than APART_M away
        self.others = list(others)      # the other answers, (x, y, yaw, score), best first

    def __repr__(self):
        return 'Fit(x=%.2f y=%.2f yaw=%.1f deg score=%.2f runner_up=%.2f)' % (
            self.x, self.y, math.degrees(self.yaw), self.score, self.runner_up)


class Relocalizer:
    """Hold a saved map's distance picture and find sweeps on it."""

    def __init__(self, grid, resolution, origin):
        """grid: rows of cells, row 0 at the lowest y; -1 unknown, 0 free, up to 100 wall."""
        self.resolution = resolution
        self.origin = origin
        walls = (grid > OCCUPIED_ABOVE).astype(np.uint8)
        self.distance = cv2.distanceTransform(1 - walls, cv2.DIST_L2, 3) * resolution
        self.free = grid == 0
        self.has_walls = bool(walls.any())
        self.kind = self.kinds(grid, MATCH_M)
        self.coarse_kind = self.kinds(grid, COARSE_MATCH_M)
        # A beam cannot cross a wall cell or floor nobody has seen: BEYOND there, FLOOR else.
        self.blocked = np.where((grid < 0) | (grid > OCCUPIED_ABOVE), BEYOND,
                                FLOOR).astype(np.int8)

    def kinds(self, grid, match_m):
        """Say for every map cell what a sweep point landing there is: HIT, FLOOR or BEYOND."""
        kind = np.full(grid.shape, BEYOND, np.int8)
        kind[grid >= 0] = FLOOR
        kind[self.distance <= match_m] = HIT
        return kind

    def sweep_points(self, ranges, angle_min, angle_increment):
        """
        Turn a sweep into points in the LiDAR's own frame, thinned to SAMPLE_POINTS.

        Rows: x, y, and how much of a miss the point is when it lands on free floor (NEAR_MISS
        for a close point, 1 for a far one).
        """
        ranges = np.asarray(ranges, float)
        angles = angle_min + angle_increment * np.arange(len(ranges))
        good = np.isfinite(ranges) & (ranges > RANGE_MIN_M) & (ranges < RANGE_MAX_M)
        ranges, angles = ranges[good], angles[good]
        if len(ranges) > SAMPLE_POINTS:
            pick = np.linspace(0, len(ranges) - 1, SAMPLE_POINTS).astype(int)
            ranges, angles = ranges[pick], angles[pick]
        return np.stack([ranges * np.cos(angles), ranges * np.sin(angles),
                         np.where(ranges < NEAR_M, NEAR_MISS, FAR_MISS)])

    def score(self, points, x, y, yaw, coarse=False):
        """
        How well the points (placed at x, y, yaw on the map) fit the walls, 0 to 1.

        Hits divided by hits plus misses; a close point on free floor is only part of a miss
        (see the top of the file). A point whose beam starts in or crosses a wall or unknown
        space is a full miss wherever it lands. `coarse` uses the wider match of the coarse
        search and skips that check. Rows 3 and 4 of `points`, when there, are where each beam
        started (an older sweep of a walk); else every beam starts at x, y.
        """
        x, y, yaw = np.atleast_1d(x), np.atleast_1d(y), np.atleast_1d(yaw)
        cos, sin = np.cos(yaw)[:, None], np.sin(yaw)[:, None]
        end_x = x[:, None] + points[0] * cos - points[1] * sin
        end_y = y[:, None] + points[0] * sin + points[1] * cos
        landed = self.look(self.coarse_kind if coarse else self.kind, end_x, end_y)
        if not coarse:
            if len(points) > 4:
                from_x = x[:, None] + points[3] * cos - points[4] * sin
                from_y = y[:, None] + points[3] * sin + points[4] * cos
            else:
                from_x, from_y = x[:, None], y[:, None]
            for part in ON_THE_WAY:
                way = self.look(self.blocked, from_x + part * (end_x - from_x),
                                from_y + part * (end_y - from_y))
                landed[way == BEYOND] = BEYOND
        hits = (landed == HIT).sum(axis=1)
        floor_miss = points[2] if len(points) > 2 else 1.0
        misses = (landed == BEYOND).sum(axis=1) + ((landed == FLOOR) * floor_miss).sum(axis=1)
        return hits / np.maximum(hits + misses, 1e-9)

    def look(self, kind, map_x, map_y):
        """Give the kind of the map cell at each point (BEYOND outside the map)."""
        col = np.floor((map_x - self.origin[0]) / self.resolution).astype(int)
        row = np.floor((map_y - self.origin[1]) / self.resolution).astype(int)
        high, wide = kind.shape
        inside = (col >= 0) & (col < wide) & (row >= 0) & (row < high)
        found = np.full(col.shape, BEYOND, np.int8)
        found[inside] = kind[row[inside], col[inside]]
        return found

    def find(self, points):
        """Search the whole map. Returns a Fit, or None when the map has no walls or no floor."""
        if not self.has_walls or len(points[0]) < 20:
            return None
        step = max(1, round(COARSE_STEP_M / self.resolution))
        rows, cols = np.nonzero(self.free[::step, ::step])
        xs = self.origin[0] + (cols * step + 0.5) * self.resolution
        ys = self.origin[1] + (rows * step + 0.5) * self.resolution
        if not len(xs):
            return None
        few = points
        if len(points[0]) > COARSE_POINTS:
            few = points[:, np.linspace(0, len(points[0]) - 1, COARSE_POINTS).astype(int)]
        yaws = np.radians(np.arange(0, 360, COARSE_TURN_DEG))
        scores = np.zeros((len(xs), len(yaws)))
        for k, yaw in enumerate(yaws):
            scores[:, k] = self.score(few, xs, ys, np.full(len(xs), yaw), coarse=True)
        best_per_spot = scores.max(axis=1)
        order = np.argsort(-best_per_spot)
        answers = []
        for index in order:
            if any(math.hypot(xs[index] - a[0], ys[index] - a[1]) < APART_M for a in answers):
                continue
            yaw = yaws[int(scores[index].argmax())]
            answers.append(self.refine(points, xs[index], ys[index], yaw))
            if len(answers) >= KEEP_BEST:
                break
        answers.sort(key=lambda a: -a[3])
        best = answers[0]
        others = [a for a in answers[1:]
                  if math.hypot(a[0] - best[0], a[1] - best[1]) >= APART_M]
        return Fit(best[0], best[1], best[2], best[3],
                   max(a[3] for a in others) if others else 0.0, others)

    def refine(self, points, x, y, yaw, reach_m=COARSE_STEP_M, reach_deg=COARSE_TURN_DEG):
        """Improve one answer with small steps around it. Returns (x, y, yaw, score)."""
        offsets = np.arange(-reach_m, reach_m + 1e-9, REFINE_STEP_M)
        turns = np.radians(np.arange(-reach_deg, reach_deg + 1e-9, REFINE_TURN_DEG))
        gx, gy, gt = np.meshgrid(offsets, offsets, turns, indexing='ij')
        scores = self.score(points, x + gx.ravel(), y + gy.ravel(), yaw + gt.ravel())
        k = int(scores.argmax())
        return x + gx.ravel()[k], y + gy.ravel()[k], yaw + gt.ravel()[k], float(scores[k])

    def refine_wide(self, points, x, y, yaw):
        """Like refine, but looks WIDE_M and WIDE_TURN_DEG around first (a spot set by hand)."""
        offsets = np.arange(-WIDE_M, WIDE_M + 1e-9, COARSE_STEP_M / 2)
        turns = np.radians(np.arange(-WIDE_TURN_DEG, WIDE_TURN_DEG + 1e-9, COARSE_TURN_DEG / 2))
        gx, gy, gt = np.meshgrid(offsets, offsets, turns, indexing='ij')
        scores = self.score(points, x + gx.ravel(), y + gy.ravel(), yaw + gt.ravel(), coarse=True)
        k = int(scores.argmax())
        return max(self.refine(points, x, y, yaw),
                   self.refine(points, x + gx.ravel()[k], y + gy.ravel()[k], yaw + gt.ravel()[k]),
                   key=lambda answer: answer[3])


def combine(first, second):
    """Put pose `second` (x, y, yaw) after pose `first`: where second lands, seen from outside."""
    x, y, yaw = first
    cos, sin = math.cos(yaw), math.sin(yaw)
    return (x + cos * second[0] - sin * second[1], y + sin * second[0] + cos * second[1],
            math.atan2(math.sin(yaw + second[2]), math.cos(yaw + second[2])))


def inverse(pose):
    """Give the pose that undoes this one: the two combined are no move at all."""
    x, y, yaw = pose
    cos, sin = math.cos(yaw), math.sin(yaw)
    return (-(cos * x + sin * y), sin * x - cos * y, -yaw)


def frame_offset(on_map, in_frame):
    """
    Give the pose of a frame on the map that puts a sensor at `on_map`.

    `in_frame` is the sensor's pose in that frame. Used for map -> rtabmap_map: the LiDAR is
    then shown where the search found it.
    """
    yaw = on_map[2] - in_frame[2]
    cos, sin = math.cos(yaw), math.sin(yaw)
    return (on_map[0] - (cos * in_frame[0] - sin * in_frame[1]),
            on_map[1] - (sin * in_frame[0] + cos * in_frame[1]),
            math.atan2(math.sin(yaw), math.cos(yaw)))


class Candidates:
    """
    Several places the device might be, followed as it moves, until one clearly wins.

    A room can look the same from two places (one sweep fits both). Each place is kept as a
    shift of the tracker's frame on the map, so it moves along with the device for free. Every
    check, each one is refined against the new sweep (or the outline of the last few metres,
    placing.py) and its fit remembered. Moving the device shows new walls: the wrong place
    stops fitting.

    The map's start spot (the room's entrance once it is marked, 17 Nov) is always one of the
    places and is never dropped (16 Nov night: it was dropped after three poor fits and could
    then never win, even with the device brought back to it). While the device may still stand
    there, each check it is also tried as "the device stands on the start spot now", and the
    better of the two is kept. It no longer wins a tie (17 Nov: started beside the taped spot,
    a 60% fit at the start spot placed the device wrong): it wins like any other place, or
    when the user was asked to go there.
    """

    REMEMBER = 3                # fits remembered per place (older views were the same for all)
    NEEDED = 3                  # fits a place needs before it can win
    DROP_BELOW = 0.45           # a place whose average fit falls below this is dropped
    MOST = 8                    # places followed at once at most
    FOLLOW_M = 0.12             # a followed place is refined this far around...
    FOLLOW_DEG = 3.0            # ...and this far in turn (CPU: a quarter of a full refine)

    def __init__(self):
        self.places = []        # dicts: shift (x, y, yaw), fits (list), start (bool)

    def add(self, shift, fit, start=False):
        self.places.append({'shift': shift, 'fits': [fit], 'start': start})

    def add_start(self, shift=(0.0, 0.0, 0.0), at=None, marked=None):
        """
        Add a start spot, with no fit yet (no shift: the tracker started there).

        at: where the LiDAR is on the map when the device stands on this spot (else update's
        at_start). marked: True for an entrance the user marked, False for the spot where the
        map was started (23 Nov: both are tried), None as before (one start spot).
        """
        self.places.append({'shift': shift, 'fits': [], 'start': True, 'at': at,
                            'marked': marked})

    def add_answers(self, answers, seen, least):
        """Add search answers (x, y, yaw, score on the map) not already followed."""
        for x, y, yaw, fit in answers:
            if fit < least:
                continue
            if any(math.hypot(where[0] - x, where[1] - y) < APART_M
                   for where in (combine(p['shift'], seen) for p in self.places)):
                continue
            self.add(frame_offset((x, y, yaw), seen), fit)

    def update(self, finder, points, seen, at_start=None):
        """
        Refine every place against this sweep, merge places that met, drop poor ones.

        at_start: where the LiDAR is on the map when the device stands on the start spot; give
        it only while the device may still stand there (or was asked to go there).
        """
        kept = []
        for place in self.places:
            if place['fits']:
                x, y, yaw, fit = finder.refine(points, *combine(place['shift'], seen),
                                               reach_m=self.FOLLOW_M, reach_deg=self.FOLLOW_DEG)
            else:
                x, y, yaw, fit = finder.refine(points, *combine(place['shift'], seen))
            if place['start'] and at_start is not None:
                again = finder.refine_wide(points, *(place.get('at') or at_start))
                if again[3] > fit:
                    x, y, yaw, fit = again
            place['shift'] = frame_offset((x, y, yaw), seen)
            place['fits'] = (place['fits'] + [fit])[-self.REMEMBER:]
            twin = next((k for k in kept if math.hypot(
                combine(k['shift'], seen)[0] - x, combine(k['shift'], seen)[1] - y) < APART_M / 2),
                None)
            if twin is not None:
                if place['start'] and not twin['start']:
                    twin['marked'], twin['at'] = place.get('marked'), place.get('at')
                elif place.get('marked'):
                    twin['marked'] = True
                twin['start'] = twin['start'] or place['start']
                continue
            if (not place['start'] and len(place['fits']) >= self.NEEDED
                    and self.average(place) < self.DROP_BELOW):
                continue
            kept.append(place)
        kept.sort(key=lambda p: (not p['start'], -self.average(p)))     # the start spot stays
        self.places = sorted(kept[:self.MOST], key=lambda p: -self.average(p))

    @staticmethod
    def average(place):
        return sum(place['fits']) / max(1, len(place['fits']))

    def winner(self, good_fit, gap, start_wins=False, rivals=(), seen=None, start_best=False):
        """
        Give (shift, why) of the place that clearly fits best, or None.

        It needs NEEDED fits, an average of `good_fit` or more, and to beat every other place by
        `gap`. The map's start spot also wins when it fits `good_fit` and `start_wins` (the
        user was asked to go there). It no longer wins a tie (17 Nov).
        rivals: (shift, fit) of the answers of the latest whole-map search, with `seen` the
        LiDAR in the tracker's frame now; those more than APART_M from the best place must be
        beaten by `gap` too (17 Nov: a place followed alone must not win just for having no
        rival followed).
        """
        ready = [p for p in self.places if len(p['fits']) >= self.NEEDED]
        if not ready:
            return None
        best = max(ready, key=self.average)
        rest = [self.average(p) for p in self.places if p is not best]
        if seen is not None:
            here = combine(best['shift'], seen)
            rest += [fit for shift, fit in rivals if math.hypot(
                *np.subtract(combine(shift, seen)[:2], here[:2])) >= APART_M]
        if self.average(best) >= good_fit and all(self.average(best) - r >= gap for r in rest):
            return best['shift'], 'it fits clearly best of %d places (%.0f%%%s)' % (
                len(self.places), 100 * self.average(best),
                ', next %.0f%%' % (100 * max(rest)) if rest else '')
        # 19 Nov (outside EB2): a marked entrance fitted 74-76% and best, the rest 66-72%; the
        # gap rule never let it win. A start spot the user marked wins when it fits well and best
        # 23 Nov (ESB117): the entrance won at 72% while the latest search had 79% and 75%
        # elsewhere, and the device was placed wrong: now it must fit best of all (no gap).
        if start_best and best['start'] and best.get('marked') is not False \
                and self.average(best) >= good_fit \
                and all(self.average(best) >= r for r in rest):
            return best['shift'], 'the marked entrance fits best (%.0f%%)' % (
                100 * self.average(best))
        if start_wins:
            for place in ready:
                if place['start'] and self.average(place) >= good_fit:
                    return place['shift'], 'the device was taken to the start spot (%.0f%%)' % (
                        100 * self.average(place))
        return None
