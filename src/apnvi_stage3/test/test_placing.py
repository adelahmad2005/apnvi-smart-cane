"""Tests for placing.py: walk-and-match and the stricter placing (17 Nov). No ROS needed."""

import math

import numpy as np

from apnvi_stage3.placing import Outline, Placer
from apnvi_stage3.relocalizer import combine, Relocalizer

RES = 0.05
ORIGIN = (-1.0, -1.0)
STILL = (0.0, 0.0, 0.0)             # the LiDAR sits on the device's own spot in these tests


def mirror_room():
    """Make a 20 x 4 m room whose ends look alike, with a pillar halfway."""
    grid = np.full((120, 440), -1, np.int8)
    grid[20:100, 20:420] = 0                      # floor from (0, 0) to (20, 4)
    grid[20, 20:420] = grid[99, 20:420] = 100
    grid[20:100, 20] = grid[20:100, 419] = 100
    grid[70:90, 200:240] = 100                    # the pillar: x 9 to 11, y 2.5 to 3.5
    return grid


def sweep(grid, x, y, yaw, beams=360):
    """Make a fake LiDAR sweep from a pose: ray-march until a wall."""
    ranges = []
    for k in range(beams):
        angle = yaw + 2 * math.pi * k / beams
        r = 0.0
        while r < 12.0:
            r += 0.02
            col = int((x + r * math.cos(angle) - ORIGIN[0]) / RES)
            row = int((y + r * math.sin(angle) - ORIGIN[1]) / RES)
            if grid[row, col] > 50:
                break
        ranges.append(r)
    return ranges, 0.0, 2 * math.pi / beams


def test_outline_keeps_a_sweep_every_half_metre_and_only_the_last_metres():
    outline = Outline()
    points = np.ones((3, 120))
    kept = [outline.add((0.1 * k, 0.0, 0.0), points) for k in range(100)]     # 10 m walked
    assert sum(kept) == 20                                  # one every 0.5 m
    assert outline.walked() <= Outline.KEEP_M + Outline.STEP_M
    assert len(outline.sweeps) <= Outline.MOST_SWEEPS
    assert all(len(p[0]) == Outline.OLD_POINTS for _pose, p in outline.sweeps)
    assert outline.add((9.9, 0.0, math.radians(40)), points)                  # a turn is kept


def test_outline_puts_older_sweeps_where_they_were_seen_from():
    grid = mirror_room()
    finder = Relocalizer(grid, RES, ORIGIN)
    outline = Outline()
    first, now = (3.0, 1.0, 0.0), (4.0, 1.5, 0.5)       # the LiDAR on the map, then 1.1 m on
    outline.add(STILL, finder.sweep_points(*sweep(grid, *first)))
    seen = (1.0, 0.5, 0.5)                              # the tracker: from first to now
    newest = finder.sweep_points(*sweep(grid, *now))
    wide = outline.points(seen, newest)
    assert len(wide) == 5 and len(wide[0]) == len(newest[0]) + Outline.OLD_POINTS
    assert finder.score(wide, *now)[0] > 0.9            # every point lands on a wall
    old = wide[:, len(newest[0]):]
    start = combine(now, (old[3, 0], old[4, 0], 0.0))   # its beams start where it was taken
    assert math.hypot(start[0] - first[0], start[1] - first[1]) < 1e-6


def test_beams_of_older_sweeps_are_checked_from_where_they_started():
    grid = mirror_room()
    finder = Relocalizer(grid, RES, ORIGIN)
    # A wall point at (9.0, 3.0) (the pillar's left side), seen from (8.0, 3.0). From the LiDAR
    # now at (12.0, 3.0) the line to it crosses the pillar: as if seen from here, a full miss.
    point = np.array([[-3.0], [0.0], [1.0]])
    assert finder.score(point, 12.0, 3.0, 0.0)[0] == 0.0
    started = np.vstack([point, [[-4.0], [0.0]]])
    assert finder.score(started, 12.0, 3.0, 0.0)[0] == 1.0


def run(finder, grid, start, truth, steps, step_m=0.5):
    """Walk the placer along +x from `truth`, a step a check. Returns (check, map pose) or None."""
    placer = Placer(finder, start)
    for check in range(steps):
        moved = check * step_m
        here = (truth[0] + moved, truth[1], truth[2])
        seen = (moved, 0.0, 0.0)
        won = placer.step(finder.sweep_points(*sweep(grid, *here)), seen, STILL)
        if won is not None:
            return check, combine(won[0], seen), here
    return None


def test_a_start_spot_that_fits_as_well_as_its_twin_does_not_place():
    # 17 Nov: the device was not on the start spot; a 60% fit there won the tie with its twin
    # and placed it wrong. Standing still in a room that looks alike turned round: no place.
    grid = mirror_room()
    finder = Relocalizer(grid, RES, ORIGIN)
    truth = (1.5, 1.0, 0.0)
    twin = (20 - truth[0], 4 - truth[1], math.pi)       # the start spot is the wrong end
    placer = Placer(finder, twin)
    for _ in range(6):
        assert placer.step(finder.sweep_points(*sweep(grid, *truth)), STILL, STILL) is None


def test_walking_places_the_device_right_from_the_wrong_start_spot():
    grid = mirror_room()
    finder = Relocalizer(grid, RES, ORIGIN)
    truth = (1.5, 1.0, 0.0)
    twin = (20 - truth[0], 4 - truth[1], math.pi)
    found = run(finder, grid, twin, truth, 18)
    assert found is not None
    _check, where, here = found
    assert math.hypot(where[0] - here[0], where[1] - here[1]) < 0.15
    assert abs(math.atan2(math.sin(where[2] - here[2]), math.cos(where[2] - here[2]))) < 0.07


def test_the_entrance_is_followed_when_the_device_is_beside_it():
    # The user comes in through the door but not exactly on the marked spot.
    grid = mirror_room()
    finder = Relocalizer(grid, RES, ORIGIN)
    truth = (1.5, 1.0, 0.0)
    entrance = (1.9, 1.3, math.radians(-10))           # marked 0.5 m and 10 degrees away
    placer = Placer(finder, entrance)
    placer.step(finder.sweep_points(*sweep(grid, *truth)), STILL, STILL)
    start = next(p for p in placer.candidates.places if p['start'])
    where = combine(start['shift'], STILL)
    assert math.hypot(where[0] - truth[0], where[1] - truth[1]) < 0.15
    found = run(finder, grid, entrance, truth, 18)
    assert found is not None
    _check, where, here = found
    assert math.hypot(where[0] - here[0], where[1] - here[1]) < 0.15


def test_a_place_followed_alone_must_beat_the_latest_search_too():
    # 17 Nov (lap_202918 replay): only one place was followed, so it "fitted clearly best of 1
    # places" at 71% with no rival. The latest whole-map search's answers elsewhere count too.
    from apnvi_stage3.relocalizer import Candidates
    candidates = Candidates()
    candidates.add((0.0, 0.0, 0.0), 0.75)
    candidates.places[0]['fits'] = [0.75, 0.75, 0.75]
    assert candidates.winner(0.7, 0.1) is not None                 # no rival: it used to win
    twin = [((5.0, 0.0, math.pi), 0.7)]                             # the search's other answer
    assert candidates.winner(0.7, 0.1, rivals=twin, seen=STILL) is None
    near = [((0.5, 0.0, 0.0), 0.74)]                                # the same place: no rival
    assert candidates.winner(0.7, 0.1, rivals=near, seen=STILL) is not None
    weak = [((5.0, 0.0, math.pi), 0.6)]
    assert candidates.winner(0.7, 0.1, rivals=weak, seen=STILL) is not None


def test_with_an_entrance_marked_the_map_start_is_tried_too():
    # 23 Nov (Adel): the device started on START, but the marked entrance had replaced START as
    # the start spot, so it searched the whole room for 76 s. Both are start spots now.
    grid = mirror_room()
    finder = Relocalizer(grid, RES, ORIGIN)
    truth = (1.5, 1.0, 0.0)                             # where the map was started (START)
    entrance = (10.0, 1.0, math.pi / 2)                 # the door, elsewhere in the room
    placer = Placer(finder, entrance, entrance=True, also_start=truth)
    placer.step(finder.sweep_points(*sweep(grid, *truth)), STILL, STILL)
    starts = [p for p in placer.candidates.places if p['start']]
    assert len(starts) == 2
    assert any(math.hypot(*np.subtract(combine(p['shift'], STILL)[:2], truth[:2])) < 0.15
               for p in starts)
    for check in range(18):
        seen = (check * 0.5, 0.0, 0.0)
        here = (truth[0] + seen[0], truth[1], truth[2])
        won = placer.step(finder.sweep_points(*sweep(grid, *here)), seen, STILL)
        if won is not None:
            where = combine(won[0], seen)
            assert math.hypot(where[0] - here[0], where[1] - here[1]) < 0.15
            return
    raise AssertionError('not placed')


def test_only_a_marked_entrance_gets_the_entrance_win():
    from apnvi_stage3.relocalizer import Candidates
    for marked, wins in ((True, True), (None, True), (False, False)):
        candidates = Candidates()
        candidates.add_start((0.0, 0.0, 0.0), marked=marked)
        candidates.places[0]['fits'] = [0.74, 0.75, 0.76]
        candidates.add((5.0, 0.0, 0.0), 0.7)
        candidates.places[1]['fits'] = [0.7, 0.7, 0.7]
        won = candidates.winner(0.7, 0.1, start_best=True)
        assert (won is not None) == wins, marked


def test_the_entrance_does_not_win_over_a_better_search_answer():
    # 23 Nov (ESB117): the entrance won at 72% while the search had 79% elsewhere: placed wrong.
    from apnvi_stage3.relocalizer import Candidates
    candidates = Candidates()
    candidates.add_start((0.0, 0.0, 0.0), marked=True)
    candidates.places[0]['fits'] = [0.72, 0.72, 0.72]
    assert candidates.winner(0.7, 0.1, start_best=True) is not None
    better = [((6.0, 0.0, 0.0), 0.79)]
    assert candidates.winner(0.7, 0.1, rivals=better, seen=STILL, start_best=True) is None
    worse = [((6.0, 0.0, 0.0), 0.66)]                  # 19 Nov outside: 74-76% vs 66-72%
    assert candidates.winner(0.7, 0.1, rivals=worse, seen=STILL, start_best=True) is not None
