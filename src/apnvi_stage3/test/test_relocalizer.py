"""Tests for relocalizer.py: finding a sweep on a saved map. No ROS needed."""

import math

import numpy as np

from apnvi_stage3.relocalizer import combine, Relocalizer

RES = 0.05
ORIGIN = (-1.0, -1.0)


def lab():
    """Make an 8 x 5 m room with a pillar and a cupboard, so no two ends look alike."""
    grid = np.full((140, 200), -1, np.int8)
    grid[20:120, 20:180] = 0                      # floor from (0, 0) to (8, 5)
    grid[20, 20:180] = grid[119, 20:180] = 100
    grid[20:120, 20] = grid[20:120, 179] = 100
    grid[60:70, 60:70] = 100                      # a pillar at about (2.5, 2.0)
    grid[20:50, 140:150] = 100                    # a cupboard at the right end
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


def test_finds_the_pose_from_anywhere():
    grid = lab()
    finder = Relocalizer(grid, RES, ORIGIN)
    for x, y, yaw in ((1.0, 1.0, 0.3), (6.5, 3.5, -2.0), (4.0, 2.5, 2.5)):
        points = finder.sweep_points(*sweep(grid, x, y, yaw))
        fit = finder.find(points)
        assert math.hypot(fit.x - x, fit.y - y) < 0.15, fit
        assert abs(math.atan2(math.sin(fit.yaw - yaw), math.cos(fit.yaw - yaw))) < math.radians(4)
        assert fit.score > 0.9


def test_a_wrong_pose_scores_low():
    grid = lab()
    finder = Relocalizer(grid, RES, ORIGIN)
    points = finder.sweep_points(*sweep(grid, 6.5, 3.5, -2.0))
    assert finder.score(points, 6.5, 3.5, -2.0)[0] > 0.9
    assert finder.score(points, 1.0, 1.0, 0.3)[0] < 0.5


def test_empty_map_gives_no_answer():
    finder = Relocalizer(np.full((50, 50), -1, np.int8), RES, ORIGIN)
    assert finder.find(np.zeros((2, 100))) is None


def test_frame_offset_puts_the_lidar_where_it_was_found():
    from apnvi_stage3.relocalizer import combine, frame_offset
    in_frame = (1.2, -0.4, 0.7)                 # where RTAB-Map thinks the LiDAR is
    on_map = (-0.56, 3.55, math.radians(161))   # where the search found it
    offset = frame_offset(on_map, in_frame)
    landed = combine(offset, in_frame)
    assert all(abs(a - b) < 1e-9 for a, b in zip(landed[:2], on_map[:2]))
    assert abs(math.atan2(math.sin(landed[2] - on_map[2]), math.cos(landed[2] - on_map[2]))) < 1e-9
    assert frame_offset(in_frame, in_frame) == (0.0, 0.0, 0.0)


def mirror_room():
    """Make a 20 x 4 m room whose ends look alike, with a pillar halfway (out of reach)."""
    grid = np.full((120, 440), -1, np.int8)
    grid[20:100, 20:420] = 0                      # floor from (0, 0) to (20, 4)
    grid[20, 20:420] = grid[99, 20:420] = 100
    grid[20:100, 20] = grid[20:100, 419] = 100
    grid[70:90, 200:240] = 100                    # the pillar: x 9 to 11, y 2.5 to 3.5
    return grid


def test_candidates_follow_both_places_until_one_wins():
    from apnvi_stage3.relocalizer import Candidates
    grid = mirror_room()
    finder = Relocalizer(grid, RES, ORIGIN)
    candidates = Candidates()
    seen = (0.0, 0.0, 0.0)                        # RTAB-Map's own frame: the device starts at 0
    true_x, true_y = 1.5, 1.0                     # facing +x, at one end...
    points = finder.sweep_points(*sweep(grid, true_x, true_y, 0.0))
    near_end = finder.score(points, true_x, true_y, 0.0)[0]
    far_end = finder.score(points, 20 - true_x, 4 - true_y, math.pi)[0]
    assert abs(near_end - far_end) < 0.05         # ...and one sweep fits both ends
    candidates.add_answers([(true_x, true_y, 0.0, near_end),
                            (20 - true_x, 4 - true_y, math.pi, far_end)], seen, 0.6)
    assert len(candidates.places) == 2
    assert candidates.winner(0.65, 0.08) is None
    for step in range(1, 9):                       # pushed 1 m at a time, to beside the pillar
        seen = (float(step), 0.0, 0.0)             # (16 Nov: a close thing that is not on the
        points = finder.sweep_points(*sweep(grid, true_x + step, true_y, 0.0))     # map costs
        candidates.update(finder, points, seen)    # little, so the far end loses only when its
    shift, _why = candidates.winner(0.65, 0.08)    # beams would have to cross the pillar)
    where = combine(shift, seen)
    assert math.hypot(where[0] - (true_x + 8), where[1] - true_y) < 0.15


def test_start_spot_no_longer_breaks_a_tie():
    # 17 Nov: started beside the taped spot, a 60% fit there won a tie and placed the device
    # wrong. Now the start spot must win clearly, like any other place.
    from apnvi_stage3.relocalizer import Candidates
    candidates = Candidates()
    candidates.add((0.0, 0.0, 0.0), 0.8, start=True)
    candidates.add((5.0, 0.0, math.pi), 0.8)
    for place in candidates.places:
        place['fits'] = [0.8, 0.8, 0.8]
    assert candidates.winner(0.7, 0.1) is None
    candidates.places[1]['fits'] = [0.6, 0.6, 0.6]      # the other place stops fitting
    assert candidates.winner(0.7, 0.1)[0] == (0.0, 0.0, 0.0)
    candidates.places[0]['fits'] = [0.6, 0.6, 0.6]      # a weak fit never wins
    assert candidates.winner(0.7, 0.1) is None


def test_an_upside_down_lidar_is_found_once_its_sweep_is_unmirrored():
    # 15 Nov: on the device the LiDAR is upside down; its beam at angle a sees what a LiDAR the
    # right way up sees at -a. Read as it is, the room comes out mirrored and fits somewhere else.
    from types import SimpleNamespace
    from apnvi_stage3.map_manager_node import sweep_angles
    grid = lab()
    finder = Relocalizer(grid, RES, ORIGIN)
    x, y, yaw = 6.5, 3.5, -2.0
    upright, _, step = sweep(grid, x, y, yaw)
    beams = len(upright)
    flipped = [upright[(-i) % beams] for i in range(beams)]
    scan = SimpleNamespace(ranges=flipped, angle_min=0.0, angle_increment=step)
    fit = finder.find(finder.sweep_points(*sweep_angles(scan, True)))
    assert math.hypot(fit.x - x, fit.y - y) < 0.15, fit
    assert abs(math.atan2(math.sin(fit.yaw - yaw), math.cos(fit.yaw - yaw))) < math.radians(4)
    wrong = finder.sweep_points(*sweep_angles(scan, False))
    assert finder.score(wrong, x, y, yaw)[0] < 0.5          # mirrored: does not fit the truth


def test_close_things_not_on_the_map_do_not_sink_the_fit():
    # 16 Nov: from between the desks more than half the sweep is furniture 1.5 to 2.5 m away
    # that the saved map does not hold. Counting each as a full miss, the right spot scored
    # about 40% and the device was never placed.
    grid = lab()
    finder = Relocalizer(grid, RES, ORIGIN)
    x, y, yaw = 4.0, 2.5, 0.3
    ranges, first, step = sweep(grid, x, y, yaw)
    cluttered = [min(r, 1.6) if (k // 3) % 2 else r for k, r in enumerate(ranges)]
    points = finder.sweep_points(cluttered, first, step)
    assert finder.score(points, x, y, yaw)[0] > 0.7             # half the sweep is a chair
    assert finder.score(points[:2], x, y, yaw)[0] < 0.6         # the old way: all full misses
    assert finder.score(points, 1.0, 1.0, 0.3)[0] < 0.5         # a wrong spot still scores low


def test_start_spot_is_kept_and_wins_when_the_device_is_brought_to_it():
    # 16 Nov: the start spot was dropped after three poor fits; "take me to the start spot of
    # the map" could then never place the device.
    from apnvi_stage3.relocalizer import Candidates
    grid = lab()
    finder = Relocalizer(grid, RES, (ORIGIN[0] - 4.0, ORIGIN[1] - 2.5))   # start spot: mid-room
    laser = (-0.7, 0.0, math.pi)                    # the LiDAR on the device

    def view(device):
        lx, ly, lyaw = combine(device, laser)
        return finder.sweep_points(*sweep(grid, lx + 4.0, ly + 2.5, lyaw))

    def between(a, b):
        dx, dy, cos, sin = b[0] - a[0], b[1] - a[1], math.cos(a[2]), math.sin(a[2])
        return (cos * dx + sin * dy, -sin * dx + cos * dy, b[2] - a[2])

    candidates = Candidates()
    candidates.add_start()
    first, now = (3.0, 1.8, 1.0), (0.2, 0.1, 0.1)   # started elsewhere; then near the start
    for _ in range(4):
        candidates.update(finder, view(first), laser, at_start=laser)
    assert [place['start'] for place in candidates.places] == [True]      # poor fit, kept
    assert candidates.winner(0.55, 0.08, start_wins=True) is None
    seen = combine(between(first, now), laser)      # the tracker followed the device there
    for _ in range(3):
        candidates.update(finder, view(now), seen, at_start=laser)
    shift, _why = candidates.winner(0.55, 0.08, start_wins=True)
    where, truth = combine(shift, seen), combine(now, laser)
    assert math.hypot(where[0] - truth[0], where[1] - truth[1]) < 0.15


def test_inverse_undoes_a_pose():
    from apnvi_stage3.relocalizer import inverse
    pose = (1.2, -0.4, 0.7)
    for back in (combine(pose, inverse(pose)), combine(inverse(pose), pose)):
        assert all(abs(value) < 1e-9 for value in back)


def test_a_marked_entrance_wins_when_it_fits_best():
    from apnvi_stage3.relocalizer import Candidates
    places = Candidates()
    places.add_start((0.0, 0.0, 0.0))
    places.places[0]['fits'] = [0.74, 0.75, 0.76]          # outside EB2, 19 Nov
    places.places.append({'shift': (3.0, 1.0, 0.0), 'fits': [0.66, 0.70, 0.72], 'start': False})
    assert places.winner(0.7, 0.1) is None                  # no 10% lead: the old rule waits
    assert places.winner(0.7, 0.1, start_best=True) is not None


def test_a_marked_entrance_that_fits_worse_does_not_win():
    from apnvi_stage3.relocalizer import Candidates
    places = Candidates()
    places.add_start((0.0, 0.0, 0.0))
    places.places[0]['fits'] = [0.71, 0.72, 0.71]
    places.places.append({'shift': (3.0, 1.0, 0.0), 'fits': [0.75, 0.76, 0.77], 'start': False})
    assert places.winner(0.7, 0.1, start_best=True) is None
