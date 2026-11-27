"""The scan filter drops only close points behind the device (the user, the handle, the pole)."""

import math

from apnvi_stage3.scan_filter import filter_ranges, rear_mask


def test_only_close_points_behind_are_dropped():
    # 4 points at -180, -90, 0 and 90 degrees; straight back is 0 (the LiDAR's zero angle)
    mask = rear_mask(-math.pi, math.pi / 2, 4, 0.0, math.radians(60))
    assert mask == [False, False, True, False]
    ranges = [0.3, 0.3, 0.3, 0.3]
    assert filter_ranges(ranges, mask, 0.7) == [0.3, 0.3, math.inf, 0.3]


def test_far_points_behind_are_kept():
    mask = rear_mask(-0.1, 0.1, 3, 0.0, math.radians(60))
    assert filter_ranges([2.0, 0.5, 0.9], mask, 0.7) == [2.0, math.inf, 0.9]


def test_the_sector_wraps_round_at_180_degrees():
    mask = rear_mask(-math.pi, math.radians(30), 12, math.pi, math.radians(45))
    behind = [i for i, m in enumerate(mask) if m]
    assert behind == [0, 1, 11]          # -180, -150 and +150 degrees
