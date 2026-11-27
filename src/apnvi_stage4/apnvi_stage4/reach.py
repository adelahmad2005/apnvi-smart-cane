"""
Move a trip's stop point out of a spot the device cannot reach (18 Nov, lab: "take me to the bin").

The bin stands in a corner by the door; the stop point 0.8 m in front of it was squeezed between
the bin, the wall and the desks, where the planner counts the device as touching something. No
route could end there, so the trip never started. Now, if the stop point costs too much on the
planner's map, the nearest point within max_m that costs little is used instead.

Costs are the planner's map as Nav2 sends it (/global_costmap/costmap): 0 free to 100 touching,
-1 not seen.

23 Nov (Adel, a trip to the "Push bar to open" sign on the door): the nearest free point jumped
every second as the map changed, about 40 times in 80 s, once to the far side of the door, where
no route could reach it. Each new point is a new route, so the device kept turning on the spot
and said "Turn me a little toward open floor" with the way open. Now (with `near`, the device's
position) a point with a wall between it and the device is skipped, and fits_point lets the
guide keep a moved point while the device still fits there.
"""

import numpy as np

OK_COST = 50            # below this the device fits there with room to spare
MAX_MOVE_M = 1.5        # never move the stop point further than this
WALL_COST = 100         # a cell this costly is a thing itself (a wall, a closed door)
SIDE_CHECK_M = 1.0      # how far towards the device a moved point is checked for a wall
MAX_TRIED = 400         # free cells tried, nearest first, for one on the device's side


def fits_point(costs, width, height, resolution, origin_x, origin_y, x, y, ok_cost=OK_COST):
    """Say if the device fits at (x, y): a seen cell that costs little. Off the map: True."""
    column = int((x - origin_x) / resolution)
    row = int((y - origin_y) / resolution)
    if not (0 <= column < width and 0 <= row < height):
        return True
    cost = int(costs[row * width + column])
    return 0 <= cost < ok_cost


def wall_between(grid, resolution, origin_x, origin_y, start, toward, check_m=SIDE_CHECK_M):
    """Say if a wall cell lies on the first check_m of the line from start towards `toward`."""
    gap = float(np.hypot(toward[0] - start[0], toward[1] - start[1]))
    if gap < 1e-6:
        return False
    length = min(gap, check_m)
    steps = max(1, int(length / (resolution / 2.0)))
    height, width = grid.shape
    for step in range(1, steps + 1):
        part = length * step / steps / gap
        column = int((start[0] + (toward[0] - start[0]) * part - origin_x) / resolution)
        row = int((start[1] + (toward[1] - start[1]) * part - origin_y) / resolution)
        if 0 <= column < width and 0 <= row < height and grid[row, column] >= WALL_COST:
            return True
    return False


def reachable_point(costs, width, height, resolution, origin_x, origin_y, x, y,
                    ok_cost=OK_COST, max_m=MAX_MOVE_M, near=None):
    """
    Give (x, y): the stop point itself if the device fits there, else the nearest that fits.

    costs: the map's cells, row after row from the lowest y. Returns None if no point within
    max_m fits, and the point unchanged if it is off the map. near: the device's (x, y); when
    given, the nearest free point with no wall between it and the device is used (the plain
    nearest if every one has a wall).
    """
    grid = np.asarray(costs, dtype=np.int16).reshape(height, width)
    column = int((x - origin_x) / resolution)
    row = int((y - origin_y) / resolution)
    if not (0 <= column < width and 0 <= row < height):
        return (x, y)
    if 0 <= grid[row, column] < ok_cost:
        return (x, y)
    reach = int(max_m / resolution)
    top, left = max(0, row - reach), max(0, column - reach)
    window = grid[top:row + reach + 1, left:column + reach + 1]
    rows, columns = np.nonzero((window >= 0) & (window < ok_cost))
    if rows.size == 0:
        return None
    gaps = (rows + top - row) ** 2 + (columns + left - column) ** 2
    order = [int(index) for index in np.argsort(gaps, kind='stable')
             if gaps[index] <= reach * reach]
    if not order:
        return None

    def point(index):
        return (origin_x + (columns[index] + left + 0.5) * resolution,
                origin_y + (rows[index] + top + 0.5) * resolution)

    if near is not None:
        for index in order[:MAX_TRIED]:
            if not wall_between(grid, resolution, origin_x, origin_y, point(index), near):
                return point(index)
    return point(order[0])
