"""
Where is the thing on the map? From Qwen's box, the depth picture and the device's position.

Plain Python and numpy, nothing from ROS, so it can be tested on its own.
Why (Adel, 15 Nov): "take me to the water bottle" needs a map point. Stage 2 used to find a
thing only as a box on the colour picture.

How it works:
  1. The box is in the colour picture. The depth picture comes from another lens with a wider
     view, so the box is moved over by angle: each edge's angle from the colour lens is the
     same angle from the depth lens (the two lenses sit 1.5 cm apart, which does not matter
     beyond half a metre).
  2. In the middle part of the box, the depth readings are sorted and a low-middle one is used
     (NEAR_SHARE): a thing fills most of its box, and what shows through behind it is further
     away, so the nearer readings are the thing.
  3. Distance and angle give the thing's place in front of the camera; the camera sits
     camera_back_m behind the wheels' centre (base_link); the device's place on the map
     (x, y and the way it faces) gives the map point.
The guide stops in front of the thing, on the line from where it was seen (see
apnvi_stage4/places.py), so the place it was seen from is kept too.
"""

from dataclasses import dataclass
import math

import numpy as np

COLOUR_FOV_DEG = 69.0           # RealSense D435 colour lens, side to side
DEPTH_FOV_DEG = 87.0            # its depth lenses, side to side
CAMERA_BACK_M = 0.70            # camera behind the wheels' centre (13 Nov; launch camera_x)
MIN_RANGE_M = 0.3               # the D435 reads nothing closer than this
MAX_RANGE_M = 5.0               # further than this the depth is too rough to pin a thing
MIDDLE_SHARE = 0.6              # use this share of the box, around its centre
NEAR_SHARE = 0.35               # of the sorted readings, take the one this far up
MIN_VALID_SHARE = 0.15          # at least this share of the readings must be real
MIN_PIXELS = 12                 # and at least this many of them

NO_DEPTH = 'no_depth'           # nothing readable in the box (shiny, too close, too small)
TOO_FAR = 'too_far'
NO_POSITION = 'no_position'     # the device does not know where it is on the map


@dataclass
class Lens:
    """One lens: picture size and where a straight-ahead ray lands, from the camera info."""

    width: int
    height: int
    fx: float
    fy: float
    cx: float
    cy: float


def lens_from_fov(width, height, fov_deg):
    """Make a Lens from the picture size and the side-to-side view angle (no camera info)."""
    fx = (width / 2.0) / math.tan(math.radians(fov_deg) / 2.0)
    return Lens(int(width), int(height), fx, fx, width / 2.0, height / 2.0)


def lens_from_info(k, width, height):
    """Make a Lens from a camera info message's K (9 numbers); None if it is empty."""
    try:
        fx, fy, cx, cy = float(k[0]), float(k[4]), float(k[2]), float(k[5])
    except (TypeError, IndexError, ValueError):
        return None
    if fx <= 0 or fy <= 0 or not width or not height:
        return None
    return Lens(int(width), int(height), fx, fy, cx, cy)


def box_on_depth(box, colour, depth):
    """Move a box (left, top, right, bottom) from the colour picture to the depth picture."""
    left, top, right, bottom = box

    def across(u):
        return depth.cx + depth.fx * (u - colour.cx) / colour.fx

    def down(v):
        return depth.cy + depth.fy * (v - colour.cy) / colour.fy

    return across(left), down(top), across(right), down(bottom)


def box_distance(depth_mm, box):
    """
    Give the thing's distance in metres from the depth picture (millimetres), or None.

    box is in depth pixels. Only the middle of the box is read; 0 means "no reading".
    """
    rows, cols = depth_mm.shape[:2]
    left, top, right, bottom = box
    mid_x, mid_y = (left + right) / 2.0, (top + bottom) / 2.0
    half_w = max(2.0, (right - left) * MIDDLE_SHARE / 2.0)
    half_h = max(2.0, (bottom - top) * MIDDLE_SHARE / 2.0)
    x1, x2 = int(max(0, mid_x - half_w)), int(min(cols, mid_x + half_w + 1))
    y1, y2 = int(max(0, mid_y - half_h)), int(min(rows, mid_y + half_h + 1))
    if x2 <= x1 or y2 <= y1:
        return None
    patch = np.asarray(depth_mm[y1:y2, x1:x2], dtype=np.float32).ravel()
    real = patch[patch > MIN_RANGE_M * 1000.0]
    if real.size < MIN_PIXELS or real.size < MIN_VALID_SHARE * patch.size:
        return None
    return float(np.quantile(real, NEAR_SHARE)) / 1000.0


def in_front(box, distance_m, colour):
    """Give (forward, left) in metres from the camera for a box in the colour picture."""
    mid = (box[0] + box[2]) / 2.0
    return distance_m, -(mid - colour.cx) / colour.fx * distance_m


def on_map(forward, left, pose, camera_back_m=CAMERA_BACK_M):
    """Turn (forward, left) from the camera into a map point; pose is (x, y, yaw)."""
    x, y, yaw = pose
    ahead = forward - camera_back_m             # from the wheels' centre
    return (x + ahead * math.cos(yaw) - left * math.sin(yaw),
            y + ahead * math.sin(yaw) + left * math.cos(yaw))


@dataclass
class Located:
    """Where a thing is: found with a map point, or the reason it could not be placed."""

    found: bool
    x: float = 0.0
    y: float = 0.0
    from_x: float = 0.0             # where the device stood (the guide comes back this way)
    from_y: float = 0.0
    distance_m: float = 0.0         # from the camera
    left_m: float = 0.0             # + = to the left of straight ahead
    problem: str = ''


def locate(box, picture_size, depth_mm, pose, colour=None, depth=None,
           camera_back_m=CAMERA_BACK_M, max_range_m=MAX_RANGE_M):
    """
    Place the thing in `box` (colour pixels) on the map.

    picture_size: (width, height) of the colour picture the box belongs to.
    depth_mm: the depth picture in millimetres. pose: the device's (x, y, yaw) on the map
    when the pictures were taken, or None. colour, depth: the two Lens objects (from the
    camera info); left out, the D435's usual view angles are used.
    """
    if depth_mm is None:
        return Located(False, problem=NO_DEPTH)
    if colour is None:
        colour = lens_from_fov(picture_size[0], picture_size[1], COLOUR_FOV_DEG)
    elif (colour.width, colour.height) != tuple(picture_size):     # the box's picture was resized
        sx, sy = picture_size[0] / colour.width, picture_size[1] / colour.height
        colour = Lens(picture_size[0], picture_size[1], colour.fx * sx, colour.fy * sy,
                      colour.cx * sx, colour.cy * sy)
    rows, cols = depth_mm.shape[:2]
    if depth is None or (depth.width, depth.height) != (cols, rows):
        depth = lens_from_fov(cols, rows, DEPTH_FOV_DEG)
    distance = box_distance(depth_mm, box_on_depth(box, colour, depth))
    if distance is None:
        return Located(False, problem=NO_DEPTH)
    forward, left = in_front(box, distance, colour)
    if distance > max_range_m:
        return Located(False, distance_m=distance, left_m=left, problem=TOO_FAR)
    if pose is None:
        return Located(False, distance_m=distance, left_m=left, problem=NO_POSITION)
    x, y = on_map(forward, left, pose, camera_back_m)
    return Located(True, x, y, pose[0], pose[1], distance, left)


def same_spot(one, other, moved_m=0.15, turned_rad=math.radians(5)):
    """Say if two poses (x, y, yaw) are the same spot: the pictures can then be used together."""
    if one is None or other is None:
        return False
    turn = abs(math.atan2(math.sin(one[2] - other[2]), math.cos(one[2] - other[2])))
    return math.hypot(one[0] - other[0], one[1] - other[1]) <= moved_m and turn <= turned_rad
