"""
Turn what the user asked for into a goal on the map. No ROS, so it can be tested anywhere.

A goal can be:
  - a map point, typed as two numbers "x y" (metres in the frame map), e.g. "2.5 -1.0"
  - a saved place, a sign or a thing, looked up by name in the running map's table (Stage 3's
    map_names.py: places saved with "save this place as ...", and the pins the device made
    itself: signs, room names, things). "the restroom" and "restroom" are the same.

A thing sits on a desk or against a wall, where no route can end (16 Nov). So the goal for a
thing or a sign is a point STAND_OFF_M in front of it, on the line from where it was seen
(approach_point). The thing's own point is kept in the goal, so on arrival the guide can say
where it is.
"""

import math
import os
import re

from apnvi_stage3.map_names import find_place, load_table, pin_name, table_path

from .guide import Goal

STAND_OFF_M = 0.8               # stop this far in front of a thing or a sign

FILLER = re.compile(r'^(the|a|an|my)\s+', re.IGNORECASE)


def clean_name(text):
    """Drop "the" and end marks: 'the Restroom.' -> 'Restroom'."""
    text = re.sub(r'[.?!,]+$', '', str(text).strip())
    return FILLER.sub('', text).strip()


def map_point(text):
    """Read a map point: '2.5 -1' or '2.5, -1' -> (2.5, -1.0); anything else -> None."""
    parts = re.split(r'[\s,]+', str(text).strip())
    if len(parts) != 2:
        return None
    try:
        return float(parts[0]), float(parts[1])
    except ValueError:
        return None


def approach_point(thing, seen_from, stand_off_m=STAND_OFF_M):
    """
    Give the point to stop at: stand_off_m before the thing, on the line from seen_from.

    If it was seen from closer than that, the spot it was seen from is the goal.
    """
    gap = math.hypot(thing[0] - seen_from[0], thing[1] - seen_from[1])
    if gap <= stand_off_m:
        return seen_from
    share = (gap - stand_off_m) / gap
    return (seen_from[0] + (thing[0] - seen_from[0]) * share,
            seen_from[1] + (thing[1] - seen_from[1]) * share)


def thing_goal(name, x, y, from_x=None, from_y=None, near=None, stand_off_m=STAND_OFF_M):
    """
    Make the Goal for a thing or a sign at (x, y).

    It was seen from (from_x, from_y); if that is not known, the device's own position
    `near` is used; with neither, the goal is the thing's point itself.
    """
    seen_from = (from_x, from_y) if from_x is not None and from_y is not None else near
    if seen_from is None:
        return Goal(float(x), float(y), name)
    stop_x, stop_y = approach_point((float(x), float(y)),
                                    (float(seen_from[0]), float(seen_from[1])), stand_off_m)
    return Goal(stop_x, stop_y, name, thing=(float(x), float(y)))


def find_goal(text, maps_dir, map_name, near=None, stand_off_m=STAND_OFF_M):
    """
    Return a Goal for this text, or None if it is not a point and not on the map.

    maps_dir: Stage 3's maps folder (~/.ros/apnvi_maps). map_name: the running map ('' when
    a new map is being drawn: its pins are in the table of the map with no name yet).
    near: the device's (x, y), to pick the nearest of several things with the same name.
    """
    point = map_point(text)
    if point is not None:
        return Goal(point[0], point[1], 'the point')
    name = clean_name(text)
    if not name:
        return None
    path = table_path(maps_dir, map_name)
    if not os.path.exists(path):
        return None
    place = find_place(load_table(path, map_name), name, near)
    if not place or 'x' not in place or 'y' not in place:
        return None
    said = pin_name(place) or name
    if 'yaw' in place:                          # a saved place: the spot itself
        return Goal(float(place['x']), float(place['y']), said)
    kind = place.get('kind', 'sign' if 'text' in place else 'thing')
    if kind == 'room':
        pass                                    # a room's name is said as it is: "ESB117"
    elif kind == 'sign':
        said = 'the %s sign' % said
    elif not said.lower().startswith('the '):
        said = 'the ' + said
    return thing_goal(said, place['x'], place['y'], place.get('from_x'), place.get('from_y'),
                      near, stand_off_m)
