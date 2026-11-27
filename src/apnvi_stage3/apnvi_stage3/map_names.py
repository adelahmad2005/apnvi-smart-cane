"""
Saved maps by name, and the table of places, signs and things that is saved next to each map.

A saved map "ESB117" is two files in the maps folder: esb117.db (the RTAB-Map map) and
esb117.json (the table: places, signs, objects). Names are spoken, so "Engineering building"
and "engineering  building" are the same map.
A map that has no name yet keeps its table in work/unsaved.json; saving the map copies it.

Pins (16 Nov, Adel: the map labels itself): a thing or a sign with a name and a map point.
Things the user asked for are sure at once; things the labeller saw must be seen twice.
The same name within 1 m of a pin is one more look at that pin, not a new pin.
The entrance (17 Nov, Adel): the doorway of the room, facing in, where users always come in.
It is marked once ("save this place as entrance" standing in the doorway, or the command
set_entrance) and kept as table['entrance']; placing on the map starts its search there.
No ROS in this file, so it can be tested with no hardware.
"""

import json
import math
import os
import re
import shutil
import sqlite3
import time
import zlib

import numpy as np

SAVED_DIR = 'named'             # inside the maps folder: the saved maps
WORK_DIR = 'work'               # inside the maps folder: the copy that is running
UNSAVED_TABLE = 'unsaved.json'  # in WORK_DIR: the pins of a map that has no name yet
PIN_SAME_M = 1.0                # the same name this close to a pin is the same thing
NEAR_PIN_M = 0.5                # a clicked pin: the one of that name this close (23 Nov)
SHOWN_AFTER = 2                 # looks a labelled thing needs before it is shown and usable
# Names that mean the same thing (two looks may call one thing by two names).
SAME_NAMES = (
    ('bin', 'trash can', 'garbage can', 'waste bin', 'rubbish bin', 'trash bin', 'dustbin'),
    ('screen', 'monitor', 'display', 'tv', 'television'),
    ('whiteboard', 'white board', 'board'),
    ('sofa', 'couch'),
    ('door', 'doorway', 'exit'),
)
# A place saved under one of these names is also the map's entrance.
ENTRANCE_NAMES = ('entrance', 'the entrance', 'entrance door', 'room entrance', 'the door',
                  'door', 'doorway', 'the doorway', 'entry', 'the entry')
ROOM_CODE = re.compile(r'^[A-Za-z]{1,5}[ -]?\d{2,5}[A-Za-z]?$')      # ESB117, ESB 2090, B-204
ROOM_WORD = re.compile(r'\b(room|lab|laboratory|office|restroom|toilet|classroom|library|'
                       r'kitchen|hall|studio|workshop)\b', re.IGNORECASE)


def slug(name):
    """Turn a spoken name into a file-safe one: letters and digits, one underscore between."""
    return re.sub(r'[^a-z0-9]+', '_', str(name).lower()).strip('_')


SAME_SLUGS = tuple(frozenset(slug(name) for name in group) for group in SAME_NAMES)


def map_paths(root, name):
    """Give (database file, table file) of a saved map. Empty name: ValueError."""
    key = slug(name)
    if not key:
        raise ValueError('a map needs a name')
    folder = os.path.join(root, SAVED_DIR)
    return os.path.join(folder, key + '.db'), os.path.join(folder, key + '.json')


def work_path(root, name):
    """Give the file of the running copy of a saved map (the saved file is never changed)."""
    return os.path.join(root, WORK_DIR, slug(name) + '_work.db')


def saved_maps(root):
    """List the names (as saved) of the maps that exist."""
    folder = os.path.join(root, SAVED_DIR)
    if not os.path.isdir(folder):
        return []
    return sorted(f[:-3] for f in os.listdir(folder) if f.endswith('.db'))


def copy_file(source, target):
    """Copy a file so that nobody ever sees half of it: write beside it, then rename."""
    os.makedirs(os.path.dirname(target), exist_ok=True)
    partial = target + '.partial'
    shutil.copyfile(source, partial)
    os.replace(partial, target)


def empty_table(map_name):
    """Start the table of a map."""
    return {'map': map_name, 'places': {}, 'signs': [], 'objects': []}


def session_path(path):
    """
    Where a map's movable things live (19 Nov, Adel): chairs, bottles and bins move, so they
    are kept for this session only, in a file next to the table that a load deletes.
    """
    return (path[:-5] if path.endswith('.json') else path) + '.session.json'


def load_table(path, map_name=''):
    """Read a table and this session's movable things; a map with none gets an empty one."""
    try:
        with open(path) as handle:
            table = json.load(handle)
    except (OSError, ValueError):
        table = empty_table(map_name)
    for key, empty in (('places', {}), ('signs', []), ('objects', [])):
        table.setdefault(key, empty)
    try:
        with open(session_path(path)) as handle:
            table['objects'].extend(json.load(handle).get('objects', []))
    except (OSError, ValueError, AttributeError):
        pass
    return table


def write_json(path, data):
    """Write a file, all or nothing."""
    os.makedirs(os.path.dirname(path), exist_ok=True)
    partial = path + '.partial'
    with open(partial, 'w') as handle:
        json.dump(data, handle, indent=1)
    os.replace(partial, path)


def save_table(path, table):
    """Write a table: movable things go to the session file, everything else is kept."""
    kept = dict(table)
    kept['objects'] = [pin for pin in table['objects'] if not pin.get('movable')]
    write_json(path, kept)
    write_json(session_path(path),
               {'objects': [pin for pin in table['objects'] if pin.get('movable')]})


def forget_session(path):
    """A map is loaded or started: its movable things from before are gone."""
    try:
        os.remove(session_path(path))
    except OSError:
        pass


def add_place(table, name, x, y, yaw, when=None):
    """Pin a named place at a map position. Saving the same name again moves it."""
    table['places'][slug(name)] = {'name': name.strip(), 'x': round(x, 3), 'y': round(y, 3),
                                   'yaw': round(yaw, 3), 'time': when or time.time()}
    return table['places'][slug(name)]


def is_entrance(name):
    """Say if a place name means the room's entrance ("entrance", "the door")."""
    return slug(name) in {slug(words) for words in ENTRANCE_NAMES}


def set_entrance(table, x, y, yaw, when=None):
    """Mark the room's entrance: the device's map position in the doorway, facing in."""
    table['entrance'] = {'x': round(x, 3), 'y': round(y, 3), 'yaw': round(yaw, 3),
                         'time': when or time.time()}
    return table['entrance']


def entrance(table):
    """Give (x, y, yaw) of the marked entrance, or None."""
    door = table.get('entrance')
    try:
        return float(door['x']), float(door['y']), float(door['yaw'])
    except (KeyError, TypeError, ValueError):
        return None


def table_path(root, name):
    """Give the table file of a map: the saved map's, or the one of a map not saved yet."""
    if slug(name):
        return map_paths(root, name)[1]
    return os.path.join(root, WORK_DIR, UNSAVED_TABLE)


def same_name(one, other):
    """
    Say if two spoken names mean the same thing.

    The same words, one inside the other ("bottle" and "water bottle"), or two names from one
    line of SAME_NAMES ("bin" and "trash can").
    """
    a, b = slug(one), slug(other)
    if not a or not b:
        return False
    if a == b or a.replace('_', '') == b.replace('_', ''):     # "ESB 117" and "ESB117"
        return True
    words_a, words_b = set(a.split('_')), set(b.split('_'))
    if words_a <= words_b or words_b <= words_a:
        return True
    return any(a in group and b in group for group in SAME_SLUGS)


def is_room(text):
    """Say if a sign's words read like a room: "ESB117", "Lab 2", "Room 204", "Restroom"."""
    words = str(text).strip()
    return bool(ROOM_CODE.match(words) or ROOM_WORD.search(words))


def pin_name(pin):
    """Give what a pin is called (signs keep their words in "text")."""
    return pin.get('name') or pin.get('text') or ''


def usable(pin):
    """
    Say if a pin may be shown and used as a goal.

    A thing the user asked for is sure at once. A thing the labeller saw must be seen
    SHOWN_AFTER times (one look can be wrong). Old tables have no count: those count as sure.
    """
    return bool(pin.get('sure')) or pin.get('seen', SHOWN_AFTER) >= SHOWN_AFTER


def add_pin(table, name, x, y, from_x=None, from_y=None, kind='thing', sure=False, when=None,
            same_m=PIN_SAME_M, movable=False):
    """
    Pin a thing or a sign at a map point, or count one more look at a pin already there.

    A pin with the same name (same_name) within same_m metres is the same thing: its point
    moves to the average of all looks and its count goes up. (from_x, from_y) is where it was
    seen from: the guide stops on the line from there to the thing.
    kind: 'thing', 'sign' or 'room' (a sign that names a room). Signs go in table['signs'].
    movable: a thing people move (a chair, a bottle): kept for this session only.
    """
    name = str(name).strip()
    if not slug(name):
        return None
    if kind == 'sign' and is_room(name):
        kind = 'room'
    pins = table['signs'] if kind in ('sign', 'room') else table['objects']
    now = when or time.time()
    for pin in pins:
        if same_name(pin_name(pin), name) and \
                math.hypot(pin['x'] - x, pin['y'] - y) <= same_m:
            looks = pin.get('seen', 1)
            pin['x'] = round((pin['x'] * looks + x) / (looks + 1), 3)
            pin['y'] = round((pin['y'] * looks + y) / (looks + 1), 3)
            pin['seen'] = looks + 1
            pin['sure'] = bool(pin.get('sure')) or sure
            pin['time'] = now
            if movable and kind not in ('sign', 'room'):
                pin['movable'] = True
            if from_x is not None and from_y is not None:
                pin['from_x'], pin['from_y'] = round(from_x, 3), round(from_y, 3)
            return pin
    pin = {'name': name, 'x': round(x, 3), 'y': round(y, 3), 'seen': 1, 'sure': bool(sure),
           'kind': kind, 'time': now}
    if movable and kind not in ('sign', 'room'):
        pin['movable'] = True
    if kind in ('sign', 'room'):
        pin['text'] = name
    if from_x is not None and from_y is not None:
        pin['from_x'], pin['from_y'] = round(from_x, 3), round(from_y, 3)
    pins.append(pin)
    return pin


def all_pins(table, only_usable=True):
    """List every pin for the map picture and the tools: name, kind, x, y, seen."""
    pins = [{'name': place.get('name', key), 'kind': 'place', 'x': place['x'], 'y': place['y'],
             'seen': SHOWN_AFTER} for key, place in table['places'].items()
            if 'x' in place and 'y' in place]
    for group, kind in ((table['signs'], 'sign'), (table['objects'], 'thing')):
        for pin in group:
            if 'x' not in pin or 'y' not in pin or (only_usable and not usable(pin)):
                continue
            pins.append({'name': pin_name(pin), 'kind': pin.get('kind', kind), 'x': pin['x'],
                         'y': pin['y'], 'seen': pin.get('seen', SHOWN_AFTER)})
            if pin.get('movable'):
                pins[-1]['movable'] = True          # a different colour on the page (19 Nov)
    return pins


def nearest_pin(table, name, near, within=None):
    """
    Find the one pin called `name` nearest the map point `near` (x, y), within `within` m.

    Gives (group, key or index) or None. 23 Nov (Adel): the console clicks one pin on the map;
    removing "table" took all 60 tables off outside_2.
    """
    within = NEAR_PIN_M if within is None else within
    key, best = slug(name), None
    place = table['places'].get(key)
    if place is not None and 'x' in place:
        gap = math.hypot(place['x'] - near[0], place['y'] - near[1])
        if gap <= within:
            best = (gap, 'places', key)
    for group in ('signs', 'objects'):
        for index, pin in enumerate(table[group]):
            if slug(pin_name(pin)) != key or 'x' not in pin:
                continue
            gap = math.hypot(pin['x'] - near[0], pin['y'] - near[1])
            if gap <= within and (best is None or gap < best[0]):
                best = (gap, group, index)
    return None if best is None else best[1:]


def remove_pin(table, name, near=None):
    """
    Take the pins with this name off the map; give how many.

    By voice: every place, sign and thing with that name. With `near` (x, y), a click on the
    console: only the one of them nearest that point.
    """
    key = slug(name)
    if near is not None:
        found = nearest_pin(table, name, near)
        if found is None:
            return 0
        group, where = found
        if group == 'places':
            table['places'].pop(where)
        else:
            table[group].pop(where)
        return 1
    removed = 1 if table['places'].pop(key, None) is not None else 0
    for group in ('signs', 'objects'):
        keep = [pin for pin in table[group] if slug(pin_name(pin)) != key]
        removed += len(table[group]) - len(keep)
        table[group] = keep
    return removed


def rename_pin(table, old, new, near=None):
    """Give every pin called `old` the name `new`, or with `near` (x, y) only the nearest one."""
    key, renamed = slug(old), 0
    new = str(new).strip()
    if not slug(new):
        return 0
    if near is not None:
        found = nearest_pin(table, old, near)
        if found is None:
            return 0
        group, where = found
        if group == 'places':
            place = table['places'].pop(where)
            place['name'] = new
            table['places'][slug(new)] = place
        else:
            table[group][where]['name'] = new
            if 'text' in table[group][where]:
                table[group][where]['text'] = new
        return 1
    if key in table['places']:
        place = table['places'].pop(key)
        place['name'] = new
        table['places'][slug(new)] = place
        renamed += 1
    for group in ('signs', 'objects'):
        for pin in table[group]:
            if slug(pin_name(pin)) == key:
                pin['name'] = new
                if 'text' in pin:
                    pin['text'] = new
                renamed += 1
    return renamed


def merge_tables(target, source):
    """Copy every pin of `source` into `target` (used when a map is saved under a name)."""
    if entrance(source) is not None and entrance(target) is None:
        target['entrance'] = dict(source['entrance'])
    for key, place in source['places'].items():
        target['places'].setdefault(key, place)
    for group in ('signs', 'objects'):
        for pin in source[group]:
            if 'x' not in pin or 'y' not in pin:
                continue
            for _ in range(max(1, pin.get('seen', 1))):
                added = add_pin(target, pin_name(pin), pin['x'], pin['y'], pin.get('from_x'),
                                pin.get('from_y'), pin.get('kind', 'thing'), pin.get('sure'),
                                pin.get('time'))
            if added is not None:
                added['seen'] = max(added.get('seen', 1), pin.get('seen', 1))
    return target


def _nearest(pins, near):
    """Pick the pin nearest to the point `near` (x, y), or the first one if near is None."""
    if not pins:
        return None
    if near is None:
        return pins[0]
    return min(pins, key=lambda pin: math.hypot(pin['x'] - near[0], pin['y'] - near[1]))


def find_place(table, name, near=None):
    """
    Find a place, a sign or a thing by name, or None.

    The exact name first (a saved place, then a sign, then a thing), then a name that means
    the same (same_name). Only pins that are usable(). When several match, the one nearest
    to `near` (x, y), the device's position.
    """
    key = slug(name)
    if not key:
        return None
    if key in table['places']:
        return table['places'][key]
    # 18 Nov (lab D1): "take me to my desk" reaches here as "desk" (Stage 2 drops "my"), and a
    # desk the camera pinned won over the saved place "my desk". A saved place wins.
    for place_key, place in table['places'].items():
        if re.sub(r'^(the|a|an|my)_', '', place_key) == key:
            return place
    signs = [pin for pin in table['signs'] if usable(pin) and 'x' in pin and 'y' in pin]
    things = [pin for pin in table['objects'] if usable(pin) and 'x' in pin and 'y' in pin]
    for group in (signs, things):
        exact = _nearest([pin for pin in group if slug(pin_name(pin)).replace('_', '')
                          == key.replace('_', '')], near)
        if exact is not None:
            return exact
    for place_key, place in table['places'].items():
        if key in place_key or place_key in key:
            return place
    for sign in signs:
        text = slug(pin_name(sign))
        if text and (key in text or text in key):
            return sign
    for group in (signs, things):
        close = _nearest([pin for pin in group if same_name(pin_name(pin), name)], near)
        if close is not None:
            return close
    return None


def with_open_floor(grid, table):
    """
    Give the map picture for route planning: walls inside the table's 'open_floor' boxes wiped.

    18 Nov (lab): the saved map had lines by the entrance where the floor is open, and no route
    could reach the bin. Wiped in the map itself, placing got much worse (the LiDAR, 77 cm up,
    does see something there), so the boxes are wiped only in the picture routes are planned on.
    Each box is [x_min, x_max, y_min, y_max] in metres. grid: (cells, resolution, (x, y)).
    """
    boxes = table.get('open_floor') or []
    if grid is None or not boxes:
        return grid
    cells, resolution, (x_left, y_low) = grid
    cells = cells.copy()
    ys = y_low + (np.arange(cells.shape[0]) + 0.5) * resolution
    xs = x_left + (np.arange(cells.shape[1]) + 0.5) * resolution
    for x_min, x_max, y_min, y_max in boxes:
        rows = (ys >= y_min) & (ys <= y_max)
        columns = (xs >= x_min) & (xs <= x_max)
        inside = rows[:, None] & columns[None, :]
        cells[inside & (cells > 0)] = 0
    return cells, resolution, (x_left, y_low)


def saved_grid(database):
    """
    Read the map's wall picture out of a map file: (grid, resolution, (x, y)) or None.

    RTAB-Map stores the finished map picture in the file: cells -1 (unknown), 0 (free) or
    100 (wall), row after row from the lowest y, with the corner (x, y) in metres. The width is
    not stored, so it is the one (of the sizes that fit) where neighbouring rows match best.
    """
    try:
        connection = sqlite3.connect('file:%s?mode=ro' % database, uri=True)
        try:
            row = connection.execute('select opt_map, opt_map_x_min, opt_map_y_min, '
                                     'opt_map_resolution from Admin').fetchone()
        finally:
            connection.close()
        cells = np.frombuffer(zlib.decompress(row[0]), np.int8)
    except (sqlite3.Error, TypeError, zlib.error):
        return None                     # no finished picture in this file
    best, width = -1.0, 0
    for candidate in range(20, min(2000, cells.size // 20)):
        if cells.size % candidate:
            continue
        grid = cells.reshape(-1, candidate)
        same = float((grid[1:] == grid[:-1]).mean())
        if same > best:
            best, width = same, candidate
    if not width:
        return None
    return cells.reshape(-1, width), row[3], (row[1], row[2])


# ---- pins that move with the map (19 Nov, outside EB2) ------------------------------------
# While a map is drawn, RTAB-Map straightens it each time it recognises a place (33 times on
# one lap outside: the tracker drifts in the open). A pin kept as a fixed map point stayed where
# the drifted map had it (vending machines a quarter turn off). So a pin also keeps the path
# point (RTAB-Map node) it was seen from and where it lies from there; when the path moves, the
# pin moves with it.

PIN_MOVED_M = 0.05              # a pin is rewritten only when its path point moved this much


def to_node(node, x, y):
    """Where the map point (x, y) lies seen from a path point node = (x, y, yaw)."""
    dx, dy = x - node[0], y - node[1]
    cos, sin = math.cos(node[2]), math.sin(node[2])
    return (round(cos * dx + sin * dy, 3), round(-sin * dx + cos * dy, 3))


def from_node(node, rel):
    """The map point at rel (x, y) seen from the path point node = (x, y, yaw)."""
    cos, sin = math.cos(node[2]), math.sin(node[2])
    return (node[0] + cos * rel[0] - sin * rel[1], node[1] + sin * rel[0] + cos * rel[1])


def tie_pin(pin, node_id, node):
    """Tie a pin (and the point it was seen from) to path point node_id at node (x, y, yaw)."""
    pin['node'] = int(node_id)
    pin['rel'] = list(to_node(node, pin['x'], pin['y']))
    if 'from_x' in pin and 'from_y' in pin:
        pin['rel_from'] = list(to_node(node, pin['from_x'], pin['from_y']))


def follow_path(table, nodes, moved_m=PIN_MOVED_M):
    """
    Move each tied pin with its path point. nodes: {id: (x, y, yaw)} on the map now.
    Returns how many pins moved.
    """
    moved = 0
    for group in ('signs', 'objects'):
        for pin in table[group]:
            node = nodes.get(pin.get('node'))
            if node is None or 'rel' not in pin:
                continue
            x, y = from_node(node, pin['rel'])
            if math.hypot(x - pin['x'], y - pin['y']) < moved_m:
                continue
            pin['x'], pin['y'] = round(x, 3), round(y, 3)
            if 'rel_from' in pin:
                pin['from_x'], pin['from_y'] = (round(v, 3) for v in from_node(node,
                                                                               pin['rel_from']))
            moved += 1
    return moved
