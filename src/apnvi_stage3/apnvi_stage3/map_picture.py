"""
Draw the map as a picture for the 8081 page: floor, walls, unknown, and the device.

Light is free floor, dark is a wall or object, mid grey is not seen yet. The device is an
orange dot with an arrow for the way it faces. The map is always SIZE x SIZE pixels,
cropped to the part of the map that has been seen (plus a margin), so it grows as the map does.
A faint grid line every metre and a scale bar show real distances.
A legend strip, LEGEND_H pixels high, sits under it.
Zoom (17 Nov): the page can ask for a closer view (zoom 2, 4 or 8 round a point);
the map is then drawn again with fewer metres across, so dots that sit close get room.
Pins are coloured dots only, with no words (Adel, 17 Nov): the page names a dot when tapped.
No ROS in this file: it only turns numbers into a picture, so it can be tested with no hardware.
"""

import math

import cv2
import numpy as np

SIZE = 480                      # width and height of the picture, in pixels
MARGIN_M = 1.0                  # empty border around what has been seen, in metres
MIN_SPAN_M = 6.0                # never zoom in closer than this many metres across
OCCUPIED_ABOVE = 50             # a cell this likely a wall (0 to 100) is drawn as a wall
UNKNOWN = (90, 90, 90)          # colours are blue, green, red
FREE = (235, 235, 235)
WALL = (30, 30, 30)
DEVICE = (0, 130, 255)          # orange, like Stage 1's "warning"
OUTLINE = (255, 255, 255)
LEGEND_H = 80                   # height of the legend strip under the map (scale bar: row 3)
DOT_PX = 7                      # radius of the device's dot
ARROW_PX = 30                   # length of its arrow
GRID_M = 1.0                    # a faint grid line every metre
GRID_LINE = (150, 150, 150)     # grid lines on the free floor (a bit darker than FREE)
GRID_ON_UNKNOWN = (110, 110, 110)
BAR_PX = 6                      # thickness of the scale bar
PIN_COLOURS = {'place': (200, 90, 0), 'sign': (160, 0, 160), 'room': (160, 0, 160),
               'thing': (0, 140, 0),        # blue places, purple signs and rooms, green things
               'movable': (0, 200, 240)}    # yellow: movable things, this session only (19 Nov)
PIN_PX = 5                      # radius of a pin's dot
MAX_PIN_CHARS = 40              # longer names are cut
PIN_MERGE_PX = 14               # the same name twice this close (pixels) is drawn once
MAX_ZOOM = 8                    # the closest view is the whole map's width / 8...
MIN_ZOOM_SPAN_M = 2.0           # ...but never less than this many metres across


def view_box(grid, resolution, origin, pose):
    """
    Pick the square of the map to show: (x_left, y_top, metres_across).

    It holds every cell that has been seen, and the device, plus a margin.
    grid: rows of cells, row 0 at the lowest y; -1 unknown, 0 free, up to 100 wall.
    origin: (x, y) in metres of the corner of cell [0][0]. pose: (x, y, yaw) or None.
    """
    seen_rows, seen_cols = np.nonzero(grid >= 0)
    if seen_rows.size:
        x_min = origin[0] + seen_cols.min() * resolution
        x_max = origin[0] + (seen_cols.max() + 1) * resolution
        y_min = origin[1] + seen_rows.min() * resolution
        y_max = origin[1] + (seen_rows.max() + 1) * resolution
    elif pose is not None:
        x_min = x_max = pose[0]
        y_min = y_max = pose[1]
    else:
        x_min = x_max = origin[0]
        y_min = y_max = origin[1]
    if pose is not None:
        x_min, x_max = min(x_min, pose[0]), max(x_max, pose[0])
        y_min, y_max = min(y_min, pose[1]), max(y_max, pose[1])
    span = max(x_max - x_min, y_max - y_min) + 2 * MARGIN_M
    span = max(span, MIN_SPAN_M)
    x_left = (x_min + x_max) / 2 - span / 2
    y_top = (y_min + y_max) / 2 + span / 2
    return x_left, y_top, span


def zoom_box(box, zoom=1.0, centre=None):
    """
    Give the square to draw: zoom times fewer metres across than box, round centre (x, y).

    No centre: the middle of box (zoom 1 and no centre is box itself). The page's drag moves
    the centre (17 Nov, Adel: drag the map like any web map, at any zoom); it is kept inside
    box, so the edge of the map can come to the middle of the picture but no further.
    """
    x_left, y_top, span = box
    zoom = min(max(float(zoom), 1.0), MAX_ZOOM)
    small = min(span, max(span / zoom, MIN_ZOOM_SPAN_M))
    if centre is None:
        if small >= span:
            return box
        centre = (x_left + span / 2, y_top - span / 2)
    middle_x = min(max(centre[0], x_left), x_left + span)
    middle_y = min(max(centre[1], y_top - span), y_top)
    return middle_x - small / 2, middle_y + small / 2, small


def to_pixel(x, y, box):
    """Where a map point (metres) lands in the picture, as (column, row)."""
    x_left, y_top, span = box
    scale = SIZE / span
    return (x - x_left) * scale, (y_top - y) * scale


def to_map(column, row, box):
    """Give the map point (metres) under a pixel: the way back, for clicks in step 7."""
    x_left, y_top, span = box
    scale = SIZE / span
    return x_left + column / scale, y_top - row / scale


def draw_grid(picture, box):
    """Draw a thin line at every whole metre, so distances can be read off the map."""
    x_left, y_top, span = box
    scale = SIZE / span
    for k in range(math.ceil(x_left / GRID_M), math.floor((x_left + span) / GRID_M) + 1):
        column = round((k * GRID_M - x_left) * scale)
        if 0 <= column < SIZE:
            _tint(picture[:, column])
    for k in range(math.ceil((y_top - span) / GRID_M), math.floor(y_top / GRID_M) + 1):
        row = round((y_top - k * GRID_M) * scale)
        if 0 <= row < SIZE:
            _tint(picture[row, :])


def _tint(line):
    """Darken the free and unknown pixels of one grid line; leave walls as they are."""
    free = np.all(line == FREE, axis=1)
    unknown = np.all(line == UNKNOWN, axis=1)
    line[free] = GRID_LINE
    line[unknown] = GRID_ON_UNKNOWN


def draw_scale_bar(picture, box, left=12, bottom=SIZE - 14):
    """
    Draw a bar 1 m long (5 m when zoomed far out) with its length beside it.

    19 Nov (Adel): it is drawn in the legend, not on the map, so it stays still when the page
    turns the map.
    """
    span = box[2]
    length_m = 1.0 if span <= 12 else 5.0
    length_px = round(length_m * SIZE / span)
    cv2.rectangle(picture, (left, bottom - BAR_PX), (left + length_px, bottom), OUTLINE, -1)
    cv2.putText(picture, '%d m' % length_m, (left + length_px + 6, bottom),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, OUTLINE, 1, cv2.LINE_AA)


def legend(box=None):
    """Make the strip that says what each colour on the map means, and the scale bar."""
    strip = np.full((LEGEND_H, SIZE, 3), (40, 40, 40), np.uint8)
    items = (('Floor seen', FREE), ('Wall or object', WALL), ('Not seen yet', UNKNOWN),
             ('Device (arrow: facing)', DEVICE), ('Saved place', PIN_COLOURS['place']),
             ('Sign or room', PIN_COLOURS['sign']), ('Fixed thing', PIN_COLOURS['thing']),
             ('Movable thing', PIN_COLOURS['movable']))
    for number, (words, colour) in enumerate(items):
        left, top = 6 + (number % 4) * 119, 8 + (number // 4) * 24
        cv2.rectangle(strip, (left, top), (left + 14, top + 14), OUTLINE, -1)
        cv2.rectangle(strip, (left + 1, top + 1), (left + 13, top + 13), colour, -1)
        cv2.putText(strip, words, (left + 18, top + 12), cv2.FONT_HERSHEY_SIMPLEX, 0.3,
                    (230, 230, 230), 1, cv2.LINE_AA)
    if box is not None:
        draw_scale_bar(strip, box, left=8, bottom=LEGEND_H - 8)
    return strip


def draw_pins(picture, box, pins):
    """
    Draw each pin as a coloured dot, with no words; return the pins drawn.

    17 Nov (Adel): names, then numbers, next to the dots were unreadable where a desk, a
    screen, a chair and a sign sit within a metre. Now the map shows only the dots, and the
    page says what a dot is when it is tapped. The same name twice within PIN_MERGE_PX is one.
    Each pin returned is {'name', 'kind', 'col', 'row', 'x', 'y'}: where its dot is in the
    picture (pixels) and on the map (metres), so the page can tell a tap on it.
    """
    shown = []
    for pin in pins:
        column, row = to_pixel(pin['x'], pin['y'], box)
        if not (0 <= column < SIZE and 0 <= row < SIZE):
            continue
        centre = (round(column), round(row))
        name = str(pin.get('name', ''))[:MAX_PIN_CHARS]
        if any(name == other['name'] and abs(centre[0] - other['col'])
               + abs(centre[1] - other['row']) < PIN_MERGE_PX for other in shown):
            continue
        kind = 'movable' if pin.get('movable') else pin.get('kind')
        shown.append({'name': name, 'kind': kind, 'col': centre[0],
                      'row': centre[1], 'x': round(float(pin['x']), 2),
                      'y': round(float(pin['y']), 2)})
    for pin in sorted(shown, key=lambda pin: pin['kind'] not in ('thing', 'movable')):
        colour = PIN_COLOURS.get(pin['kind'], PIN_COLOURS['thing'])
        cv2.circle(picture, (pin['col'], pin['row']), PIN_PX + 1, OUTLINE, -1, cv2.LINE_AA)
        cv2.circle(picture, (pin['col'], pin['row']), PIN_PX, colour, -1, cv2.LINE_AA)
    return shown


def draw_map(grid, resolution, origin, pose=None, pins=(), zoom=1.0, centre=None):
    """
    Return (picture, box, shown): the picture, the box it shows, and the pins drawn.

    The picture is the map with the legend under it.
    pins: the map's named places, signs and things, each {'name', 'kind', 'x', 'y'}.
    zoom, centre: draw closer in, round the map point centre (x, y); see zoom_box.
    """
    box = zoom_box(view_box(grid, resolution, origin, pose), zoom, centre)
    x_left, y_top, span = box
    scale = SIZE / span
    rows, cols = grid.shape
    cells = np.full((rows, cols, 3), UNKNOWN, np.uint8)
    cells[(grid >= 0) & (grid <= OCCUPIED_ABOVE)] = FREE
    cells[grid > OCCUPIED_ABOVE] = WALL
    cells = cells[::-1]                         # row 0 of a picture is the top: highest y
    cell = resolution * scale                   # one map cell, in pixels
    # OpenCV counts a pixel by its centre, so move half a cell, less half a pixel.
    shift = np.float32([[cell, 0, (origin[0] - x_left) * scale + (cell - 1) / 2],
                        [0, cell, (y_top - origin[1] - rows * resolution) * scale
                         + (cell - 1) / 2]])
    picture = cv2.warpAffine(cells, shift, (SIZE, SIZE), flags=cv2.INTER_NEAREST,
                             borderValue=UNKNOWN)
    draw_grid(picture, box)
    listed = draw_pins(picture, box, pins)
    if pose is not None:
        column, row = to_pixel(pose[0], pose[1], box)
        tip = (column + ARROW_PX * math.cos(pose[2]), row - ARROW_PX * math.sin(pose[2]))
        start, end = (round(column), round(row)), (round(tip[0]), round(tip[1]))
        cv2.arrowedLine(picture, start, end, OUTLINE, 5, cv2.LINE_AA, tipLength=0.4)
        cv2.arrowedLine(picture, start, end, DEVICE, 3, cv2.LINE_AA, tipLength=0.4)
        cv2.circle(picture, start, DOT_PX + 2, OUTLINE, -1, cv2.LINE_AA)
        cv2.circle(picture, start, DOT_PX, DEVICE, -1, cv2.LINE_AA)
    return np.vstack([picture, legend(box)]), box, listed
