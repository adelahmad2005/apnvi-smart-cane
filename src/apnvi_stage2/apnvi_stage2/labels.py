"""
The labeller's thinking: what to ask Qwen, when to look, and which answers to keep.

Plain Python, nothing from ROS, so it can be tested on its own. labeler_node.py uses it.
Why (Adel, 15 Nov): the map labels itself. As the device moves, the AI looks at the camera
picture, reads signs and room names and names landmarks; each is placed on the map
(locate.py) and pinned (apnvi_stage3/map_names.py), so "take me to the door" works later,
also after the saved map is loaded again.
"""

import math
import re

from .finder import read_items, to_pixels

LABEL_PROMPT = (
    'This photo is from a device that helps a blind person find their way indoors. '
    'List the landmarks in it. '
    'Signs: every sign, door plate or room number whose words you can read; label = the exact '
    'words, kind = "sign". '
    'Things: every clear thing such as a door, vending machine, whiteboard, desk, table, chair, '
    'sofa, bench, pillar, stairs, elevator, counter, notice board, printer, bin, screen, '
    'cupboard, shelf, sink, water dispenser, fire extinguisher, plant or bottle; '
    'label = one or two plain words; kind = "movable" if people often move it (a chair, '
    'bottle, bin, bag, cup, laptop, box, stool), else kind = "fixed" (a door, vending '
    'machine, whiteboard, desk, pillar, stairs, elevator, cupboard, shelf, sink, water '
    'dispenser, printer). '
    'Do not list people, walls, the floor, the ceiling or lights. At most 8 items, the '
    'clearest first. '
    'Report in JSON format as [{{"bbox_2d": [x1, y1, x2, y2], "label": "short name", '
    '"kind": "sign"}}, ...].'
    '{question}'
)
# 19 Nov (Adel: in front of a whiteboard, no pin): the prompt ended "If there is nothing to list,
# reply with [] and nothing else", and the small Qwen took that way out on most looks (a clear
# photo of a whiteboard, desks and chairs: "[]"). Without it the same photo lists 8 things.

LOOK_MOVED_M = 0.5              # look again after moving this far
LOOK_TURNED_RAD = math.radians(20)      # or turning this much
LOOK_GAP_S = 2.0                # never more often than this
MAX_ITEMS = 8
MAX_WORDS = 4                   # longer "names" are sentences, not names
MAX_BOX_SHARE = 0.6             # a box over this share of the picture is a wall, not a thing
MIN_BOX_PX = 12
NOT_LANDMARKS = {'person', 'people', 'man', 'woman', 'boy', 'girl', 'hand', 'arm', 'wall',
                 'floor', 'ceiling', 'light', 'lights', 'lamp', 'shadow', 'object', 'thing',
                 'room', 'corridor', 'window', 'carpet', 'cable', 'wire', 'paper'}
UNREAD_SIGNS = {'sign', 'signs', 'a sign', 'the sign', 'door sign', 'door plate', 'plate',
                'name plate', 'nameplate', 'room number', 'room sign', 'number', 'label',
                'notice', 'poster', 'plaque', 'text', 'writing', 'unreadable'}   # no words read
ARTICLE = re.compile(r'^(the|a|an)\s+', re.IGNORECASE)


def clean_label(label, kind):
    """
    Tidy a name from Qwen, or '' if it is not worth pinning.

    Things: lower case, no "the", at most MAX_WORDS words, not in NOT_LANDMARKS.
    Signs keep their words as read (capitals matter on a door plate), trimmed.
    """
    words = ' '.join(str(label).replace('\n', ' ').split()).strip(' .,:;"\'')
    if not words or len(words) > 40:
        return ''
    if kind == 'sign':
        # 17 Nov (Adel): a sign must be pinned by what it says ("ESB117"), not as "sign"
        if words.lower() in UNREAD_SIGNS:
            return ''
        return words if re.search(r'[A-Za-z0-9]', words) else ''
    words = ARTICLE.sub('', words.lower())
    if (not words or len(words.split()) > MAX_WORDS or words in NOT_LANDMARKS
            or words in UNREAD_SIGNS):         # 19 Nov: "sign" as a thing says nothing either
        return ''
    return words if re.fullmatch(r"[a-z0-9 '\-]+", words) else ''


def read_labels(answer, width, height):
    """
    Read Qwen's list: [(label, kind, box in pixels)], or None if it cannot be understood.

    Drops what is not a landmark, boxes that are tiny or fill the picture, and a second item
    with the same name and nearly the same box.
    """
    items = read_items(answer)
    if items is None:
        return None
    kept = []
    for item in items[:MAX_ITEMS]:
        numbers = item.get('bbox_2d')
        if not isinstance(numbers, (list, tuple)) or len(numbers) != 4:
            continue
        try:
            scaled = tuple(float(n) for n in numbers)
        except (TypeError, ValueError):
            continue
        said = str(item.get('kind', '')).strip().lower()
        # 19 Nov (Adel): movable things (chairs, bottles) are pinned for this session only
        kind = 'sign' if said == 'sign' else 'movable' if said == 'movable' else 'thing'
        label = clean_label(item.get('label', ''), kind)
        if not label:
            continue
        box = to_pixels(scaled + (label,), width, height)
        wide, tall = box[2] - box[0], box[3] - box[1]
        if wide < MIN_BOX_PX or tall < MIN_BOX_PX:
            continue
        if wide * tall > MAX_BOX_SHARE * width * height:
            continue
        if any(label == other and abs(box[0] - old[0]) < wide / 2 for other, _k, old in kept):
            continue
        kept.append((label, kind, box))
    return kept


def should_look(pose, last_pose, now, last_time, scanning=False, moved_m=LOOK_MOVED_M,
                turned_rad=LOOK_TURNED_RAD, gap_s=LOOK_GAP_S):
    """
    Say if the labeller should look now.

    pose, last_pose: (x, y, yaw) now and at the last look (None: never looked).
    Never within gap_s of the last look. While scanning ("scan the room") every gap_s;
    otherwise only after the device moved moved_m or turned turned_rad.
    """
    if pose is None or now - last_time < gap_s:
        return False
    if scanning or last_pose is None:
        return True
    turn = abs(math.atan2(math.sin(pose[2] - last_pose[2]), math.cos(pose[2] - last_pose[2])))
    return math.hypot(pose[0] - last_pose[0], pose[1] - last_pose[1]) >= moved_m \
        or turn >= turned_rad


STILL_M = 0.1                   # moving less than this...
STILL_RAD = math.radians(5)     # ...and turning less than this counts as standing still
STILL_AFTER_S = 1.0             # standing still this long: look (a sharp picture)
STILL_LOOKS = 2                 # this many looks per stop: seen twice, so the pin is shown


def still_look(pose, still_pose, still_since, looks_here, now, after_s=STILL_AFTER_S,
               most=STILL_LOOKS, still_m=STILL_M, still_rad=STILL_RAD):
    """
    Look when the user stops (19 Nov, Adel: stopped 5 s at the ESB117 sign, no pin).

    The labeller looked only after moving 0.5 m or turning 20 degrees, so a stop in front of a
    sign got no new look, and the one look was taken while moving (the sign read "ESBME").
    pose, still_pose: (x, y, yaw) now and where this stop began (None: not known).
    Returns (look_now, still_pose, still_since, looks_here) with the stop's state updated; the
    caller adds 1 to looks_here when it really looks.
    """
    if pose is None:
        return False, None, now, 0
    if still_pose is None:
        return False, pose, now, 0
    turn = abs(math.atan2(math.sin(pose[2] - still_pose[2]), math.cos(pose[2] - still_pose[2])))
    if math.hypot(pose[0] - still_pose[0], pose[1] - still_pose[1]) >= still_m or turn >= still_rad:
        return False, pose, now, 0                  # moved: a new stop may begin here
    return (now - still_since >= after_s and looks_here < most,
            still_pose, still_since, looks_here)


def scan_summary(before, after, most=5):
    """Make the sentence said after "scan the room": how many new names, and up to five."""
    new = [name for name in after if name not in set(before)]
    if not new:
        return 'I found nothing new'
    listed = ', '.join(new[:most])
    more = '' if len(new) <= most else ' and %d more' % (len(new) - most)
    return 'I found %d thing%s: %s%s' % (len(new), '' if len(new) == 1 else 's', listed, more)
