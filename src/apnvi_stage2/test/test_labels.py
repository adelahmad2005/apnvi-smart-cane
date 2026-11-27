"""Tests for labels.py: what the labeller keeps from Qwen's list, and when it looks."""

import math

from apnvi_stage2.finder import make_prompt
from apnvi_stage2.labels import (
    clean_label, LABEL_PROMPT, read_labels, scan_summary, should_look)

ANSWER = '''```json
[{"bbox_2d": [100, 200, 300, 600], "label": "The Door", "kind": "thing"},
 {"bbox_2d": [500, 100, 620, 160], "label": "ESB117", "kind": "sign"},
 {"bbox_2d": [0, 0, 1000, 1000], "label": "wall", "kind": "thing"},
 {"bbox_2d": [400, 400, 600, 800], "label": "person", "kind": "thing"},
 {"bbox_2d": [700, 700, 702, 702], "label": "bottle", "kind": "thing"},
 {"bbox_2d": [650, 500, 800, 900], "label": "a very long description of a chair here"},
 {"bbox_2d": [110, 210, 300, 600], "label": "door", "kind": "thing"},
 {"label": "no box"}]
```'''


def test_keeps_landmarks_and_signs_only():
    kept = read_labels(ANSWER, 1280, 720)
    assert [(label, kind) for label, kind, _box in kept] == [('door', 'thing'), ('ESB117', 'sign')]
    assert kept[0][2] == (128, 144, 384, 431)


def test_empty_list_and_nonsense():
    assert read_labels('[]', 1280, 720) == []
    assert read_labels('I see a room.', 1280, 720) is None
    assert read_labels('{"bbox_2d": [10, 10, 200, 200], "label": "bin"}', 640, 480)[0][0] == 'bin'


def test_clean_label():
    assert clean_label(' The  Water bottle. ', 'thing') == 'water bottle'
    assert clean_label('Fire Exit', 'sign') == 'Fire Exit'
    assert clean_label('floor', 'thing') == '' and clean_label('---', 'sign') == ''
    assert clean_label('', 'thing') == '' and clean_label('x' * 50, 'sign') == ''
    # 17 Nov (Adel): a sign is pinned by what it says, never as just "sign"
    assert clean_label('sign', 'sign') == '' and clean_label('Door plate', 'sign') == ''
    assert clean_label('ESB117', 'sign') == 'ESB117'


def test_looks_after_moving_or_turning_but_never_too_often():
    here = (1.0, 1.0, 0.0)
    assert should_look(here, None, 10.0, 0.0)
    assert not should_look(here, here, 10.0, 0.0)
    assert should_look((1.6, 1.0, 0.0), here, 10.0, 0.0)
    assert should_look((1.0, 1.0, math.radians(25)), here, 10.0, 0.0)
    assert not should_look((1.6, 1.0, 0.0), here, 1.0, 0.0)          # 1 s after the last look
    assert should_look(here, here, 10.0, 0.0, scanning=True)
    assert not should_look(None, here, 10.0, 0.0, scanning=True)


def test_scan_summary():
    assert scan_summary(['door'], ['door']) == 'I found nothing new'
    assert scan_summary([], ['door']) == 'I found 1 thing: door'
    words = scan_summary(['door'], ['door', 'a', 'b', 'c', 'd', 'e', 'f', 'g'])
    assert words == 'I found 7 things: a, b, c, d, e and 2 more'


def test_the_prompt_survives_being_filled_in():
    prompt = make_prompt('', template=LABEL_PROMPT)
    assert '"bbox_2d": [x1, y1, x2, y2]' in prompt and '{' in prompt and 'question' not in prompt


def test_a_stop_gets_two_looks_then_none():
    from apnvi_stage2.labels import still_look
    state = (None, 0.0, 0)
    pose = (1.0, 2.0, 0.0)
    looks = []
    for tick in range(12):                      # 6 s standing still, a tick every 0.5 s
        now = tick * 0.5
        look, *state = still_look(pose, *state, now)
        if look:
            state[2] += 1
            looks.append(now)
    assert len(looks) == 2 and looks[0] >= 1.0


def test_walking_never_counts_as_a_stop():
    from apnvi_stage2.labels import still_look
    state = (None, 0.0, 0)
    for tick in range(12):
        look, *state = still_look((tick * 0.2, 0.0, 0.0), *state, tick * 0.5)
        assert not look


def test_moving_on_starts_a_new_stop():
    from apnvi_stage2.labels import still_look
    state = (None, 0.0, 0)
    look, *state = still_look((0.0, 0.0, 0.0), *state, 0.0)
    look, *state = still_look((0.0, 0.0, 0.0), *state, 1.5)
    assert look
    state[2] += 1
    look, *state = still_look((1.0, 0.0, 0.0), *state, 2.0)     # walked on
    assert not look and state[2] == 0
