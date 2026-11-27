"""Tests for map_view_node.py's small helpers: the zoom the page asks for, the view it gets."""

import json

from apnvi_stage3.map_view_node import read_zoom, view_words


def test_read_zoom():
    assert read_zoom('{"zoom": 4, "x": 1.5, "y": -2}') == (4.0, (1.5, -2.0))
    assert read_zoom('{"zoom": 2}') == (2.0, None)
    assert read_zoom('{"zoom": 99, "x": 0, "y": 0}') == (8.0, (0.0, 0.0))   # 8 at most
    assert read_zoom('{"zoom": 1}') == (1.0, None)                          # the whole map
    assert read_zoom('{"zoom": 1, "x": 3, "y": 3}') == (1.0, (3.0, 3.0))    # dragged
    for bad in ('', 'nonsense', '[]', '{"zoom": "big"}', '{"zoom": NaN}',
                '{"zoom": 2, "x": 1}'):
        assert read_zoom(bad) == (1.0, None)
    assert read_zoom('{"zoom": 2, "x": NaN, "y": 0}') == (2.0, None)


def test_view_words():
    pins = [{'name': 'desk', 'kind': 'thing', 'col': 10, 'row': 20, 'x': 1.0, 'y': 2.0}]
    view = json.loads(view_words((-1.0, 5.0, 6.0), 2.0, pins))
    assert view['span'] == 6.0 and view['size'] == 480 and view['zoom'] == 2.0
    assert view['pins'] == pins
