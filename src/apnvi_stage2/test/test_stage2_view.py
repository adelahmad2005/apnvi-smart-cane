"""Tests for the no-ROS parts of stage2_view.py: drawing, and the browser buttons."""

import json
import os
import threading
import urllib.error
import urllib.request

from apnvi_stage2.stage2_view import (
    answer_lines, LEVEL_COLOURS, make_handler, PAGE, render, scale_box, show_box, SHOW_ANSWER_S,
    small_bgr, stage1_strip, WIDTH)
from http.server import ThreadingHTTPServer
import numpy as np
import pytest

RED = (0, 0, 255)


def photo(colour=50, width=1280, height=720):
    """Make a plain photo of one colour."""
    return np.full((height, width, 3), colour, dtype=np.uint8)


FOUND = {'question': 'where is the chair', 'found': True, 'box': [640, 360, 960, 540],
         'label': 'chair', 'seconds': 2.3, 'size': [1280, 720]}


# ---- Drawing ----

class FakeImage:
    """Stands in for a ROS Image message."""

    def __init__(self, picture, encoding, padding=0):
        self.height, self.width = picture.shape[:2]
        self.encoding, self.step = encoding, self.width * 3 + padding
        rows = [row.tobytes() + b'\0' * padding for row in picture]
        self.data = b''.join(rows)


def test_small_bgr_shrinks_and_swaps_colours():
    red_rgb = np.zeros((720, 1280, 3), np.uint8)
    red_rgb[:, :, 0] = 255                                  # red, in red-green-blue order
    small = small_bgr(FakeImage(red_rgb, 'rgb8', padding=4), 480)
    assert small.shape == (270, 480, 3) and tuple(small[10, 10]) == RED
    assert tuple(small_bgr(FakeImage(red_rgb[:, :, ::-1].copy(), 'bgr8'), 640)[5, 5]) == RED
    with pytest.raises(ValueError):
        small_bgr(FakeImage(red_rgb, 'mono8'), 480)


def test_stage1_strip_shows_level_colour_and_distance():
    strip = stage1_strip({'level': 'warning', 'distance': 0.83, 'status': 'camera_ok',
                          'fresh': True}, 640)
    assert strip.shape == (30, 640, 3)
    assert tuple(strip[2, 600]) == LEVEL_COLOURS['warning']      # coloured by the level
    quiet = stage1_strip({'level': 'warning', 'fresh': False}, 640)
    assert tuple(quiet[2, 600]) == (60, 60, 60)                 # grey: Stage 1 went quiet


def test_page_shows_both_stages():
    alone = render(photo(), FOUND, 0, 'ready', False)
    both = render(photo(), FOUND, 0, 'ready', False,
                  stage1={'level': 'stop', 'distance': 0.4, 'status': 'camera_ok', 'fresh': True})
    assert both.shape[0] == alone.shape[0] + 30
    assert tuple(both[360, 600]) == LEVEL_COLOURS['stop']       # strip right under the picture


def test_side_warning_shows_a_red_edge_and_text():
    side = {'level': 'warning', 'distance': 1.2, 'status': 'camera_ok', 'side': 'left',
            'fresh': True}
    frame = render(photo(), None, 0, 'ready', False, stage1=side)
    assert tuple(frame[100, 5]) == RED and tuple(frame[100, 630]) != RED    # left edge only
    calm = render(photo(), None, 0, 'ready', False, stage1=dict(side, side=None))
    assert tuple(calm[100, 5]) != RED
    assert not np.array_equal(stage1_strip(side, 640), stage1_strip(dict(side, side=None), 640))


def test_asking_for_a_picture_marks_the_page_as_watched(page):
    view, url = page
    view.publish_frame(b'jpeg')
    with urllib.request.urlopen(url + '/frame?after=0', timeout=5):
        pass
    assert view.last_request > 0


def test_speed_line_is_drawn():
    plain = render(photo(), None, 0, 'ready', False)
    with_speed = render(photo(), None, 0, 'ready', False, speed='camera 30/s | tracker 30/s')
    assert plain.shape == with_speed.shape and not np.array_equal(plain, with_speed)


def test_no_camera_yet_still_draws():
    assert render(None, None, 99, 'ready', False).shape[1] == WIDTH


def test_live_picture_never_freezes_and_box_is_drawn_on_it():
    frame = render(photo(50), FOUND, 1.0, 'ready', False)
    assert frame.shape[1] == WIDTH == 640
    # The box [640..960] x [360..540] at 1280 wide becomes [320..480] x [180..270] at 640.
    assert tuple(frame[180, 400]) == RED                     # top edge of the box
    assert tuple(frame[100, 100]) == (50, 50, 50)            # the live picture around it


def test_follower_box_is_drawn_instead_of_the_old_answer_box():
    following = dict(FOUND, follower=True)
    frame = render(photo(50), following, 1.0, 'ready', True, follow_box=(100, 100, 300, 300))
    assert tuple(frame[50, 100]) == RED                      # follower box: top edge at y = 50
    assert tuple(frame[180, 400]) == (50, 50, 50)            # old answer box not drawn


def test_follower_lost_means_no_box():
    lost = dict(FOUND, follower=True)
    frame = render(photo(50), lost, 1.0, 'ready', True, follow_box=None)
    assert not (frame[:360] == RED).all(axis=2).any()


def test_one_off_box_disappears_after_a_while_but_tracking_keeps_it():
    assert show_box(FOUND, 1.0, False)
    assert not show_box(FOUND, SHOW_ANSWER_S + 1, False)
    assert show_box(FOUND, SHOW_ANSWER_S + 60, True)
    assert not show_box({'found': False}, 0.0, True)


def test_scale_box():
    assert scale_box([640, 360, 960, 540], (1280, 720), (960, 540)) == (480, 270, 720, 405)


def test_answer_lines():
    assert answer_lines(None)[0].startswith('No question yet')
    assert answer_lines(FOUND, 3.2) == ('Asked: "where is the chair"', 'FOUND: chair in 2.3 s')
    assert answer_lines(dict(FOUND, follower=True))[1] == 'FOUND: chair in 2.3 s, following'
    missed = answer_lines({'question': 'where is it', 'found': False, 'seconds': 2.0,
                           'said': "I can't see it, turn slowly and try again"})
    assert missed[1].startswith('NOT FOUND in 2.0 s. Said:')
    silent = answer_lines({'question': 'where is it', 'found': False, 'tracking': True,
                           'said': "I can't see it, turn slowly and try again"})
    assert 'Said' not in silent[1]


# ---- The browser page and its buttons ----

class FakeView:
    """Stands in for the ROS node: remembers what the buttons asked for."""

    def __init__(self):
        self.lock = threading.Lock()
        self.new_frame = threading.Condition(self.lock)
        self.jpeg = None
        self.frame_no = 0
        self.map_jpeg = None
        self.map_view_text = None
        self.calls = []
        self.counts = {'camera': 0, 'sent': 0}

    def publish_frame(self, jpeg):
        with self.new_frame:
            self.jpeg, self.frame_no = jpeg, self.frame_no + 1
            self.new_frame.notify_all()

    def map_picture(self):
        return self.map_jpeg

    def map_view(self):
        return self.map_view_text

    def zoom_map(self, text):
        self.calls.append(('zoom', text))
        return 'Zoom 4.0'

    def ask(self, question):
        self.calls.append(('ask', question))
        return f'Asked once: "{question}"'

    def track(self, question):
        self.calls.append(('track', question))
        return 'Stopped' if not question else f'Tracking: "{question}"'

    def talk(self, pressed):
        self.calls.append(('talk', pressed))
        return 'Listening' if pressed else 'Sent'


@pytest.fixture
def page():
    """Start the page's web server on a free port with a fake view."""
    view = FakeView()
    server = ThreadingHTTPServer(('127.0.0.1', 0), make_handler(view))
    threading.Thread(target=server.serve_forever, daemon=True).start()
    yield view, f'http://127.0.0.1:{server.server_port}'
    server.shutdown()
    server.server_close()


def post(url, text):
    """Press a button: send text, return (status, reply)."""
    request = urllib.request.Request(url, data=text.encode())
    try:
        with urllib.request.urlopen(request, timeout=5) as response:
            return response.status, response.read().decode()
    except urllib.error.HTTPError as problem:
        return problem.code, problem.read().decode()


def test_page_has_text_box_and_buttons(page):
    _, url = page
    with urllib.request.urlopen(url + '/', timeout=5) as response:
        html = response.read()
    assert html == PAGE
    for part in (b'<input', b'Ask once', b'Track', b'Stop', b'/frame?after='):
        assert part in html


def test_buttons(page):
    view, url = page
    assert post(url + '/ask', 'where is the exit') == (200, 'Asked once: "where is the exit"')
    assert post(url + '/track', ' where is the chair ')[0] == 200
    assert post(url + '/stop', '') == (200, 'Stopped')
    assert view.calls == [('ask', 'where is the exit'), ('track', 'where is the chair'),
                          ('track', '')]


def test_empty_question_is_refused(page):
    view, url = page
    assert post(url + '/ask', '   ')[0] == 400
    assert view.calls == []


def test_result_json_has_picture_size_and_tracking_flag():
    from apnvi_stage2.asking import Outcome
    from apnvi_stage2.finder import FindResult
    from apnvi_stage2.stage2_node import result_json
    data = json.loads(result_json(Outcome('q', '', FindResult(True, (1, 2, 3, 4), 'x', 1.0),
                                          photo()), tracking=True))
    assert data['size'] == [1280, 720] and data['tracking'] is True
    two = FindResult(True, (1, 2, 3, 4), 'hand', 1.0, boxes=((1, 2, 3, 4), (5, 6, 7, 8)))
    data = json.loads(result_json(Outcome('q', '', two, photo())))
    assert data['boxes'] == [[1, 2, 3, 4], [5, 6, 7, 8]]


def test_frame_gives_the_newest_picture_and_its_number(page):
    view, url = page
    view.publish_frame(b'first')
    view.publish_frame(b'second')
    with urllib.request.urlopen(url + '/frame?after=0', timeout=5) as response:
        assert response.read() == b'second' and response.headers['X-Frame'] == '2'


def test_frame_waits_for_a_newer_picture(page):
    view, url = page
    view.publish_frame(b'old')
    threading.Timer(0.2, view.publish_frame, args=(b'new',)).start()
    with urllib.request.urlopen(url + '/frame?after=1', timeout=5) as response:
        assert response.read() == b'new'


def test_frame_before_any_picture(page):
    _, url = page
    try:
        urllib.request.urlopen(url + '/frame?after=0', timeout=5)
        assert False, 'expected an error'
    except urllib.error.HTTPError as problem:
        assert problem.code == 503


def test_answer_lines_for_a_spoken_answer():
    result = {'question': 'read the sign', 'kind': 'answer', 'said': 'Exit, straight ahead.',
              'seconds': 1.6, 'found': True, 'box': [1, 2, 3, 4], 'label': 'sign'}
    assert answer_lines(result) == ('Asked: "read the sign"',
                                    'ANSWER in 1.6 s: "Exit, straight ahead."')


def test_hold_to_speak_sends_press_and_let_go(page):
    view, url = page
    assert post(url + '/talk/start', '') == (200, 'Listening')
    assert post(url + '/talk/stop', '') == (200, 'Sent')
    assert view.calls == [('talk', True), ('talk', False)]


def test_page_has_listening_mode_next_to_ask_once():
    assert b'Listening mode' in PAGE and b'/talk/start' in PAGE and b'/talk/stop' in PAGE
    assert PAGE.index(b'Ask once') < PAGE.index(b'Listening mode') < PAGE.index(b'</form>')


def get(url):
    """Fetch a page: return (status, content type, body)."""
    try:
        with urllib.request.urlopen(url, timeout=5) as response:
            return response.status, response.headers['Content-Type'], response.read()
    except urllib.error.HTTPError as problem:
        return problem.code, problem.headers['Content-Type'], problem.read()


def test_page_has_the_map_next_to_the_camera():
    assert b'id="map"' in PAGE and b"fetch('/map'" in PAGE
    assert PAGE.index(b'id="live"') < PAGE.index(b'id="map"')       # camera first, map beside it
    assert b'flex-wrap:wrap' in PAGE                                # below the camera on a phone
    assert b'No map' in PAGE


def test_map_route_says_no_map_before_stage_3_sends_one(page):
    view, base = page
    status, _, body = get(base + '/map')
    assert status == 503 and body == b'No map'


def test_map_route_sends_the_newest_map_picture(page):
    view, base = page
    view.map_jpeg = b'\xff\xd8map'
    status, kind, body = get(base + '/map?t=1')
    assert (status, kind, body) == (200, 'image/jpeg', b'\xff\xd8map')


# ---- clickable dots and zoom on the map ----

def test_map_sends_where_its_dots_are_in_a_header(page):
    view, base = page
    view.map_jpeg = b'\xff\xd8map'
    view.map_view_text = json.dumps({'span': 6.0, 'zoom': 1.0, 'pins': [{'n': 1, 'name': 'desk'}]})
    with urllib.request.urlopen(base + '/map', timeout=5) as response:
        assert json.loads(response.headers['X-Map-View'])['pins'][0]['name'] == 'desk'
    view.map_view_text = None
    with urllib.request.urlopen(base + '/map', timeout=5) as response:
        assert response.headers['X-Map-View'] is None


def test_view_header_refuses_text_that_cannot_be_a_header():
    from apnvi_stage2.stage2_view import view_header
    assert view_header('{"a": 1}') == '{"a": 1}'
    for bad in (None, '', 'two\r\nlines', 'café', 'x' * 40000):
        assert view_header(bad) is None


def test_zoom_words():
    from apnvi_stage2.stage2_view import zoom_words
    assert json.loads(zoom_words('{"zoom": 2, "x": 1.23456, "y": -4}')) == {
        'zoom': 2.0, 'x': 1.235, 'y': -4.0}
    assert json.loads(zoom_words('{"zoom": 50}')) == {'zoom': 8.0}      # 8 times at most
    assert json.loads(zoom_words('{"zoom": 0.1}')) == {'zoom': 1.0}     # the whole map
    for bad in ('', 'zoom', '[2]', '{"x": 1}', '{"zoom": "near"}', '{"zoom": NaN}',
                '{"zoom": 2, "x": 1}', '{"zoom": 2, "x": Infinity, "y": 0}'):
        assert zoom_words(bad) is None


def test_zoom_button_is_passed_on(page):
    view, base = page
    assert post(base + '/map/zoom', '{"zoom": 4, "x": 1, "y": 2}') == (200, 'Zoom 4.0')
    assert view.calls[-1] == ('zoom', '{"zoom": 4, "x": 1, "y": 2}')


def test_page_has_clickable_dots_and_zoom():
    for part in (b'X-Map-View', b"fetch('/map/zoom'", b'Zoom in', b'Zoom out', b'Whole map',
                 b'id="pininfo"', b'id="ring"', b"'wheel'", b'id="legend"'):
        assert part in PAGE
    # 17 Nov (Adel): zoomed in, press and drag (mouse or finger) to move the view
    for part in (b"'pointerdown'", b"'pointermove'", b"'pointerup'", b'touch-action:none',
                 b'draggable="false"', b"'dragstart'", b'user-select:none'):
        assert part in PAGE


def test_an_old_tab_learns_there_is_a_newer_page(page):
    # 17 Nov (Adel): a tab open from before a restart ran the old script ("undefined")
    from apnvi_stage2.stage2_view import PAGE_VERSION
    view, base = page
    assert b'__PAGE_VERSION__' not in PAGE and b"'" + PAGE_VERSION.encode() + b"'" in PAGE
    view.map_jpeg = b'\xff\xd8map'
    with urllib.request.urlopen(base + '/map', timeout=5) as response:
        assert response.headers['X-Page'] == PAGE_VERSION
    with urllib.request.urlopen(base + '/', timeout=5) as response:
        assert response.headers['Cache-Control'] == 'no-store'
    assert PAGE.index(b'id="map"') < PAGE.index(b'Zoom in') < PAGE.index(b'id="pininfo"')


# ---- what the device is doing, in words ----

def test_state_words():
    from apnvi_stage2.stage2_view import state_words
    guiding = {'state': 'guiding', 'goal': {'name': 'the door'}, 'distance_m': 3.24,
               'route_ok': True}
    assert state_words(guiding, '') == 'Guiding to the door, 3.2 m to go'
    assert state_words(dict(guiding, route_ok=False), '').endswith('(no route right now)')
    paused = {'state': 'paused', 'goal': {'name': 'the door'}, 'paused_by': 'red'}
    assert state_words(paused, '') == 'Paused on the way to the door (red button)'
    assert state_words({'state': 'idle', 'goal': None}, 'You have arrived') == \
        'Free mode (no trip) | Said: "You have arrived"'
    assert state_words({}, '') == ''


# ---- sound through the browser: the server side ----

class SoundView:
    """Just the sound parts of Stage2View, without ROS."""

    from apnvi_stage2.stage2_view import Stage2View
    on_sound, next_sound = Stage2View.on_sound, Stage2View.next_sound
    sound_file, take_recording = Stage2View.sound_file, Stage2View.take_recording

    def __init__(self):
        from collections import deque
        self.sounds, self.sound_no = deque(maxlen=30), 0
        self.new_sound = threading.Condition()
        self.sent = []
        self.recording_pub = self

    def publish(self, msg):
        self.sent.append(msg.data)


class Line:
    """Stands in for a ROS String message."""

    def __init__(self, data):
        self.data = data


def test_a_new_browser_never_gets_old_sentences(tmp_path):
    view = SoundView()
    wav = tmp_path / 'a.wav'
    wav.write_bytes(b'RIFF' + b'\0' * 100)
    view.on_sound(Line(json.dumps({'kind': 'play', 'file': str(wav), 'volume': 0.4,
                                   'sentence': 'Listening'})))
    assert view.next_sound(-1) == {'kind': 'reset', 'seq': 1}      # just opened the page
    assert view.next_sound(1, wait_s=0.05) is None                  # nothing new
    view.on_sound(Line(json.dumps({'kind': 'stop'})))
    assert view.next_sound(1) == {'kind': 'stop', 'seq': 2}
    assert view.next_sound(99) == {'kind': 'reset', 'seq': 2}      # the node was restarted


def test_play_events_carry_the_sound_and_a_missing_file_is_dropped(tmp_path):
    view = SoundView()
    wav = tmp_path / 'b.wav'
    wav.write_bytes(b'RIFF' + b'\1' * 50)
    view.on_sound(Line(json.dumps({'kind': 'play', 'file': str(wav), 'volume': 1.0,
                                   'sentence': 'You have arrived'})))
    event = view.next_sound(0)
    assert event == {'kind': 'play', 'volume': 1.0, 'sentence': 'You have arrived', 'seq': 1}
    assert view.sound_file(1) == wav.read_bytes() and view.sound_file(7) is None
    view.on_sound(Line(json.dumps({'kind': 'play', 'file': str(tmp_path / 'gone.wav')})))
    assert view.sound_no == 1
    view.on_sound(Line('not json'))
    assert view.sound_no == 1


def test_a_browser_recording_becomes_a_file_for_stage2():
    view = SoundView()
    assert view.take_recording(b'nothing') == 'That recording was empty'
    assert view.sent == []
    words = view.take_recording(b'RIFF' + b'\0' * 40 + b'\1' * 32000)
    assert words == 'Sent (1.0 s)' and len(view.sent) == 1
    with open(view.sent[0], 'rb') as file:
        assert file.read().startswith(b'RIFF')
    os.remove(view.sent[0])
