"""Tests for follower.py with a made-up moving object (no camera, no ROS)."""

import time

from apnvi_stage2.follower import (Follower, FollowerWorker, iou, make_tracker, shrink, spread,
                                   TRACK_WIDTH)
import cv2
import numpy as np
import pytest

needs_tracker = pytest.mark.skipif(make_tracker() is None,
                                   reason='this OpenCV has no tracker')
needs_kcf = pytest.mark.skipif(not hasattr(cv2, 'TrackerKCF_create'),
                               reason='only KCF (on the Jetson) reports a lost object')


def scene(x, width=1280, height=720):
    """Make a textured background with a textured square object whose left edge is at x."""
    rng = np.random.default_rng(0)
    picture = rng.integers(60, 120, (height, width, 3), dtype=np.uint8)
    patch = np.random.default_rng(1).integers(0, 255, (160, 160, 3), dtype=np.uint8)
    cv2.rectangle(patch, (0, 0), (159, 159), (0, 255, 255), 6)
    picture[280:440, x:x + 160] = patch
    return picture


def test_shrink_keeps_shape():
    assert shrink(scene(100)).shape == (270, TRACK_WIDTH, 3)
    small = np.zeros((100, 200, 3), np.uint8)
    assert shrink(small) is small


def test_no_pictures_yet_cannot_start():
    assert not Follower().start((0, 0, 10, 10), (1280, 720))


def test_box_too_small_is_refused():
    follower = Follower()
    follower.add_frame(0.0, scene(100))
    assert not follower.start((100, 280, 102, 282), (1280, 720))


@needs_tracker
def test_follows_a_moving_object():
    follower = Follower()
    follower.add_frame(0.0, scene(100))
    assert follower.start((100, 280, 260, 440), (1280, 720), stamp=0.0)
    for step in range(1, 21):                       # the object moves 10 px per picture
        follower.add_frame(step * 0.1, scene(100 + 10 * step))
    left, top, right, bottom = follower.box_in(1280, 720)
    assert abs(left - 300) < 30 and abs(top - 280) < 30    # it is now at x = 300


@needs_tracker
def test_catches_up_from_an_old_answer():
    """The answer is about an older picture; the box must end up where the object is now."""
    follower = Follower()
    for step in range(0, 21):
        follower.add_frame(step * 0.1, scene(100 + 10 * step))
    # The answer describes the picture at time 0.0, when the object was at x = 100.
    assert follower.start((100, 280, 260, 440), (1280, 720), stamp=0.0)
    left, _, _, _ = follower.box_in(1280, 720)
    assert abs(left - 300) < 30


@needs_kcf
def test_object_gone_means_box_hidden():
    follower = Follower()
    follower.add_frame(0.0, scene(100))
    follower.start((100, 280, 260, 440), (1280, 720), stamp=0.0)
    blank = np.zeros((720, 1280, 3), np.uint8)
    for step in range(1, 10):
        follower.add_frame(step * 0.1, blank)
    assert follower.box_in(1280, 720) is None and follower.lost


def test_old_pictures_are_dropped():
    follower = Follower(history_s=1.0)
    for step in range(30):
        follower.add_frame(step * 0.1, np.zeros((72, 128, 3), np.uint8))
    assert 10 <= len(follower.frames) <= 11          # about 1 s of pictures at 10 a second


def test_no_tracker_available_means_no_follower():
    follower = Follower(tracker_factory=lambda: None)
    follower.add_frame(0.0, scene(100))
    assert not follower.start((100, 280, 260, 440), (1280, 720))
    assert follower.box_in(1280, 720) is None


def two_hands(x, width=1280, height=720):
    """Two identical textured squares: one moving (left edge at x), one still at x = 900."""
    picture = scene(x, width, height)
    patch = picture[280:440, x:x + 160].copy()
    picture[280:440, 900:1060] = patch
    return picture


def test_spread():
    assert spread([1, 2, 3], 5) == [1, 2, 3]
    picked = spread(list(range(100)), 10)
    assert len(picked) == 10 and picked[-1] == 99 and picked == sorted(picked)


@needs_tracker
def test_two_matches_keeps_the_one_being_followed():
    """Qwen names the other hand first; the box must stay on the hand it was on."""
    follower = Follower()
    for step in range(0, 11):
        follower.add_frame(step * 0.1, two_hands(100 + 10 * step))
    assert follower.start((100, 280, 260, 440), (1280, 720), stamp=0.0)
    other, ours = (900, 280, 1060, 440), (150, 280, 310, 440)     # at 0.5 s
    for _ in range(3):
        assert follower.correct(other, (1280, 720), stamp=0.5, boxes=[other, ours])
        left, _, _, _ = follower.box_in(1280, 720)
        assert abs(left - 200) < 30                 # still on the moving one, now at x = 200


@needs_tracker
def test_restart_catch_up_is_capped():
    """However old the answer, a restart only steps through a few pictures."""
    calls = []

    def counting_tracker():
        tracker = make_tracker()

        class Counting:
            def init(self, *args):
                return tracker.init(*args)

            def update(self, picture):
                calls.append(1)
                return tracker.update(picture)
        return Counting()

    follower = Follower(tracker_factory=counting_tracker)
    for step in range(0, 61):
        follower.add_frame(step * 0.1, scene(100 + 2 * step))
    assert follower.start((100, 280, 260, 440), (1280, 720), stamp=0.0)
    assert len(calls) <= 12


def test_iou():
    assert iou((0, 0, 10, 10), (0, 0, 10, 10)) == 1.0
    assert iou((0, 0, 10, 10), (20, 20, 30, 30)) == 0.0
    assert abs(iou((0, 0, 10, 10), (5, 0, 15, 10)) - 1 / 3) < 1e-9


@needs_tracker
def test_answer_that_agrees_nudges_without_restarting():
    follower = Follower()
    for step in range(0, 11):
        follower.add_frame(step * 0.1, scene(100 + 10 * step))
    assert follower.start((100, 280, 260, 440), (1280, 720), stamp=0.0)
    # A later answer about the picture at 0.5 s (object at x = 150), slightly off.
    assert follower.correct((156, 280, 316, 440), (1280, 720), stamp=0.5)
    left, _, _, _ = follower.box_in(1280, 720)
    assert abs(left - 200) < 30                     # still where the object is now (x = 200)


@needs_tracker
def test_answer_that_disagrees_restarts_on_it():
    follower = Follower()
    for step in range(0, 11):
        follower.add_frame(step * 0.1, scene(100 + 10 * step))
    follower.start((100, 280, 260, 440), (1280, 720), stamp=0.0)
    # The answer says the object is somewhere else entirely: believe the answer.
    assert follower.correct((800, 100, 960, 260), (1280, 720), stamp=1.0)
    left, top, _, _ = follower.box_in(1280, 720)
    assert left > 600 and top < 200


def test_correct_with_nothing_followed_starts():
    follower = Follower(tracker_factory=lambda: None)
    follower.add_frame(0.0, scene(100))
    assert not follower.correct((100, 280, 260, 440), (1280, 720))


def wait_until(condition, timeout_s=5.0):
    end = time.monotonic() + timeout_s
    while time.monotonic() < end:
        if condition():
            return True
        time.sleep(0.01)
    return False


@needs_tracker
def test_worker_follows_in_its_own_thread():
    worker = FollowerWorker()
    try:
        worker.submit_frame(0.0, scene(100))
        worker.submit_answer((100, 280, 260, 440), (1280, 720), 0.0)
        assert wait_until(lambda: worker.following)
        for step in range(1, 11):
            worker.submit_frame(step * 0.1, scene(100 + 10 * step))
            time.sleep(0.05)                        # like a camera: one picture at a time
        assert wait_until(lambda: abs(worker.box_in(1280, 720)[0] - 200) < 30)
        worker.stop_following()
        assert wait_until(lambda: worker.box_in(1280, 720) is None)
    finally:
        worker.close()


def test_worker_closes():
    worker = FollowerWorker()
    worker.close()
    assert not worker.thread.is_alive()


@needs_tracker
def test_fast_move_is_followed():
    """The object crosses half the picture in half a second (1200 pixels a second)."""
    follower = Follower()
    follower.add_frame(0.0, scene(100))
    assert follower.start((100, 280, 260, 440), (1280, 720), stamp=0.0)
    for step in range(1, 16):
        follower.add_frame(step / 30, scene(100 + 40 * step))
    left, top, _, _ = follower.box_in(1280, 720)
    assert abs(left - 700) < 40 and abs(top - 280) < 30
