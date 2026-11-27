"""When a camera guess places the device."""

import math

from apnvi_stage3.camera_guess import anchors_agree, CameraGuesses


def test_a_guess_the_walls_fit_places_at_once():
    guesses = CameraGuesses()
    assert guesses.add((1.0, 2.0, 0.0), 0.8, 0.0) == 'walls fit'


def test_one_guess_in_open_space_waits_for_a_second():
    guesses = CameraGuesses()
    assert guesses.add((1.0, 2.0, 0.0), 0.4, 0.0) is None


def test_two_agreeing_guesses_place():
    guesses = CameraGuesses()
    guesses.add((1.0, 2.0, 0.0), 0.4, 0.0)
    assert guesses.add((1.2, 2.1, math.radians(5)), 0.35, 10.0) == 'seen twice'


def test_two_guesses_that_disagree_do_not_place():
    guesses = CameraGuesses()
    guesses.add((1.0, 2.0, 0.0), 0.4, 0.0)
    assert guesses.add((4.0, 2.0, 0.0), 0.4, 10.0) is None          # 3 m apart
    assert guesses.add((4.1, 2.0, 0.0), 0.4, 12.0) == 'seen twice'  # the newer one, twice


def test_walls_that_object_refuse_the_guess():
    guesses = CameraGuesses()
    guesses.add((1.0, 2.0, 0.0), 0.1, 0.0)
    assert guesses.add((1.0, 2.0, 0.0), 0.1, 5.0) is None
    assert 'refused' in guesses.note


def test_an_old_guess_is_forgotten():
    guesses = CameraGuesses()
    guesses.add((1.0, 2.0, 0.0), 0.4, 0.0)
    assert guesses.add((1.0, 2.0, 0.0), 0.4, 100.0) is None


def test_anchors_agree_across_the_half_turn():
    assert anchors_agree((0.0, 0.0, math.radians(179)), (0.0, 0.0, math.radians(-179)))
    assert not anchors_agree((0.0, 0.0, 0.0), (0.0, 0.0, math.radians(30)))


def test_without_a_camera_the_lidar_places_alone():
    assert CameraGuesses().lets_lidar_place((1.0, 2.0, 0.0), 0.0)


def test_with_the_camera_working_the_lidar_needs_its_agreement():
    guesses = CameraGuesses()
    guesses.heard((1.0, 2.0, 0.0), 0.0)
    assert guesses.lets_lidar_place((1.2, 2.0, 0.0), 5.0)
    assert not guesses.lets_lidar_place((4.0, -1.0, 0.0), 5.0)      # 84% there, but wrong


def test_a_quiet_camera_hands_back_to_the_lidar():
    guesses = CameraGuesses()
    guesses.heard((1.0, 2.0, 0.0), 0.0)
    assert guesses.lets_lidar_place((4.0, -1.0, 0.0), 100.0)


def test_after_a_load_the_lidar_waits_for_the_camera():
    guesses = CameraGuesses()
    guesses.heard((1.0, 2.0, 0.0), 0.0)
    guesses.forget(new_map=True)
    assert not guesses.lets_lidar_place((1.0, 2.0, 0.0), 10.0)
