"""Health: each failure is said once, each recovery once, and "Ready" once."""

from apnvi_stage4.health import Health, READY


def up(health, now, parts=('warnings', 'camera')):
    for name in parts:
        health.ok(name, now)


def test_ready_is_said_once_when_the_warnings_and_the_camera_work():
    health = Health(0.0)
    assert health.step(1.0) == []
    health.ok('warnings', 2.0)
    assert health.step(2.0) == []
    health.ok('camera', 3.0)
    assert health.step(3.0) == [READY]
    up(health, 4.0)
    assert health.step(4.0) == []


def test_says_what_is_missing_if_it_never_gets_ready():
    health = Health(0.0)
    health.ok('warnings', 59.0)
    assert health.step(59.0) == []
    health.ok('warnings', 61.0)
    assert health.step(61.0) == ['Not ready: the camera is not working']
    health.ok('warnings', 62.0)
    assert health.step(62.0) == []


def test_a_heartbeat_part_that_goes_quiet_is_said_once_and_again_when_back():
    health = Health(0.0)
    up(health, 1.0)
    health.ok('lidar', 1.0)
    health.step(1.0)
    up(health, 4.0)                                         # the LiDAR has been quiet 3 s
    assert health.step(4.0) == ['The laser scanner has stopped']
    up(health, 5.0)
    assert health.step(5.0) == []
    health.ok('lidar', 6.0)
    up(health, 6.0)
    assert health.step(6.0) == ['The laser scanner is working again']


def test_a_reporting_part_must_be_bad_for_a_moment():
    health = Health(0.0)
    up(health, 1.0)
    health.ok('handle', 1.0)
    health.step(1.0)
    health.bad('handle', 2.0)
    up(health, 3.0)
    assert health.step(3.0) == []                           # a short radio gap: nothing said
    up(health, 5.5)
    assert health.step(5.5) == ['The handle is not connected']
    health.ok('handle', 6.0)
    up(health, 6.0)
    assert health.step(6.0) == ['The handle is connected again']


def test_a_part_never_seen_working_is_never_announced():
    health = Health(0.0)
    up(health, 1.0)
    health.step(1.0)
    health.bad('wheels', 2.0)                               # drive:=false: never connected
    up(health, 30.0)
    assert health.step(30.0) == []
    assert health.summary(30.0)['wheels'] is None and health.summary(30.0)['camera'] is True


def test_the_warnings_stopping_tells_the_user_to_stand_still():
    health = Health(0.0)
    up(health, 1.0)
    health.step(1.0)
    health.ok('camera', 5.0)
    assert health.step(5.0) == ['The obstacle warning has stopped. Stand still']
