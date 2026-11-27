"""The keyboard test tool's keys."""

from apnvi_stage4.keyboard_drive import next_power, next_speeds


def test_speed_keys():
    assert next_speeds('w', 0.0, 0.0) == (0.05, 0.0)
    assert next_speeds('a', 0.1, 0.0) == (0.1, 0.1)
    assert next_speeds('d', 0.1, 0.0) == (0.1, -0.1)
    assert next_speeds('x', 0.1, 0.0) == (0.05, 0.0)


def test_speed_is_capped():
    forward = 0.0
    for _ in range(30):
        forward, _turn = next_speeds('w', forward, 0.0)
    assert forward == 0.5


def test_any_other_key_stops():
    for key in (' ', 's', 'q', 'z'):
        assert next_speeds(key, 0.3, 0.5) == (0.0, 0.0)
        assert next_power(key, 50, 50) == (0, 0)


def test_power_keys():
    assert next_power('w', 0, 0) == (10, 10)
    assert next_power('a', 50, 50) == (40, 60)
    assert next_power('w', 100, 100) == (100, 100)
