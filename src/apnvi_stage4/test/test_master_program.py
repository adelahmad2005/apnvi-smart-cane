"""
Checks on the master ESP32 program's text: the decided stop rules and settings are there.

It is C++ for the ESP32, so it cannot run here; it is compiled with arduino-cli. These
checks catch a setting changed by mistake.
"""

import os
import re

HERE = os.path.dirname(os.path.realpath(__file__))
INO = os.path.join(HERE, '..', '..', '..', 'firmware', 'esp32_master', 'esp32_master.ino')


def source():
    with open(INO) as handle:
        return handle.read()


def setting(name):
    match = re.search(r'\b%s\s*=\s*([^;]+);' % name, source())
    assert match, name
    return match.group(1).strip()


def test_stop_times_are_the_decided_ones():
    assert setting('HANDLE_QUIET_MS') == '300'          # handle quiet 0.3 s
    assert setting('JETSON_QUIET_MS') == '500'          # Jetson quiet 0.5 s


def test_every_stop_reason_is_reported():
    text = source()
    names = re.search(r'REASON_NAMES\[N_REASONS\]\s*=\s*\{([^}]*)\}', text).group(1)
    for reason in ('red', 'green', 'handle', 'jetson', 'rear'):
        assert '"%s"' % reason in names
    assert '"stop" : "clear"' in text


def test_rear_check_is_off_and_its_pins_are_settings():
    assert setting('REAR_CHECK') == 'false'
    assert setting('REAR_TRIG').isdigit() and setting('REAR_ECHO').isdigit()
    assert setting('REAR_MAX_M') == '1.0'


def test_drive_and_vibration_commands_exist():
    text = source()
    assert 's.startsWith("d ")' in text and 's.startsWith("vibe ")' in text
    assert '"who master"' in text


def test_old_test_commands_are_kept():
    text = source()
    for command in ("case 'f'", "case 'b'", "case 'l'", "case 'r'", "case 's'",
                    's == "lf"', 's == "lb"', 's == "fw"', 's.startsWith("vib")',
                    's.startsWith("power ")', 's == "slave"', 's == "ping"', 's == "p"'):
        assert command in text, command


def test_vibration_power_is_zero_unless_asked():
    assert 'vibeOn ? vibeStrength : ((cmdM1 || cmdM2) ? cmdPower : 0)' in source()


def test_vibe_takes_a_motor_number():
    text = source()
    assert 'vibeMotor = (m == 2 || m == 3) ? m : 1;' in text           # 3 = both (17 Nov)
    assert 'bool m1 = cmdM1 || (vibeOn && vibeMotor != 2);' in text
    assert 'bool m2 = cmdM2 || (vibeOn && vibeMotor >= 2);' in text
    assert '(uint8_t)m1, (uint8_t)m2, power}' in text
