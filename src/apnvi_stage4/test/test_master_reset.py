"""A master that comes up stuck is restarted over USB. No real port is opened."""

from apnvi_stage4.drive_link_node import MAX_RESETS, SerialLink, should_reset, WHO_GIVE_UP_S


class FakePort:
    """A fake serial port that records what happens to its DTR and RTS lines."""

    def __init__(self):
        self.lines = []

    def __setattr__(self, name, value):
        if name in ('dtr', 'rts'):
            self.lines.append((name, value))
        object.__setattr__(self, name, value)

    def readline(self):
        return b''

    def write(self, data):
        pass

    def close(self):
        pass


def test_reset_only_when_open_long_enough_silent_and_tries_are_left():
    assert not should_reset(10.0, 8.0, False, 0)                    # only 2 s after opening
    assert should_reset(10.0 + WHO_GIVE_UP_S, 10.0, False, 0)
    assert not should_reset(100.0, 10.0, True, 0)                   # it answered
    assert not should_reset(100.0, 10.0, False, MAX_RESETS)         # tried enough


def test_reset_pulse_is_the_one_that_worked_on_13_nov():
    port = FakePort()
    link = SerialLink(lambda path: port, lambda line: None, lambda text: None)
    assert link.open('/dev/fake')
    waits = []
    assert link.reset_board(sleep=waits.append)
    assert port.lines == [('dtr', False), ('rts', True), ('rts', False)] and waits == [0.2]
    link.close()
    assert not link.reset_board()                                   # no port: nothing to do
