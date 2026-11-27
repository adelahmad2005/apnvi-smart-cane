"""
Health: notice when a part of the device stops working, and say it once. No ROS here.

Why (requirements FR8.3 and FR8.4, and the lab days of 13 to 15 Nov): a blind user cannot see
a status page. When the camera is covered, the LiDAR stops, the handle's radio drops or the
device loses its place on the map, they must be told, once, in plain words, and told again
when it works again.

A part is watched only after it was seen working once, so a part that is switched off on
purpose (the wheels with drive:=false, a test without the map) never nags. "Ready" is said
once, when every part that must be there is working.
"""

from dataclasses import dataclass

STALE_S = 2.0                   # a part that sends regularly: silent this long = stopped
BAD_FOR_S = 3.0                 # a part that reports "bad": bad this long before saying so
READY = 'Ready'
READY_PARTS = ('warnings', 'camera')    # these must work before "Ready" is said
READY_WAIT_S = 60.0             # say what is missing if they are not working by then


@dataclass
class Part:
    """One watched part: its sentences and how it is judged."""

    down: str                   # said when it stops working
    up: str                     # said when it works again
    beat: bool = True           # True: judged by regular messages (heartbeat). False: by
    #                             the newest good/bad report
    seen_ok: bool = False
    last_ok: float = -1e9       # when it was last known to work
    bad_since: float = None
    announced: bool = False     # "down" was said and "up" not yet


PARTS = {
    'warnings': ('The obstacle warning has stopped. Stand still',
                 'The obstacle warning is working again', True),
    'camera': ('The camera is covered or not working', 'The camera is working again', False),
    'lidar': ('The laser scanner has stopped', 'The laser scanner is working again', True),
    'wheels': ('The wheels are not connected', 'The wheels are connected again', False),
    'handle': ('The handle is not connected', 'The handle is connected again', False),
    'place': ('I have lost my place on the map', 'I know where I am again', False),
}


class Health:
    """Give it what each part reports; step() gives the sentences to say now."""

    def __init__(self, started_at=0.0):
        self.parts = {name: Part(down, up, beat) for name, (down, up, beat) in PARTS.items()}
        self.started_at = started_at
        self.ready_said = False
        self.missing_said = False

    def ok(self, name, now):
        """Note that a part works (a message came, or it reported good)."""
        part = self.parts[name]
        part.seen_ok, part.last_ok, part.bad_since = True, now, None

    def bad(self, name, now):
        """Note that a part reported it does not work."""
        part = self.parts[name]
        if part.bad_since is None:
            part.bad_since = now

    def working(self, name, now):
        """Say if a part is working now (None: never seen working)."""
        part = self.parts[name]
        if not part.seen_ok:
            return None
        if part.beat:
            return now - part.last_ok <= STALE_S
        return part.bad_since is None or now - part.bad_since < BAD_FOR_S

    def step(self, now):
        """Give the sentences to say now: failures, recoveries, and "Ready" once."""
        words = []
        for name, part in self.parts.items():
            state = self.working(name, now)
            if state is False and not part.announced:
                part.announced = True
                words.append(part.down)
            elif state is True and part.announced:
                part.announced = False
                words.append(part.up)
        if not self.ready_said:
            if all(self.working(name, now) for name in READY_PARTS):
                self.ready_said = True
                words.append(READY)
            elif not self.missing_said and now - self.started_at >= READY_WAIT_S:
                self.missing_said = True
                missing = [name for name in READY_PARTS if not self.working(name, now)]
                words.append('Not ready: the %s %s not working' % (
                    ' and the '.join('obstacle warning' if n == 'warnings' else n
                                     for n in missing), 'is' if len(missing) == 1 else 'are'))
        return words

    def summary(self, now):
        """Give each part's state for the status line: True, False or None (never seen)."""
        return {name: self.working(name, now) for name in self.parts}
