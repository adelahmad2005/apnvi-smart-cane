"""
Eye: listens to camera topics only while a picture is wanted.

Every full-size camera picture a Python program receives costs the Jetson time, even when the
picture is thrown away. The depth picture is needed a few times a minute (to place a found
thing on the map, or for the labeller's look every few seconds), so the Eye subscribes when
want() is called and lets go KEEP_S after the last call. Subscribing and letting go both
happen in the node's own timer, never in a worker thread.
"""

import threading
import time

KEEP_S = 5.0                    # stay subscribed this long after the last want()
TICK_S = 0.1


class Eye:
    """Newest messages of a few topics, received only while wanted."""

    def __init__(self, node, topics, keep_s=KEEP_S, clock=time.monotonic):
        """topics: {name: (message type, topic, qos)}."""
        self.node, self.topics, self.keep_s, self.clock = node, topics, keep_s, clock
        self.subs = {}
        self.newest = {}                # name -> (time it came in, message)
        self.wanted_until = 0.0
        self.lock = threading.Lock()
        node.create_timer(TICK_S, self.tick)

    def want(self):
        """Ask for pictures from now on (call again to keep them coming)."""
        self.wanted_until = self.clock() + self.keep_s

    def tick(self):
        """Subscribe when wanted, let go when not (runs in the node's timer)."""
        wanted = self.clock() < self.wanted_until
        if wanted and not self.subs:
            for name, (kind, topic, qos) in self.topics.items():
                self.subs[name] = self.node.create_subscription(
                    kind, topic, lambda msg, name=name: self.take(name, msg), qos)
        elif not wanted and self.subs:
            for sub in self.subs.values():
                self.node.destroy_subscription(sub)
            self.subs = {}
            with self.lock:
                self.newest = {}

    def take(self, name, msg):
        with self.lock:
            self.newest[name] = (self.clock(), msg)

    def get(self, name, max_age_s=0.5):
        """Give the newest message of this topic, or None if there is none this fresh."""
        with self.lock:
            entry = self.newest.get(name)
        if entry is None or self.clock() - entry[0] > max_age_s:
            return None
        return entry[1]

    def wait(self, names, timeout_s=1.5, max_age_s=0.5):
        """
        Wait for a fresh message of each topic; {name: message}, or None when time runs out.

        Blocks: call it from a worker thread, never from a ROS callback.
        """
        end = self.clock() + timeout_s
        while True:
            self.want()
            got = {name: self.get(name, max_age_s) for name in names}
            if all(message is not None for message in got.values()):
                return got
            if self.clock() >= end:
                return None
            time.sleep(0.05)
