"""
The fast follower: keeps the red box on the object between AI answers.

Plain Python and OpenCV, nothing from ROS, so it can be tested on its own.

Why it is needed (Adel, 2026-10-13: tracking was "clunky and slow"): Qwen answers about every
2 seconds, and each answer is about a picture taken when the question was sent. On its own the
box only jumps every 2 seconds, and it is always where the object was, not where it is.

How it works (the usual "detect now and then, track in between" design, as in the YOLO + KCF
and Grounded-SAM-2 projects):
  1. Keep the last few seconds of camera pictures (small copies) and where the box was in each.
  2. The first answer starts an OpenCV tracker (CSRT, see make_tracker) on the picture the answer
     is about, then runs it forward through the pictures since, to catch up to now.
  3. Every new picture moves the box. The shown box is smoothed a little so it does not jitter.
  4. Each later answer is checked against where the tracker had the box at that same moment.
     While tracking, Qwen lists every match (two hands = two boxes), and the one that overlaps
     the tracker most is used, so the box stays on the same hand (the usual "match by IoU"
     step of multi-object trackers such as SORT):
     - they overlap (IoU at least AGREE_IOU): the tracker is right. Nudge the box by the
       difference and carry on. No catching up, so no pause.
     - none overlaps: the tracker followed the wrong thing (trackers can do that and still
       report success). Restart from the nearest match, as in step 2.
If the tracker loses the object, the box is hidden and the follower searches for it again:
it compares a snapshot of the object, cut from a picture Qwen confirmed, with the area
around where the object was last seen (OpenCV template matching, the
"re-detect" step of long-term trackers such as TLD). When it finds a good match it carries
on from there.

Catching up runs the tracker on at most MAX_CATCH_UP of the kept pictures, spread evenly,
so a restart costs a fixed, small amount of time however old the answer is.

FollowerWorker runs all of this in its own thread, and the browser picture only reads a
copy of the newest box, so it never waits for the tracker (Adel, 13 Oct: video lagged).
"""

from collections import deque
import threading
import time

import cv2
import numpy as np

TRACK_WIDTH = 480           # track on small copies: much faster, accurate enough for a box
HISTORY_S = 8.0             # keep this many seconds of pictures to catch up after an answer
MIN_BOX_PX = 4              # boxes smaller than this (in the small copy) cannot be tracked
AGREE_IOU = 0.3             # answer and tracker overlap at least this much = tracker is right
SMOOTHING = 0.8             # shown box = 80% newest position + 20% previous (0.6 lagged)
CSRT_TEMPLATE = 120         # CSRT's working size in pixels (default 200): faster, as accurate
MAX_CATCH_UP = 12           # catch up on at most this many pictures after a restart
REFIND_SCORE = 0.6          # how alike (0 to 1) a place must look to the snapshot to re-find it
REFIND_REACH = 3            # search this many box sizes around where the object was last seen
REFIND_SIZES = (0.8, 1.0, 1.25)  # also try the snapshot a bit smaller and bigger
REFIND_EVERY = 3            # while lost, search on every 3rd picture (saves the Jetson's CPU)


def make_tracker():
    """
    Return a new OpenCV tracker: CSRT if available, else KCF, else MIL, else None.

    CSRT, not KCF (changed 13 Oct): on a test of a turning, fast-moving object over a busy
    background, KCF stayed on it 2 to 4% of the time and CSRT 100%, even at 3 times the speed.
    CSRT_TEMPLATE and no segmentation make CSRT about as fast as KCF with the same result.
    """
    if hasattr(cv2, 'TrackerCSRT_create'):
        params = cv2.TrackerCSRT_Params()
        params.template_size = CSRT_TEMPLATE
        params.use_segmentation = False
        return cv2.TrackerCSRT_create(params)
    for name in ('TrackerKCF_create', 'TrackerMIL_create'):
        if hasattr(cv2, name):
            return getattr(cv2, name)()
    legacy = getattr(cv2, 'legacy', None)
    if legacy is not None and hasattr(legacy, 'TrackerKCF_create'):
        return legacy.TrackerKCF_create()
    return None


def shrink(picture):
    """Return a copy TRACK_WIDTH wide (or the picture itself if it is already smaller)."""
    height, width = picture.shape[:2]
    if width <= TRACK_WIDTH:
        return picture
    new_height = round(height * TRACK_WIDTH / width)
    return cv2.resize(picture, (TRACK_WIDTH, new_height), interpolation=cv2.INTER_AREA)


def iou(a, b):
    """Return how much two boxes (left, top, right, bottom) overlap: 0 = not at all, 1 = same."""
    left, top = max(a[0], b[0]), max(a[1], b[1])
    right, bottom = min(a[2], b[2]), min(a[3], b[3])
    inter = max(0, right - left) * max(0, bottom - top)
    union = (a[2] - a[0]) * (a[3] - a[1]) + (b[2] - b[0]) * (b[3] - b[1]) - inter
    return inter / union if union > 0 else 0.0


def centre_distance(a, b):
    """Return the distance in pixels between the centres of two boxes."""
    dx = (a[0] + a[2] - b[0] - b[2]) / 2
    dy = (a[1] + a[3] - b[1] - b[3]) / 2
    return (dx * dx + dy * dy) ** 0.5


def spread(items, most):
    """Return at most `most` items, evenly spread, always ending with the last one."""
    if len(items) <= most:
        return items
    step = len(items) / most
    return [items[len(items) - 1 - round(i * step)] for i in reversed(range(most))]


class Follower:
    """Follows one box through the camera pictures. See the module description."""

    def __init__(self, tracker_factory=make_tracker, history_s=HISTORY_S):
        self.tracker_factory = tracker_factory
        self.history_s = history_s
        self.frames = deque()       # (time in seconds, small picture, tracker box or None)
        self.tracker = None
        self.box = None             # newest tracker box (left, top, right, bottom), small copy
        self.shown = None           # smoothed box for drawing, small copy
        self.lost = False
        self.last_seen = None       # where the object was last seen, when lost
        self.template = None        # grey snapshot of the object from a confirmed picture
        self.lost_pictures = 0      # pictures since the object was lost

    @property
    def available(self):
        """Say if this OpenCV has a tracker at all."""
        return self.tracker_factory() is not None

    def add_frame(self, stamp, picture):
        """Add the newest camera picture (time in seconds) and move the box with it."""
        small = shrink(np.asarray(picture))
        if self.tracker is not None:
            self._step(small)
        elif self.lost:
            self.lost_pictures += 1
            if self.lost_pictures % REFIND_EVERY == 0:
                self._refind(small)
        self.frames.append((stamp, small, self.box))
        while self.frames and stamp - self.frames[0][0] > self.history_s:
            self.frames.popleft()

    def start(self, box, size, stamp=None):
        """
        Start following an answer's box, catching up from its picture to now.

        box is (left, top, right, bottom) in a picture of size (width, height); stamp is the
        time of the picture the answer is about (None = the newest picture). Returns True if
        the follower started.
        """
        self.stop()
        if not self.frames:
            return False
        index = self._index_of(stamp)
        small = self.frames[index][1]
        clipped = self._to_small(box, size, small)
        if clipped is None or not self._init(small, clipped):
            return False
        self._remember(small, clipped)
        # Catch up to now, on a few pictures only. Every kept picture from the answer's one on
        # gets the box written in (pictures skipped get the last box worked out), so the next
        # answer can be compared with where the tracker was, whichever picture it is about.
        self.frames[index] = self.frames[index][:2] + (self.box,)
        later = range(index + 1, len(self.frames))
        steps = set(spread(list(later), MAX_CATCH_UP))
        for i in later:
            if self.tracker is None:
                break
            if i in steps:
                self._step(self.frames[i][1])
            self.frames[i] = self.frames[i][:2] + (self.box,)
        self.shown = self.box
        return self.tracker is not None

    def correct(self, box, size, stamp=None, boxes=None):
        """
        Take a new answer into account. Returns True if the box is being followed.

        boxes is every match Qwen gave (box is the first). The one that overlaps the tracker
        most is used. If it agrees with the tracker, nudge it (no catching up, no pause). If
        none does, start again from the nearest one. If nothing is followed yet, start on box.
        """
        if self.tracker is None or not self.frames:
            return self.start(box, size, stamp)
        index = self._index_of(stamp)
        then_small = self.frames[index][1]
        tracked_then = self.frames[index][2]
        if tracked_then is None:
            return self.start(box, size, stamp)
        candidates = []
        for each in boxes or [box]:
            small = self._to_small(each, size, then_small)
            if small is not None:
                candidates.append((each, small))
        if not candidates:
            return self.start(box, size, stamp)
        box, answer = max(candidates, key=lambda c: (iou(c[1], tracked_then),
                                                     -centre_distance(c[1], tracked_then)))
        if iou(answer, tracked_then) < AGREE_IOU:
            return self.start(box, size, stamp)
        self._remember(then_small, answer)
        # Agreed: move today's box by how far the tracker was off at the answer's moment.
        dx = (answer[0] + answer[2] - tracked_then[0] - tracked_then[2]) / 2
        dy = (answer[1] + answer[3] - tracked_then[1] - tracked_then[3]) / 2
        width, height = answer[2] - answer[0], answer[3] - answer[1]
        cx = (self.box[0] + self.box[2]) / 2 + dx
        cy = (self.box[1] + self.box[3]) / 2 + dy
        nudged = (round(cx - width / 2), round(cy - height / 2),
                  round(cx + width / 2), round(cy + height / 2))
        newest = self.frames[-1][1]
        nudged = self._clip(nudged, newest)
        if nudged is None or not self._init(newest, nudged):
            return self.start(box, size, stamp)
        return True

    def stop(self):
        """Stop following (the box disappears)."""
        self.tracker, self.box, self.shown, self.lost = None, None, None, False
        self.last_seen, self.template = None, None

    def box_in(self, width, height):
        """Return the shown box scaled to a picture of width x height, or None."""
        if self.shown is None or not self.frames:
            return None
        small = self.frames[-1][1]
        sx, sy = width / small.shape[1], height / small.shape[0]
        left, top, right, bottom = self.shown
        return (round(left * sx), round(top * sy), round(right * sx), round(bottom * sy))

    def _index_of(self, stamp):
        """Return the index of the kept picture closest in time to stamp (newest if None)."""
        if stamp is None:
            return len(self.frames) - 1
        times = [frame[0] for frame in self.frames]
        return min(range(len(times)), key=lambda i: abs(times[i] - stamp))

    def _to_small(self, box, size, small):
        """Scale an answer's box into the small copy's pixels, clipped to it, or None."""
        scale = small.shape[1] / size[0]
        return self._clip(tuple(round(v * scale) for v in box), small)

    @staticmethod
    def _clip(box, small):
        """Keep a box inside the picture. None if what is left is too small to track."""
        left, top = max(box[0], 0), max(box[1], 0)
        right, bottom = min(box[2], small.shape[1] - 1), min(box[3], small.shape[0] - 1)
        if right - left < MIN_BOX_PX or bottom - top < MIN_BOX_PX:
            return None
        return (left, top, right, bottom)

    def _init(self, small, box):
        """Start a fresh tracker on this picture at this box. Returns True if it worked."""
        tracker = self.tracker_factory()
        if tracker is None:
            return False
        left, top, right, bottom = box
        try:
            tracker.init(small, (int(left), int(top), int(right - left), int(bottom - top)))
        except cv2.error:
            return False
        self.tracker, self.box, self.lost = tracker, box, False
        if self.shown is None:
            self.shown = box
        return True

    def _step(self, small):
        """Move the box to where the object is in this picture, or mark it lost."""
        try:
            ok, (x, y, w, h) = self.tracker.update(small)
        except cv2.error:
            ok = False
        box = self._clip((int(x), int(y), int(x + w), int(y + h)), small) if ok else None
        if box is None:                                 # lost: search for it again now
            self.last_seen = self.box
            self.tracker, self.box, self.shown, self.lost = None, None, None, True
            self.lost_pictures = 0
            self._refind(small)
            return
        self.box = box
        if self.shown is None:
            self.shown = self.box
        else:
            self.shown = tuple(round(SMOOTHING * new + (1 - SMOOTHING) * old)
                               for new, old in zip(self.box, self.shown))

    def _remember(self, small, box):
        """Keep a grey snapshot of the object at this box, to search for it if it is lost."""
        left, top, right, bottom = box
        self.template = cv2.cvtColor(small[top:bottom, left:right], cv2.COLOR_BGR2GRAY)

    def _refind(self, small):
        """Look for the snapshot near where the object was last seen; carry on if found."""
        if self.template is None or self.last_seen is None:
            return
        left, top, right, bottom = self.last_seen
        reach = REFIND_REACH * max(right - left, bottom - top)
        x0, y0 = max(left - reach, 0), max(top - reach, 0)
        x1 = min(right + reach, small.shape[1])
        y1 = min(bottom + reach, small.shape[0])
        area = cv2.cvtColor(small[y0:y1, x0:x1], cv2.COLOR_BGR2GRAY)
        best = (REFIND_SCORE, None)
        for size in REFIND_SIZES:
            template = cv2.resize(self.template, None, fx=size, fy=size)
            height, width = template.shape
            if (height < MIN_BOX_PX or width < MIN_BOX_PX
                    or height > area.shape[0] or width > area.shape[1]):
                continue
            scores = cv2.matchTemplate(area, template, cv2.TM_CCOEFF_NORMED)
            _, score, _, (x, y) = cv2.minMaxLoc(scores)
            if score >= best[0]:
                best = (score, (x0 + x, y0 + y, x0 + x + width, y0 + y + height))
        found = best[1] and self._clip(best[1], small)
        if found and self._init(small, found):
            self.shown = found


class FollowerWorker:
    """
    Runs a Follower in its own thread, so drawing the browser picture never waits for it.

    submit_frame() and submit_answer() only hand things over and return at once. The thread
    always works on the newest picture; if it falls behind, older pictures are skipped.
    """

    def __init__(self, follower=None):
        self.follower = follower or Follower()
        self.lock = threading.Lock()            # protects the Follower
        self.wake = threading.Condition()
        self.frame = None                       # newest (stamp, picture) not yet processed
        self.answers = []                       # (box, size, stamp, boxes) waiting
        self.latest = (None, None)              # copy of (shown box, small picture size)
        self.done_stamp = None                  # time of the picture that box belongs to
        self.done, self.busy_s = 0, 0.0         # pictures handled, time spent, since last asked
        self.stop_request = False
        self.running = True
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()

    @property
    def available(self):
        """Say if this OpenCV has a tracker at all."""
        return self.follower.available

    def submit_frame(self, stamp, picture):
        """Hand over the newest camera picture."""
        with self.wake:
            self.frame = (stamp, picture)
            self.wake.notify()

    def submit_answer(self, box, size, stamp, boxes=None):
        """Hand over a new found answer (boxes: every match Qwen gave, if several)."""
        with self.wake:
            self.answers.append((box, size, stamp, boxes))
            self.wake.notify()

    def stop_following(self):
        """Hide the box and forget the object."""
        with self.wake:
            self.answers.clear()
            self.stop_request = True
            self.wake.notify()

    def box_in(self, width, height):
        """Return the shown box in a width x height picture, or None. Never waits."""
        shown, small_size = self.latest
        if shown is None:
            return None
        sx, sy = width / small_size[0], height / small_size[1]
        left, top, right, bottom = shown
        return (round(left * sx), round(top * sy), round(right * sx), round(bottom * sy))

    @property
    def following(self):
        """Say if a box is being followed. Never waits."""
        return self.latest[0] is not None

    def take_counts(self):
        """Return (pictures handled, seconds spent on them) since the last call, for the view."""
        done, busy_s = self.done, self.busy_s
        self.done, self.busy_s = 0, 0.0
        return done, busy_s

    def close(self):
        """Stop the thread."""
        with self.wake:
            self.running = False
            self.wake.notify()
        self.thread.join(timeout=2.0)

    def _run(self):
        while True:
            with self.wake:
                self.wake.wait_for(lambda: (not self.running or self.frame is not None
                                            or self.answers or self.stop_request))
                if not self.running:
                    return
                frame, self.frame = self.frame, None
                answers, self.answers = self.answers, []
                stop, self.stop_request = self.stop_request, False
            started = time.monotonic()
            with self.lock:
                if stop:
                    self.follower.stop()
                if frame is not None:
                    self.follower.add_frame(*frame)
                    self.done_stamp = frame[0]
                for box, size, stamp, boxes in answers:
                    self.follower.correct(box, size, stamp, boxes)
                follower = self.follower
                if follower.shown is None or not follower.frames:
                    self.latest = (None, None)
                else:
                    small = follower.frames[-1][1]
                    self.latest = (follower.shown, (small.shape[1], small.shape[0]))
            if frame is not None:
                self.done += 1
                self.busy_s += time.monotonic() - started
