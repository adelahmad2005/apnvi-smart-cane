"""
The guide's decisions, with no ROS, so they can be tested anywhere.

States:
  idle      no route; the device stands still (hazard warnings and vibration still run)
  guiding   following a route to the goal
  paused    the user pressed the talk button (red) or the emergency stop (green) while being
            guided: all driving stopped, the old goal kept until the next question decides

What happens (Adel, 11 Nov):
  - a goal ("take me to ...", a map point)   -> say "Taking you to ...", start guiding
  - red pressed while guiding                -> stop at once and pause (Stage 2 says
                                                "Stopping. Listening" for this press)
  - green switched on while guiding          -> stop at once, pause, say "Stopping"
  - after a pause, the next question decides:
      a new goal       -> the new goal wins
      a follow-up      -> carry on to the old goal ("Continuing")
      anything else    -> the old route is dropped (the new question wins)
  - within 0.5 m of the goal                 -> stop, say "You have arrived"
  - the goal is a thing (16 Nov)             -> the stop is a little in front of it; on
                                                arrival it also says where the thing is:
                                                "The water bottle is ahead of you, about
                                                1 metre"
  - a thing that is not on the map           -> the node asks Stage 2 to look for it now
                                                ("Looking for the water bottle"); found:
                                                a goal; not found: a sentence that says why

Every method returns a list of actions for the node to carry out, each a tuple:
  ('follow', goal)   plan and follow a route to goal (also sent again to re-plan)
  ('cancel',)        stop following the route now
  ('say', words)     say this sentence
"""

from dataclasses import dataclass
import math

ARRIVED_M = 0.5                 # arrival within about 0.5 m (Adel, 11 Nov)

ARRIVED = 'You have arrived'
STOPPING = 'Stopping'
CONTINUING = 'Continuing'
NOTHING_TO_CONTINUE = 'There is no route to continue. Say take me to, then a place'
LOOKING = 'Looking for the %s'
BLOCKED = 'Something is in the way. Waiting'
NO_WAY = "I can't find a way there right now. Turn me a little toward open floor"
# 17 Nov (lab A2, Adel): when the camera keeps it stopped facing desks, the device cannot turn
# away by itself; the user is told to turn it.
STILL_BLOCKED = 'The way is still blocked. Turn me a little to the left or right'
# 17 Nov (Adel): while the device has not found itself on the map, a trip waits (its position
# would be wrong), and starts by itself once it is found.
WAIT_PLACED = 'Taking you to %s once I know where you are. Keep walking'
BLOCKED_SAY_S = 3.0             # blocked this long while guiding: say so (23 Nov: 4 -> 3)
STILL_BLOCKED_S = 8.0           # still blocked this long: ask to be turned (23 Nov: 15 -> 8)
NO_WAY_SAY_S = 6.0              # no route this long while guiding: say so (once per stop)
# 19 Nov demo: the follower stopped itself 490 times ("collision ahead") and nothing was said.
# Now its giving up is said once, then not again until the device has moved CLOSE_RESET_M.
CLOSE = 'Something is close. Waiting a moment'
CLOSE_RESET_M = 0.5
# Why Stage 2 could not place a thing on the map (its "problem"), as a sentence. %s = the name.
NOT_LOCATED = {
    'not_seen': "I can't see the %s and it isn't on the map. Turn slowly and ask again",
    'no_depth': "I can see the %s but I can't tell how far it is. Go closer and ask again",
    'too_far': 'I can see the %s but it is too far away. Go closer and ask again',
    'no_position': "I can see the %s but I don't know where I am on the map yet",
    'moved': 'Hold still and ask for the %s again',
    'no_camera': 'The camera is not ready',
    'no_helper': "I can't reach the helper computer",
    'busy': 'Please ask again',
    'timeout': "I couldn't look for the %s. Please ask again",
}


@dataclass
class Goal:
    """Where to go: a point on the map and the name to say."""

    x: float
    y: float
    name: str = 'the point'
    thing: tuple = None             # (x, y) of the thing itself, when the goal is in front of it
    wanted: tuple = None            # (x, y) first asked for, before any move_stop_point
    held: bool = False              # the moved stop point got a route: kept while it fits


def move_stop_point(goal, finder, fits=None):
    """
    Move the stop point to where the device fits, always measured from the point first asked for.

    finder(x, y) gives the nearest point that fits, or None. 19 Nov demo (outside EB2): each second
    the stop point was moved again from the last moved point, so it crept 1.9 m away (more than
    reach.MAX_MOVE_M) and the trip "arrived" with the push-bar sign behind the user. Returns the
    old (x, y) if the stop point changed, else None.

    fits(x, y) says if the device still fits at a point. 23 Nov (a trip to the door): the point
    was picked again every second and jumped about as the map changed, so the route changed and
    the device kept turning. A moved point that got a route (goal.held) is now kept while it fits.
    """
    if goal.wanted is None:
        goal.wanted = (goal.x, goal.y)
    if fits is not None and goal.held and (goal.x, goal.y) != goal.wanted \
            and fits(goal.x, goal.y):
        return None
    point = finder(*goal.wanted)
    if point is None:
        point = goal.wanted
    if distance(point[0], point[1], goal.x, goal.y) <= 0.05:
        return None
    old = (goal.x, goal.y)
    goal.x, goal.y = point
    goal.held = False                   # a new point is held once a route reaches it
    return old


def distance(x1, y1, x2, y2):
    """Straight-line distance in metres."""
    return math.hypot(x2 - x1, y2 - y1)


def where_words(name, position, yaw, thing):
    """
    Say where a thing is from the device: "The door is ahead of you, about 1 metre".

    position: the device's (x, y). yaw: the way it faces (radians), or None if not known.
    """
    gap = distance(position[0], position[1], thing[0], thing[1])
    far = 'less than a metre' if gap < 0.75 else 'about %d metre%s' % (
        int(round(gap)), '' if int(round(gap)) == 1 else 's')
    words = name[0].upper() + name[1:]
    if yaw is None:
        return '%s is %s away' % (words, far)
    angle = math.degrees(math.atan2(thing[1] - position[1], thing[0] - position[0]) - yaw)
    angle = (angle + 180.0) % 360.0 - 180.0             # -180..180, + = to the left
    side = 'left' if angle > 0 else 'right'
    if abs(angle) <= 20:
        where = 'ahead of you'
    elif abs(angle) <= 60:
        where = 'ahead on your %s' % side
    elif abs(angle) <= 120:
        where = 'on your %s' % side
    else:
        where = 'behind you'
    return '%s is %s, %s' % (words, where, far)


def metres_words(metres):
    """'about 12 metres to go', said with a follow-up."""
    whole = max(1, int(round(metres)))
    return 'about %d metre%s to go' % (whole, '' if whole == 1 else 's')


class Guide:
    """The guide's state, changed only through its methods."""

    def __init__(self, arrived_m=ARRIVED_M):
        self.arrived_m = arrived_m
        self.state = 'idle'
        self.goal = None
        self.position = None            # (x, y) on the map, newest
        self.yaw = None                 # the way the device faces on the map, newest
        self.why_paused = ''
        self.blocked_since = self.no_way_since = None   # for waiting(): when each began
        self.blocked_said = self.no_way_said = False
        self.still_said = False
        self.placed = True              # False while Stage 3 is still finding the device
        self.close_said_at = None       # where CLOSE was last said this trip ('?' if unknown)

    def distance_left(self):
        """Metres from the device to the goal, or None if either is unknown."""
        if self.goal is None or self.position is None:
            return None
        return distance(self.position[0], self.position[1], self.goal.x, self.goal.y)

    def arrived_words(self):
        """Give what is said on arrival: for a thing, also where it is."""
        if self.goal is None or self.goal.thing is None or self.position is None:
            return ARRIVED
        return ARRIVED + '. ' + where_words(self.goal.name, self.position, self.yaw,
                                            self.goal.thing)

    # ---- inputs --------------------------------------------------------------------------

    def new_goal(self, goal):
        """Start a trip to a new place. It always wins over an old one."""
        self.goal = goal
        self.why_paused = ''
        self.close_said_at = None
        if not self.placed:
            self.state = 'guiding'
            return [('cancel',), ('say', WAIT_PLACED % goal.name)]
        left = self.distance_left()
        if left is not None and left <= self.arrived_m:
            words = self.arrived_words()
            self.state = 'idle'
            self.goal = None
            return [('cancel',), ('say', words)]
        self.state = 'guiding'
        return [('follow', goal), ('say', 'Taking you to %s' % goal.name)]

    def unknown_place(self, name):
        """Handle a place that is not on the map: it is still a new question."""
        actions = self.drop() if self.state == 'paused' else []
        return actions + [('say', "I don't know where %s is" % name)]

    def looking_for(self, name):
        """Handle a thing that is not on the map: Stage 2 is asked to look for it now."""
        actions = self.drop() if self.state == 'paused' else []
        return actions + [('say', LOOKING % name)]

    def not_located(self, name, problem):
        """Handle Stage 2's answer that the thing could not be placed on the map."""
        words = NOT_LOCATED.get(problem, NOT_LOCATED['not_seen'])
        return [('say', words % name if '%s' in words else words)]

    def talk_pressed(self):
        """Handle the talk button (red, or the old GPIO 23 button) going down."""
        if self.state != 'guiding':
            return []
        self.state, self.why_paused = 'paused', 'red'
        return [('cancel',)]            # Stage 2 says "Stopping. Listening"

    def emergency_on(self):
        """Handle the green button being switched on."""
        if self.state != 'guiding':
            return []
        self.state, self.why_paused = 'paused', 'green'
        return [('cancel',), ('say', STOPPING)]

    def follow_up(self, green_on=False):
        """Handle a follow-up question: carry on to the old goal."""
        left = self.distance_left()
        if self.state == 'paused' and self.goal is not None:
            if green_on:
                return [('say', 'The emergency stop is on. Switch it off, then say continue')]
            self.state, self.why_paused = 'guiding', ''
            self.close_said_at = None
            if not self.placed:
                return [('say', WAIT_PLACED % self.goal.name)]
            words = CONTINUING + (', %s' % metres_words(left) if left is not None else '')
            return [('follow', self.goal), ('say', words)]
        if self.state == 'guiding' and left is not None:
            return [('say', metres_words(left).capitalize())]
        return [('say', NOTHING_TO_CONTINUE)]

    def other_question(self):
        """Handle any other question: after a pause, the old route is dropped."""
        return self.drop() if self.state == 'paused' else []

    def drop(self):
        """Forget the goal."""
        self.state, self.goal, self.why_paused = 'idle', None, ''
        return [('cancel',)]

    def moved(self, x, y, yaw=None):
        """Take the device's newest position on the map. Arrival is checked here."""
        self.position, self.yaw = (x, y), yaw
        said = self.close_said_at
        if said not in (None, '?') and distance(x, y, said[0], said[1]) >= CLOSE_RESET_M:
            self.close_said_at = None
        left = self.distance_left()
        if self.state == 'guiding' and self.placed and left is not None and left <= self.arrived_m:
            words = self.arrived_words()
            self.state, self.goal = 'idle', None
            return [('cancel',), ('say', words)]
        return []

    def follower_finished(self, succeeded, blocked=False):
        """
        Handle the end of Nav2's follower. Success means within its 0.5 m goal check.

        The device's own position is checked too: on 15 Nov the follower once said "reached"
        the moment it got a route 1.5 m long. If the position is known and still too far,
        nothing is said and the next re-plan sends the route again.

        A failure (the follower gave up, mostly "collision ahead") is said once as CLOSE, unless
        the hazard gate is stopping it (blocked: BLOCKED is said for that). The next re-plan
        sends the route again.
        """
        if self.state != 'guiding':
            return []
        if not succeeded:
            if blocked or self.close_said_at is not None:
                return []
            self.close_said_at = self.position if self.position is not None else '?'
            return [('say', CLOSE)]
        left = self.distance_left()
        if left is not None and left > self.arrived_m + 0.25:
            return []
        words = self.arrived_words()
        self.state, self.goal = 'idle', None
        return [('cancel',), ('say', words)]

    def waiting(self, now, blocked, route_ok):
        """
        Once a second: if the trip is held up, say why (each reason once per hold-up).

        blocked: the hazard gate stops forward driving. route_ok: False when the planner
        found no route. Silence while the device stands still is the worst answer for a
        blind user, so after a few seconds it says what it is waiting for.
        """
        if self.state != 'guiding':
            self.blocked_since = self.no_way_since = None
            self.blocked_said = self.no_way_said = self.still_said = False
            return []
        actions = []
        if blocked:
            if self.blocked_since is None:
                self.blocked_since = now
            elif not self.blocked_said and now - self.blocked_since >= BLOCKED_SAY_S:
                self.blocked_said = True
                actions.append(('say', BLOCKED))
            elif not self.still_said and now - self.blocked_since >= STILL_BLOCKED_S:
                self.still_said = True
                actions.append(('say', STILL_BLOCKED))
        else:
            self.blocked_since, self.blocked_said, self.still_said = None, False, False
        if route_ok is False:
            if self.no_way_since is None:
                self.no_way_since = now
            elif not self.no_way_said and now - self.no_way_since >= NO_WAY_SAY_S:
                self.no_way_said = True
                actions.append(('say', NO_WAY))
        else:
            self.no_way_since, self.no_way_said = None, False
        return actions

    def tick(self):
        """Once a second: re-plan while guiding (the route redraws when someone steps in)."""
        if self.state == 'guiding' and self.goal is not None and self.placed:
            return [('follow', self.goal)]
        return []

    def placement(self, placed):
        """
        Take Stage 3's news: placed on the map or not (17 Nov).

        Not placed: a trip is held (the route stops) and waits. Placed again: a waiting trip
        starts.
        """
        was, self.placed = self.placed, placed
        if self.state != 'guiding' or self.goal is None or was == placed:
            return []
        if not placed:
            return [('cancel',)]
        return [('follow', self.goal), ('say', 'Taking you to %s' % self.goal.name)]

    def summary(self):
        """Give what the 8081 page shows."""
        left = self.distance_left()
        return {
            'state': self.state,
            'goal': None if self.goal is None else {
                'name': self.goal.name, 'x': round(self.goal.x, 2), 'y': round(self.goal.y, 2)},
            'distance_m': None if left is None else round(left, 2),
            'paused_by': self.why_paused,
            'placed': self.placed,
        }
