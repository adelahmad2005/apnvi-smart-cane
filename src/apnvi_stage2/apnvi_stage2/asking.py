"""
What happens to one question, from the recording (or typed words) to the answer.

Plain Python, nothing from ROS, so it can be tested with fake helpers. stage2_node.py
calls answer_recording() or answer_typed() and then shows or says what comes back.

Decided 2026-10-08 (stage-design.md, Stage 2 section 10):
  found      -> red box on the screen, silent
  not found  -> "I can't see it, turn slowly and try again"
The other three sentences were decided by Adel on 2026-10-10.

Questions that need words back (Adel, 2026-10-26): when a router is given, Qwen first sorts
the question. FIND goes on as above. ANSWER gets Qwen's own short sentence spoken, with a red
box on what it is about if there is one. No router (tracking, old callers) = always FIND.

Stage 4 (12 Nov): two kinds of question are sorted by their words, before Qwen is asked
(no Qwen, no camera needed, Stage 2 says nothing; the guide node answers):
  "take me to the restroom" (also guide/lead/bring/walk me to, navigate to, go to)
      -> kind 'guide', place 'restroom'
  a follow-up while being guided: the whole question is one of FOLLOW_UPS
      ("continue", "keep going", "how far", "are we there yet", ...) -> kind 'follow_up'
Everything else is sorted as before. After each question, request_for() gives the line the
guide node gets on /stage2/request.

Map commands (16 Nov), also sorted by their words, with no Qwen and no camera. Stage 2 says
nothing; the map manager or the labeller answers (spoken_command):
  "save this place as my desk"            -> the spot where the device stands gets that name
  "scan the room"                         -> the labeller looks all round for a minute
  "forget the bin"                        -> that pin is taken off the map
  "save this map as ESB117", "load the map ESB117", "start a new map"
"""

from dataclasses import dataclass, replace
import math
import re

import numpy as np

from .finder import draw_box, find, FindResult, reply
from .labels import UNREAD_SIGNS
from .speech import to_text

NOT_FOUND = "I can't see it, turn slowly and try again"          # thing not in view
NOT_HEARD = "I didn't catch that. Hold the button and ask again"   # no words heard
NO_HELPER = "I can't reach the helper computer"                    # campus PC unreachable
NO_CAMERA = 'The camera is not ready'                              # no picture yet

# Feedback while asking (voice plan, Adel, 26 Oct)
LISTENING = 'Listening'                                            # button pressed
HEARD = 'Heard'                                                    # the words came back
STILL_THINKING = 'Still thinking'                                  # no answer after a while
TOO_LONG = 'Sorry, that took too long. Please ask again'           # no answer at all
ASK_AGAIN = 'Please ask again'                                     # answer held too long
STOPPING = 'Stopping'           # Stage 4: talk button pressed while being guided

# Stage 4: "take me to ..." and follow-ups, sorted by their words (see the top of this file)
GUIDE_STARTS = ('take me to', 'guide me to', 'lead me to', 'bring me to', 'walk me to',
                'navigate to', 'go to', 'pick me to')    # 18 Nov: "take" heard as "pick"
FOLLOW_UPS = {'continue', 'keep going', 'carry on', 'go on', 'resume', 'lets go', 'go',
              'how far', 'how far is it', 'how much further', 'how much farther',
              'are we there', 'are we there yet'}
# Map commands by voice (16 Nov). Each start is followed by the name.
SAVE_PLACE_STARTS = ('save this place as', 'save this spot as', 'save this location as',
                     'remember this place as', 'remember this spot as', 'mark this place as',
                     'call this place', 'name this place', 'save this as', 'remember this as')
SAVE_MAP_STARTS = ('save this map as', 'save the map as', 'save map as')
LOAD_MAP_STARTS = ('load the map', 'load map', 'open the map', 'open map')
FORGET_STARTS = ('forget', 'remove')
NEW_MAP_WORDS = {'start a new map', 'new map', 'make a new map', 'start new map'}
SCAN_WORDS = {'scan the room', 'scan this room', 'scan room', 'scan the area', 'scan',
              'look around', 'label the room', 'label this room'}
POLITE = re.compile(r'^(please|ok|okay|can you|could you)\s+|\s+please$')


def plain_words(question):
    """'Please, take me to the Restroom!' -> 'take me to the restroom'."""
    words = re.sub(r"[^a-z0-9 ]+", ' ', question.lower().replace("'", ''))
    words = ' '.join(words.split())
    while True:
        shorter = POLITE.sub('', words).strip()
        if shorter == words:
            return words
        words = shorter


def guide_place(question):
    """'take me to the restroom' -> 'restroom'; not a "take me to" question -> None."""
    words = plain_words(question)
    for start in GUIDE_STARTS:
        if words.startswith(start + ' '):
            place = re.sub(r'^(the|a|an|my)\s+', '', words[len(start):].strip())
            return place or None
    return None


def is_follow_up(question):
    """Say if the whole question is a follow-up to the trip ("continue", "how far")."""
    return plain_words(question) in FOLLOW_UPS


def spoken_command(question):
    """
    Give the map command in these words, as the line for /stage3/command, or None.

    'save this place as my desk' -> {'cmd': 'save_place', 'name': 'my desk'}.
    """
    words = plain_words(question)
    if words in SCAN_WORDS:
        return {'cmd': 'scan_room'}
    if words in NEW_MAP_WORDS:
        return {'cmd': 'new_map'}
    for starts, command in ((SAVE_PLACE_STARTS, 'save_place'), (SAVE_MAP_STARTS, 'save_map'),
                            (LOAD_MAP_STARTS, 'load_map'), (FORGET_STARTS, 'remove_pin')):
        for start in starts:
            if words.startswith(start + ' '):
                name = words[len(start):].strip()
                if command == 'remove_pin':
                    name = re.sub(r'\s+(from|off) the map$', '', name)
                    name = re.sub(r'^(the|a|an)\s+', '', name)
                elif command != 'save_place':
                    name = re.sub(r'^(called|named)\s+', '', name)
                return {'cmd': command, 'name': name} if name else None
    return None


FIND_STARTS = ('where is', 'where are', 'wheres', 'where s', 'find', 'locate', 'look for',
               'can you see', 'do you see', 'show me')


def thing_asked(question):
    """Give the thing asked for: 'Where is my water bottle?' -> 'water bottle'; else ''."""
    words = plain_words(question)
    for start in FIND_STARTS:
        if words.startswith(start + ' '):
            return re.sub(r'^(the|a|an|my|me)\s+', '', words[len(start):].strip())
    return ''


def on_the_map(question, pins, pose):
    """
    Answer "where is X" from the map when the camera cannot see X: a sentence, or ''.

    pins: the map's pins, each {'name', 'x', 'y'} (Stage 3's /stage3/pins). pose: the
    device's (x, y, yaw) on the map, or None. The nearest pin whose name has the asked
    words in it (or the other way round) is used.
    """
    thing = thing_asked(question)
    if not thing or not pins:
        return ''
    asked = set(thing.split())
    matches = []
    for pin in pins:
        named = set(plain_words(str(pin.get('name', ''))).split())
        if named and (asked <= named or named <= asked):
            matches.append(pin)
    if not matches:
        return ''
    name = matches[0].get('name', thing)
    if pose is None:
        return "I can't see it from here, but %s is on the map. Say take me to %s" % (
            'the ' + name, 'the ' + name)
    pin = min(matches, key=lambda p: math.hypot(p['x'] - pose[0], p['y'] - pose[1]))
    name = pin.get('name', thing)
    gap = math.hypot(pin['x'] - pose[0], pin['y'] - pose[1])
    return "I can't see it from here, but the %s is on the map, about %d metre%s away. " \
           'Say take me to the %s' % (name, max(1, round(gap)),
                                      '' if max(1, round(gap)) == 1 else 's', name)


@dataclass
class Outcome:
    """The answer to one question."""

    question: str = ''          # the words, as heard or typed
    say: str = ''               # sentence to speak; '' = stay silent
    result: FindResult = None   # the finder's answer, None if it was never asked
    picture: object = None      # the photo, with the red box drawn on when found
    problem: str = ''           # for the log: what went wrong, if anything
    kind: str = 'find'          # 'find' = mark it in the picture, 'answer' = told in words,
    #                             'guide' = take me to a place, 'follow_up' = carry on (Stage 4)
    #                             'command' = a map command (save this place as ...)
    place: str = ''             # kind 'guide': the place asked for
    command: dict = None        # kind 'command': the line for /stage3/command (16 Nov)


def request_for(outcome):
    """Make the line for the guide node (Stage 4) after a question; None if nothing asked."""
    if not outcome.question:
        return None
    if outcome.kind == 'guide':
        return {'kind': 'guide', 'place': outcome.place, 'question': outcome.question}
    if outcome.kind == 'follow_up':
        return {'kind': 'follow_up', 'question': outcome.question}
    return {'kind': 'other', 'question': outcome.question}


def unreachable(error):
    """Return True if this error means the campus PC could not be reached."""
    return error.startswith(('cannot reach', 'unexpected reply'))


def answer_typed(question, image_bgr, ollama_url, model, finder=find, router=None,
                 replier=reply):
    """
    Answer a question given as text (typed, or already turned into words).

    router: a function that sorts the question into 'find' or 'answer' (finder.route).
    None = always find, as before (tracking uses this).
    """
    question = question.strip()
    if not question:
        return Outcome(say=NOT_HEARD, problem='empty question')
    place = guide_place(question)
    if place:                               # Stage 4: the guide node answers these
        return Outcome(question, kind='guide', place=place)
    if is_follow_up(question):
        return Outcome(question, kind='follow_up')
    command = spoken_command(question)
    if command:                             # the map manager or the labeller answers these
        return Outcome(question, kind='command', command=command)
    if image_bgr is None:
        return Outcome(question, NO_CAMERA, problem='no colour picture from the camera yet')
    image = np.asarray(image_bgr)
    kind, sorting_s = 'find', 0.0
    if router is not None:
        routed = router(question, url=ollama_url, model=model)
        if routed.error.startswith('cannot reach'):
            return Outcome(question, NO_HELPER, None, image, routed.error)
        kind, sorting_s = routed.kind, routed.seconds    # other errors: find, as before
    if kind == 'answer':
        return answer_in_words(question, image, ollama_url, model, replier, sorting_s)
    result = finder(image, question, url=ollama_url, model=model)
    result = replace(result, seconds=result.seconds + sorting_s)
    # 18 Nov (lab D1): "where is ESB117" found only "sign" (its words not read) and nothing was
    # said. A sign with no words read is not the named thing: "not found", then the map answers.
    unread = str(result.label).strip().lower() in UNREAD_SIGNS         and thing_asked(question) not in UNREAD_SIGNS
    if result.found and not unread:
        return Outcome(question, '', result, draw_box(image, result.box, result.label))
    if unreachable(result.error):
        return Outcome(question, NO_HELPER, result, image, result.error)
    # Not there, or an answer that made no sense: either way the user hears "not found".
    return Outcome(question, NOT_FOUND, result, image, result.error)


def answer_in_words(question, image, ollama_url, model, replier, sorting_s=0.0):
    """Get Qwen's sentence for a question that needs words back, with its box if any."""
    result = replier(image, question, url=ollama_url, model=model)
    result = replace(result, seconds=result.seconds + sorting_s)
    if result.say:
        picture = draw_box(image, result.box, result.label) if result.found else image
        return Outcome(question, result.say, result, picture, kind='answer')
    if unreachable(result.error):
        return Outcome(question, NO_HELPER, result, image, result.error, 'answer')
    # A reply with no sentence in it: the user hears "not found", as for finding.
    return Outcome(question, NOT_FOUND, result, image, result.error, 'answer')


def answer_recording(wav_bytes, image_bgr, speech_url, ollama_url, model,
                     speech=to_text, finder=find, router=None, replier=reply, on_heard=None):
    """
    Answer a spoken question: turn the recording into words, then answer those.

    on_heard(words) is called as soon as the words come back, before Qwen is asked, so the
    user can be told "Heard".
    """
    heard = speech(wav_bytes, url=speech_url)
    if heard.error:
        return Outcome(say=NO_HELPER, problem=heard.error)
    if not heard.text:
        return Outcome(say=NOT_HEARD, problem='no words in the recording')
    if on_heard is not None:
        on_heard(heard.text)
    return answer_typed(heard.text, image_bgr, ollama_url, model, finder, router, replier)
