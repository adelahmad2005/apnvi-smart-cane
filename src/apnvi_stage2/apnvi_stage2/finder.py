"""
The finder: one colour photo and one question in, a box (or "not found") out.

Plain Python, numpy and OpenCV only. Nothing from ROS, so it can be tested on its own,
the same way as Stage 1's hazard_check.py. The Stage 2 ROS program calls find().

It asks Qwen3-VL-4B, which runs under Ollama on the campus PC (decided 2026-10-08).
Ollama is a program that runs AI models and answers requests over the network.

How Qwen answers (from Qwen's own grounding examples): a short JSON list such as
  [{"bbox_2d": [x1, y1, x2, y2], "label": "exit sign"}]
where the numbers run from 0 to 1000 across the photo, whatever its real size
(0 = left or top edge, 1000 = right or bottom edge). We ask for [] when it is not there.

Questions that need words back (added 2026-10-26, Adel): route() first asks Qwen, with the
question only and no photo, whether the user wants a thing found (FIND) or something told to
them (ANSWER). reply() then asks for one short sentence to speak, such as a sign's words and
its arrow, plus a box on the thing the sentence is about:
  {"say": "Restroom, arrow pointing left", "bbox_2d": [x1, y1, x2, y2], "label": "sign"}
"""

import base64
from dataclasses import dataclass, replace
import json
import re
import time
import urllib.error
import urllib.request

import cv2
import numpy as np

# ---- Settings ----
OLLAMA_URL = 'http://localhost:11434'   # campus PC address goes here on Monday
# Qwen3-VL-4B, fast version without the thinking step (0.5 s against 3.2 s; Adel, 12 Oct)
MODEL = 'qwen3-vl:4b-instruct'
TIMEOUT_S = 30.0                        # give up waiting for an answer after this
KEEP_LOADED = '60m'                     # keep the model in memory between questions
MAX_SIDE_PX = 1280                      # shrink bigger photos first: faster, same box
JPEG_QUALITY = 90
BOX_SCALE = 1000                        # Qwen's box numbers run from 0 to this

ROUTE_TIMEOUT_S = 10.0                  # sorting the question is quick: give up sooner
MAX_SAY_CHARS = 200                     # longest sentence spoken, so a ramble gets cut short

BOX_COLOUR = (0, 0, 255)                # red, in OpenCV's blue-green-red order
BOX_THICKNESS = 4

PROMPT = (
    'A blind person asked: "{question}". '
    'Find the one thing in this photo that answers the question. It may be a sign, a '
    'room number, a symbol, a door or an object. '
    'Report its bbox coordinates in JSON format as '
    '[{{"bbox_2d": [x1, y1, x2, y2], "label": "short name"}}]. '
    'If it is not in the photo, reply with [] and nothing else.'
)

# Tracking asks for every match (e.g. both hands), so the follower can keep the one it is on
# instead of jumping to whichever one Qwen happens to name first (Adel, 13 Oct).
TRACK_PROMPT = (
    'A blind person asked: "{question}". '
    'Find every thing in this photo that answers the question. It may be a sign, a '
    'room number, a symbol, a door or an object. '
    'Report the bbox coordinates of each one in JSON format as '
    '[{{"bbox_2d": [x1, y1, x2, y2], "label": "short name"}}, ...]. '
    'If none is in the photo, reply with [] and nothing else.'
)

# Sorting the question, without the photo (Adel, 26 Oct: "the AI must be able to
# differentiate from the question itself").
ROUTE_PROMPT = (
    'A blind person pressed a button and asked: "{question}". '
    'Decide what they want. '
    'FIND: they want to know where a thing is, so it can be marked in the camera picture '
    '(for example "where is the exit", "find a water bottle", "where is room ESB2090"). '
    'ANSWER: they want something told to them in words (for example "what is the nearest '
    'sign", "read the sign", "what is in front of me", "is the door open", "how many '
    'chairs are there"). '
    'Reply with one word, FIND or ANSWER, and nothing else.'
)

# A question that needs words back: one short sentence to speak, and a box if it is about
# one thing in the photo.
REPLY_PROMPT = (
    'A blind person asked: "{question}". '
    'Answer from what you can see in this photo, in one short sentence that will be read '
    'out to them, at most 20 words. Speak to them directly and do not mention the photo. '
    'If you read a sign, say its words and, if it has an arrow, which way the arrow points: '
    'left, right or straight ahead. '
    'If you cannot see enough to answer, say so in the sentence. '
    'Reply in JSON format as {{"say": "your sentence", "bbox_2d": [x1, y1, x2, y2], '
    '"label": "short name"}}, where bbox_2d is the thing your answer is about. '
    'If your answer is not about one thing in the photo, leave out bbox_2d and label.'
)


@dataclass
class FindResult:
    """What find() gives back."""

    found: bool                 # True = there is a box
    box: tuple = None           # (left, top, right, bottom) in pixels of the photo given
    label: str = ''             # what Qwen called it, e.g. 'exit sign'
    seconds: float = 0.0        # how long the question took
    error: str = ''             # filled in when something went wrong (PC not reachable...)
    answer: str = ''            # Qwen's raw answer, for checking by eye
    boxes: tuple = ()           # every box Qwen gave, in pixels (box is the first of them)
    say: str = ''               # reply() only: the sentence to speak ('' = none)


@dataclass
class RouteResult:
    """What route() gives back."""

    kind: str = 'find'          # 'find' = mark it in the picture, 'answer' = tell them in words
    seconds: float = 0.0
    error: str = ''


def make_prompt(question, every=False, template=None):
    """Wrap the user's question in the instruction Qwen gets (every=True: all matches)."""
    if template is None:
        template = TRACK_PROMPT if every else PROMPT
    return template.format(question=question.strip().replace('"', "'"))


def photo_to_base64(image_bgr):
    """Shrink the photo if it is big, turn it into a JPEG and then into text for sending."""
    height, width = image_bgr.shape[:2]
    scale = MAX_SIDE_PX / max(height, width)
    if scale < 1.0:
        image_bgr = cv2.resize(image_bgr, (round(width * scale), round(height * scale)),
                               interpolation=cv2.INTER_AREA)
    ok, jpeg = cv2.imencode('.jpg', image_bgr, [cv2.IMWRITE_JPEG_QUALITY, JPEG_QUALITY])
    if not ok:
        raise ValueError('could not turn the photo into a JPEG')
    return base64.b64encode(jpeg.tobytes()).decode('ascii')


def ask_qwen(image_bgr, question, url=OLLAMA_URL, model=MODEL, timeout_s=TIMEOUT_S,
             every=False, template=None):
    """
    Send the photo and the question to Ollama and return Qwen's answer as text.

    image_bgr None sends the question alone, with no photo (used by route()).
    template picks another instruction, such as ROUTE_PROMPT or REPLY_PROMPT.
    """
    message = {'role': 'user', 'content': make_prompt(question, every, template)}
    if image_bgr is not None:
        message['images'] = [photo_to_base64(image_bgr)]
    request = {
        'model': model,
        'messages': [message],
        'stream': False,                     # one complete answer, not word by word
        'keep_alive': KEEP_LOADED,
        'options': {'temperature': 0},       # same photo + question = same answer
    }
    data = json.dumps(request).encode('utf-8')
    http = urllib.request.Request(url.rstrip('/') + '/api/chat', data=data,
                                  headers={'Content-Type': 'application/json'})
    with urllib.request.urlopen(http, timeout=timeout_s) as response:
        reply = json.loads(response.read().decode('utf-8'))
    return reply['message']['content']


def read_boxes(answer):
    """
    Pull the boxes out of Qwen's answer.

    Returns a list of (x1, y1, x2, y2, label) on Qwen's 0-1000 scale, [] for "not
    there", or None if the answer cannot be understood at all.
    """
    text = re.sub(r'<think>.*?</think>', '', answer, flags=re.DOTALL)   # thinking, if any
    fenced = re.search(r'```(?:json)?\s*(.*?)```', text, flags=re.DOTALL)
    if fenced:
        text = fenced.group(1)
    start = min([i for i in (text.find('['), text.find('{')) if i >= 0], default=-1)
    if start < 0:
        return None
    try:
        data, _ = json.JSONDecoder().raw_decode(text[start:])
    except json.JSONDecodeError:
        return None
    if isinstance(data, dict):                 # a single box without the list around it
        data = [data]
    if not isinstance(data, list):
        return None
    boxes = []
    for item in data:
        numbers = item.get('bbox_2d') if isinstance(item, dict) else None
        if not isinstance(numbers, (list, tuple)) or len(numbers) != 4:
            continue
        try:
            x1, y1, x2, y2 = (float(n) for n in numbers)
        except (TypeError, ValueError):
            continue
        boxes.append((x1, y1, x2, y2, str(item.get('label', ''))))
    if data and not boxes:
        return None                            # there was something, but not a box
    return boxes


def read_items(answer):
    """
    Pull the JSON list out of Qwen's answer as it stands: a list of dicts, or None.

    Like read_boxes, but every key is kept (the labeller's items also carry "kind").
    """
    text = re.sub(r'<think>.*?</think>', '', answer, flags=re.DOTALL)
    fenced = re.search(r'```(?:json)?\s*(.*?)```', text, flags=re.DOTALL)
    if fenced:
        text = fenced.group(1)
    start = min([i for i in (text.find('['), text.find('{')) if i >= 0], default=-1)
    if start < 0:
        return None
    try:
        data, _ = json.JSONDecoder().raw_decode(text[start:])
    except json.JSONDecodeError:
        return None
    if isinstance(data, dict):
        data = [data]
    if not isinstance(data, list):
        return None
    return [item for item in data if isinstance(item, dict)]


def read_route(answer):
    """Return 'answer' if Qwen said ANSWER (and not FIND), otherwise 'find', as before."""
    text = re.sub(r'<think>.*?</think>', '', answer, flags=re.DOTALL).upper()
    if 'ANSWER' in text and 'FIND' not in text:
        return 'answer'
    return 'find'


def read_reply(answer):
    """
    Pull the sentence, and the box if there is one, out of Qwen's reply.

    Returns (sentence, (x1, y1, x2, y2, label) or None) on Qwen's 0-1000 scale, or None if
    there is no sentence in it. A reply in plain words, without JSON, is taken as the
    sentence.
    """
    text = re.sub(r'<think>.*?</think>', '', answer, flags=re.DOTALL).strip()
    fenced = re.search(r'```(?:json)?\s*(.*?)```', text, flags=re.DOTALL)
    if fenced:
        text = fenced.group(1).strip()
    start = min([i for i in (text.find('['), text.find('{')) if i >= 0], default=-1)
    if start < 0:
        return (text, None) if text else None          # plain words, no JSON
    try:
        data, _ = json.JSONDecoder().raw_decode(text[start:])
    except json.JSONDecodeError:
        return None
    if isinstance(data, list):                 # the reply wrapped in a list
        data = data[0] if data else None
    if not isinstance(data, dict):
        return None
    say = data.get('say')
    if not isinstance(say, str) or not say.strip():
        return None
    boxes = read_boxes(json.dumps([data])) if 'bbox_2d' in data else None
    return say.strip(), (boxes[0] if boxes else None)


def to_pixels(box, width, height):
    """Turn one box on Qwen's 0-1000 scale into whole pixels of a width x height photo."""
    x1, y1, x2, y2 = box[:4]
    left, right = sorted((x1, x2))
    top, bottom = sorted((y1, y2))

    def scale(value, size):
        """Scale one number and keep it inside the photo."""
        return int(round(min(max(value, 0.0), BOX_SCALE) / BOX_SCALE * (size - 1)))

    return scale(left, width), scale(top, height), scale(right, width), scale(bottom, height)


def find(image_bgr, question, url=OLLAMA_URL, model=MODEL, timeout_s=TIMEOUT_S, every=False):
    """
    Ask where the thing in the question is, in this photo.

    Never raises: problems come back as FindResult(found=False, error=...).
    If Qwen gives several boxes, box is the first one; boxes has them all.
    every=True asks Qwen for every match instead of one (used while tracking).
    """
    start = time.monotonic()
    image = np.asarray(image_bgr)
    try:
        answer = ask_qwen(image, question, url, model, timeout_s, every)
    except (urllib.error.URLError, TimeoutError, OSError) as problem:
        return FindResult(False, seconds=time.monotonic() - start,
                          error=f'cannot reach Qwen at {url}: {problem}')
    except (KeyError, ValueError) as problem:
        return FindResult(False, seconds=time.monotonic() - start,
                          error=f'unexpected reply from Ollama: {problem}')
    seconds = time.monotonic() - start
    boxes = read_boxes(answer)
    if boxes is None:
        return FindResult(False, seconds=seconds, answer=answer,
                          error='could not understand the answer')
    if not boxes:
        return FindResult(False, seconds=seconds, answer=answer)
    height, width = image.shape[:2]
    pixels = tuple(to_pixels(box, width, height) for box in boxes)
    return FindResult(True, pixels[0], boxes[0][4], seconds, '', answer, pixels)


def route(question, url=OLLAMA_URL, model=MODEL, timeout_s=ROUTE_TIMEOUT_S):
    """
    Ask Qwen, without the photo, whether the user wants a thing found or told.

    Never raises: problems come back as RouteResult('find', error=...).
    """
    start = time.monotonic()
    try:
        answer = ask_qwen(None, question, url, model, timeout_s, template=ROUTE_PROMPT)
    except (urllib.error.URLError, TimeoutError, OSError) as problem:
        return RouteResult(seconds=time.monotonic() - start,
                           error=f'cannot reach Qwen at {url}: {problem}')
    except (KeyError, ValueError) as problem:
        return RouteResult(seconds=time.monotonic() - start,
                           error=f'unexpected reply from Ollama: {problem}')
    return RouteResult(read_route(answer), time.monotonic() - start)


def reply(image_bgr, question, url=OLLAMA_URL, model=MODEL, timeout_s=TIMEOUT_S):
    """
    Ask for one short sentence that answers the question from this photo.

    Never raises: problems come back as FindResult(found=False, error=...).
    found and box are set when the sentence is about one thing Qwen boxed.
    """
    start = time.monotonic()
    image = np.asarray(image_bgr)
    try:
        answer = ask_qwen(image, question, url, model, timeout_s, template=REPLY_PROMPT)
    except (urllib.error.URLError, TimeoutError, OSError) as problem:
        return FindResult(False, seconds=time.monotonic() - start,
                          error=f'cannot reach Qwen at {url}: {problem}')
    except (KeyError, ValueError) as problem:
        return FindResult(False, seconds=time.monotonic() - start,
                          error=f'unexpected reply from Ollama: {problem}')
    seconds = time.monotonic() - start
    sentence = read_reply(answer)
    if sentence is None:
        return FindResult(False, seconds=seconds, answer=answer,
                          error='could not understand the answer')
    say, box = sentence
    result = FindResult(False, seconds=seconds, answer=answer, say=say[:MAX_SAY_CHARS])
    if box is None:
        return result
    height, width = image.shape[:2]
    pixels = to_pixels(box, width, height)
    return replace(result, found=True, box=pixels, label=box[4], boxes=(pixels,))


def draw_box(image_bgr, box, label=''):
    """Return a copy of the photo with a red box (and its label) drawn on it."""
    picture = np.array(image_bgr, copy=True)
    left, top, right, bottom = box
    cv2.rectangle(picture, (left, top), (right, bottom), BOX_COLOUR, BOX_THICKNESS)
    if label:
        y = top - 10 if top > 30 else bottom + 30
        cv2.putText(picture, label, (left, y), cv2.FONT_HERSHEY_SIMPLEX, 0.9,
                    BOX_COLOUR, 2, cv2.LINE_AA)
    return picture
