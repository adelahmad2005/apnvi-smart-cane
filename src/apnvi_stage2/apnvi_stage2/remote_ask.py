"""
One trip to the campus PC for a whole question (remote link; Adel, 18 Nov).

Over the internet every call to the PC costs about 0.4 to 0.5 s more (it goes through Cloudflare
in Europe), and a spoken question made four: speech to text, sorting, Qwen, the voice. With the
PC's front door (ai_server/front_door.py) the Jetson sends the recording (or the typed words) and
the photo once; the PC runs this very code (answer_recording / answer_typed, so every decision is
the same as on the hotspot) next to its servers, and sends back the outcome and, when there is a
sentence, its voice, squeezed. One trip instead of four.

Plain Python, nothing from ROS: the front door on the PC imports it too.

  Jetson: ask_front_door(door, image, model, wav=... or text=..., voice=True)
          -> (Outcome, the sentence's WAV or None). Raises NoAsk when the front door has no /ask,
          so the caller asks the old way.
  PC:     answer_on_pc(request, speech_url, ollama_url, voice) -> the reply (a dict for JSON)

The picture is not sent back: the PC says 'none', 'photo' or 'boxed', and the Jetson draws the box
on its own photo, the same as answer_typed does.
"""

import base64
import json
import urllib.error
import urllib.request

import cv2
import numpy as np

from .asking import answer_recording, answer_typed, NO_HELPER, Outcome
from .finder import draw_box, find, FindResult, MODEL, reply, route
from .speech import to_text
from .voice import unpack_opus

ASK_TIMEOUT_S = 60.0        # speech 15 s + sorting 10 s + Qwen 30 s at most, plus the trip
PHOTO_QUALITY = 95          # sent at full size, so Qwen's box fits the Jetson's own photo


class NoAsk(Exception):
    """The front door has no /ask (an older one, or no numpy/OpenCV on the PC)."""


# ---- the photo and the outcome, as JSON ----

def photo_to_text(image_bgr):
    """Turn the photo, full size, into a JPEG in base64; None for no photo."""
    if image_bgr is None:
        return None
    ok, jpeg = cv2.imencode('.jpg', np.asarray(image_bgr),
                            [cv2.IMWRITE_JPEG_QUALITY, PHOTO_QUALITY])
    if not ok:
        raise ValueError('could not turn the photo into a JPEG')
    return base64.b64encode(jpeg.tobytes()).decode('ascii')


def photo_from_text(text):
    """Back from photo_to_text: the photo (BGR), or None."""
    if not text:
        return None
    image = cv2.imdecode(np.frombuffer(base64.b64decode(text), np.uint8), cv2.IMREAD_COLOR)
    if image is None:
        raise ValueError('the photo could not be read')
    return image


def outcome_to_dict(outcome, image):
    """Turn the outcome into plain JSON. `image`: the photo it was given (to see if boxed)."""
    if outcome.picture is None:
        picture = 'none'
    elif outcome.picture is image:
        picture = 'photo'
    else:
        picture = 'boxed'               # draw_box made a copy with the red box on it
    result = outcome.result
    return {
        'question': outcome.question, 'say': outcome.say, 'problem': outcome.problem,
        'kind': outcome.kind, 'place': outcome.place, 'command': outcome.command,
        'picture': picture,
        'result': None if result is None else {
            'found': bool(result.found), 'box': list(result.box) if result.box else None,
            'label': result.label, 'seconds': result.seconds, 'error': result.error,
            'answer': result.answer, 'boxes': [list(box) for box in result.boxes],
            'say': result.say},
    }


def outcome_from_dict(data, image):
    """Back from outcome_to_dict, with the picture made from the Jetson's own photo."""
    got = data.get('result')
    result = None if got is None else FindResult(
        found=bool(got.get('found')), box=tuple(got['box']) if got.get('box') else None,
        label=got.get('label', ''), seconds=float(got.get('seconds', 0.0)),
        error=got.get('error', ''), answer=got.get('answer', ''),
        boxes=tuple(tuple(box) for box in got.get('boxes') or ()), say=got.get('say', ''))
    picture = None
    if data.get('picture') == 'photo':
        picture = image
    elif data.get('picture') == 'boxed' and image is not None and result is not None \
            and result.box:
        picture = draw_box(image, result.box, result.label)
    return Outcome(question=data.get('question', ''), say=data.get('say', ''), result=result,
                   picture=picture, problem=data.get('problem', ''),
                   kind=data.get('kind', 'find'), place=data.get('place', ''),
                   command=data.get('command'))


# ---- the PC's side ----

def answer_on_pc(request, speech_url, ollama_url, voice=None, speech=to_text, finder=find,
                 router=route, replier=reply):
    """
    Answer one question next to the PC's servers, the same way the Jetson would.

    request: {'wav': base64 recording, or 'text': typed words; 'photo'; 'model'; 'voice'}
    voice(sentence, squeezed) gives the sentence's sound (bytes), or None. Called only when the
    Jetson asked for the voice ('ogg' = it can unpack Opus, 'wav' = it cannot).
    """
    image = photo_from_text(request.get('photo'))
    model = request.get('model') or MODEL
    if request.get('wav'):
        outcome = answer_recording(base64.b64decode(request['wav']), image, speech_url,
                                   ollama_url, model, speech=speech, finder=finder,
                                   router=router, replier=replier)
    else:
        outcome = answer_typed(request.get('text', ''), image, ollama_url, model,
                               finder=finder, router=router, replier=replier)
    answer = outcome_to_dict(outcome, image)
    wanted = request.get('voice')
    if voice is not None and wanted in ('ogg', 'wav') and outcome.say:
        sound = voice(outcome.say, wanted == 'ogg')
        if sound:
            answer['voice'] = base64.b64encode(sound).decode('ascii')
    return answer


# ---- the Jetson's side ----

def ask_front_door(door, image_bgr, model=MODEL, wav=None, text=None, voice=False,
                   can_unpack=None, unpack=unpack_opus, timeout_s=ASK_TIMEOUT_S):
    """
    Ask a whole question in one trip. Returns (Outcome, the sentence's WAV bytes or None).

    door: the front door address with its token (https://<tunnel>/k/<token>).
    Raises NoAsk if the front door has no /ask; the caller then asks in four trips.
    """
    if can_unpack is None:
        import shutil
        can_unpack = shutil.which('gst-launch-1.0') is not None
    image = None if image_bgr is None else np.asarray(image_bgr)
    body = {'model': model, 'photo': photo_to_text(image),
            'voice': ('ogg' if can_unpack else 'wav') if voice else ''}
    if wav is not None:
        body['wav'] = base64.b64encode(wav).decode('ascii')
    else:
        body['text'] = text or ''
    request = urllib.request.Request(door.rstrip('/') + '/ask', data=json.dumps(body).encode(),
                                     headers={'Content-Type': 'application/json'})
    try:
        with urllib.request.urlopen(request, timeout=timeout_s) as answer:
            data = json.loads(answer.read())
    except urllib.error.HTTPError as error:
        if error.code == 404:
            raise NoAsk('the front door has no /ask') from None
        problem = 'cannot reach the campus PC through the front door: HTTP %d %s' % (
            error.code, error.read()[:200])
        return Outcome(say=NO_HELPER, problem=problem), None
    except (urllib.error.URLError, OSError, ValueError) as problem:     # also a timeout
        reason = getattr(problem, 'reason', problem)
        return Outcome(say=NO_HELPER, problem='cannot reach the campus PC through the front '
                                              'door: %s' % reason), None
    outcome = outcome_from_dict(data, image)
    sound = base64.b64decode(data['voice']) if data.get('voice') else None
    if sound and sound.startswith(b'OggS'):
        try:
            sound = unpack(sound)
        except Exception:               # cannot unpack: the voice is fetched the normal way
            sound = None
    if sound and not sound.startswith(b'RIFF'):
        sound = None
    return outcome, sound
