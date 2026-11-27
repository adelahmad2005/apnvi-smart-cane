#!/usr/bin/env python3
"""
Try the labeller's question on photos: what does Qwen list as signs and landmarks?

Run it before the labelling lap, on a few photos of the room taken at camera
height, to see what the AI names and how long one look takes. Needs only Python with numpy
and OpenCV. No ROS.

Usage (from the Stage 2 code folder):
  python3 tools/try_labels.py PHOTO_OR_FOLDER --url http://CAMPUS_PC_IP:11434
Results: a table on screen, and boxed copies in PHOTO_FOLDER/labels/.
"""

import argparse
import os
import sys
import time

import cv2

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..', 'src', 'apnvi_stage2'))
from apnvi_stage2.finder import ask_qwen, draw_box, MODEL, OLLAMA_URL  # noqa: E402
from apnvi_stage2.labels import LABEL_PROMPT, read_labels  # noqa: E402

PHOTO_TYPES = ('.png', '.jpg', '.jpeg', '.bmp')


def main():
    """Ask Qwen for the landmarks in each photo and print what the labeller would keep."""
    parser = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    parser.add_argument('photos', help='one photo, or a folder of photos')
    parser.add_argument('--url', default=OLLAMA_URL, help='where Qwen runs')
    parser.add_argument('--model', default=MODEL)
    args = parser.parse_args()
    if os.path.isdir(args.photos):
        folder = args.photos
        names = sorted(n for n in os.listdir(folder) if n.lower().endswith(PHOTO_TYPES))
    else:
        folder, name = os.path.split(args.photos)
        names = [name]
    out = os.path.join(folder or '.', 'labels')
    os.makedirs(out, exist_ok=True)
    for name in names:
        image = cv2.imread(os.path.join(folder or '.', name))
        if image is None:
            print('%s: cannot read it' % name)
            continue
        start = time.monotonic()
        try:
            answer = ask_qwen(image, '', args.url, args.model, 60.0, template=LABEL_PROMPT)
        except OSError as problem:
            sys.exit('cannot reach Qwen at %s: %s' % (args.url, problem))
        seconds = time.monotonic() - start
        height, width = image.shape[:2]
        labels = read_labels(answer, width, height)
        if labels is None:
            print('%s: %.1f s, could not understand the answer: %r' % (
                name, seconds, answer[:200]))
            continue
        print('%s: %.1f s, kept %d: %s' % (
            name, seconds, len(labels),
            ', '.join('%s (%s)' % (label, kind) for label, kind, _box in labels) or '-'))
        boxed = image
        for label, _kind, box in labels:
            boxed = draw_box(boxed, box, label)
        cv2.imwrite(os.path.join(out, name), boxed)
    print('boxed copies are in', out)


if __name__ == '__main__':
    main()
