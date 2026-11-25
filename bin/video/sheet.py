#!/usr/bin/env python3
"""Contact sheet: N evenly spaced, timestamped frames of a video tiled into one JPEG for the Read tool.

Usage: sheet.py VIDEO [-o OUT.jpg] [-n 12] [--cols 4] [--width 480] [--start S] [--end E]
Twelve 480 px frames in a 4x3 grid make a 1920x1080 sheet, which the Read tool shows in one image.
"""
import argparse
import math
import os
import sys

import cv2
import numpy as np


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("video")
    ap.add_argument("-o", "--out")
    ap.add_argument("-n", type=int, default=12)
    ap.add_argument("--cols", type=int, default=4)
    ap.add_argument("--width", type=int, default=480)
    ap.add_argument("--start", type=float, default=0.0)
    ap.add_argument("--end", type=float)
    a = ap.parse_args()

    cap = cv2.VideoCapture(a.video)
    if not cap.isOpened():
        sys.exit(f"cannot open {a.video}")
    fps = cap.get(cv2.CAP_PROP_FPS) or 30.0
    duration = int(cap.get(cv2.CAP_PROP_FRAME_COUNT)) / fps
    end = a.end if a.end is not None else duration
    times = np.linspace(a.start, max(a.start, end - 1.5 / fps), a.n)

    tiles = []
    for t in times:
        cap.set(cv2.CAP_PROP_POS_MSEC, float(t) * 1000.0)
        ok, frame = cap.read()
        if not ok:
            break
        h, w = frame.shape[:2]
        frame = cv2.resize(frame, (a.width, int(h * a.width / w)), interpolation=cv2.INTER_AREA)
        label = f"t={t:5.2f}s"
        cv2.rectangle(frame, (0, 0), (10 + 12 * len(label), 28), (0, 0, 0), -1)
        cv2.putText(frame, label, (5, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 255), 2, cv2.LINE_AA)
        tiles.append(frame)
    if not tiles:
        sys.exit("no frames read")

    th, tw = tiles[0].shape[:2]
    rows = math.ceil(len(tiles) / a.cols)
    sheet = np.zeros((rows * th, a.cols * tw, 3), np.uint8)
    for i, tile in enumerate(tiles):
        r, c = divmod(i, a.cols)
        sheet[r * th:(r + 1) * th, c * tw:(c + 1) * tw] = tile
    out = a.out or os.path.splitext(a.video)[0] + "_sheet.jpg"
    cv2.imwrite(out, sheet, [cv2.IMWRITE_JPEG_QUALITY, 85])
    print(f"{out}  {len(tiles)} frames {times[0]:.2f}-{times[len(tiles)-1]:.2f}s of {duration:.2f}s, "
          f"{sheet.shape[1]}x{sheet.shape[0]} px, read left-to-right then top-to-bottom")


if __name__ == "__main__":
    main()
