#!/usr/bin/env python3
"""Motion trail for a static-camera video: one image showing where the moving object was over time,
plus a motion-energy timeline. Needs no markers, only a camera that does not move.

Usage: trail.py VIDEO [-o OUTDIR] [--samples 24] [--thresh 35] [--min-area 150]
Writes OUTDIR/trail.png (background + time-coloured onion skin + centroid path) and OUTDIR/motion.png
(fraction of changed pixels per frame). Prints when motion started/stopped and the centroid path.
"""
import argparse
import os
import sys

import cv2
import numpy as np
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402


def read_frames(path, max_frames=400):
    cap = cv2.VideoCapture(path)
    if not cap.isOpened():
        sys.exit(f"cannot open {path}")
    fps = cap.get(cv2.CAP_PROP_FPS) or 30.0
    n = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    step = max(1, n // max_frames)
    frames, times, i = [], [], 0
    while True:
        ok, f = cap.read()
        if not ok:
            break
        if i % step == 0:
            frames.append(f)
            times.append(i / fps)
        i += 1
    return frames, np.array(times), fps


def foreground(frame, bg, thresh, min_area):
    diff = cv2.absdiff(frame, bg).max(axis=2)
    mask = (diff > thresh).astype(np.uint8) * 255
    k3, k7 = np.ones((3, 3), np.uint8), np.ones((7, 7), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, k3)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k7)
    n, labels, stats, cents = cv2.connectedComponentsWithStats(mask)
    if n <= 1:
        return None, None
    j = 1 + np.argmax(stats[1:, cv2.CC_STAT_AREA])
    if stats[j, cv2.CC_STAT_AREA] < min_area:
        return None, None
    return labels == j, cents[j]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("video")
    ap.add_argument("-o", "--out")
    ap.add_argument("--samples", type=int, default=24)
    ap.add_argument("--thresh", type=int, default=35)
    ap.add_argument("--min-area", type=int, default=150)
    a = ap.parse_args()
    out = a.out or os.path.splitext(a.video)[0] + "_watch"
    os.makedirs(out, exist_ok=True)

    frames, times, fps = read_frames(a.video)
    gray = [cv2.cvtColor(f, cv2.COLOR_BGR2GRAY) for f in frames]
    # change versus ~0.5 s earlier, so slow continuous motion registers as well as stepped motion
    lag = max(1, int(round(0.5 * (len(times) - 1) / max(times[-1], 1e-6))))
    energy = np.array([0.0] * lag + [float(np.mean(cv2.absdiff(gray[i], gray[i - lag]) > 20)) for i in range(lag, len(gray))])
    noise = float(np.percentile(energy[lag:], 10)) if len(energy) > lag else 0.0
    thr = max(0.0015, 3 * noise + 0.001)
    moving = np.where(energy > thr)[0]
    t_start, t_stop = (times[max(0, moving[0] - lag)], times[moving[-1]]) if len(moving) else (None, None)

    idx = np.linspace(0, len(frames) - 1, min(25, len(frames))).astype(int)
    bg = np.median(np.stack([frames[i] for i in idx]).astype(np.float32), axis=0).astype(np.uint8)

    if t_start is not None:
        lo, hi = max(0, t_start - 0.3), min(times[-1], t_stop + 0.3)
    else:
        lo, hi = times[0], times[-1]
    sample_idx = np.unique(np.searchsorted(times, np.linspace(lo, hi, a.samples)).clip(0, len(times) - 1))
    cmap = plt.get_cmap("turbo")
    comp = bg.copy()
    path = []
    for k, i in enumerate(sample_idx):
        mask, cent = foreground(frames[i], bg, a.thresh, a.min_area)
        if mask is None:
            continue
        rgb = np.array(cmap(k / max(1, len(sample_idx) - 1))[:3]) * 255
        color = rgb[::-1]
        comp[mask] = (0.7 * frames[i][mask] + 0.3 * color).astype(np.uint8)
        path.append((times[i], cent, color))
    for j in range(1, len(path)):
        p0, p1 = tuple(path[j - 1][1].astype(int)), tuple(path[j][1].astype(int))
        cv2.line(comp, p0, p1, path[j][2].tolist(), 2, cv2.LINE_AA)
    for j, (t, cent, color) in enumerate(path):
        p = tuple(cent.astype(int))
        cv2.circle(comp, p, 5, color.tolist(), -1, cv2.LINE_AA)
        if j == 0 or j == len(path) - 1 or j % 4 == 0:
            cv2.putText(comp, f"{t:.1f}s", (p[0] + 7, p[1] - 7), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 3, cv2.LINE_AA)
            cv2.putText(comp, f"{t:.1f}s", (p[0] + 7, p[1] - 7), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 0), 1, cv2.LINE_AA)
    cv2.putText(comp, "motion trail: blue=early, red=late", (10, comp.shape[0] - 12), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2, cv2.LINE_AA)
    cv2.imwrite(os.path.join(out, "trail.png"), comp)

    fig, ax = plt.subplots(figsize=(10, 3))
    ax.plot(times, energy * 100, lw=1.2)
    ax.axhline(thr * 100, color="r", ls="--", lw=0.8, label="motion threshold")
    if t_start is not None:
        ax.axvspan(t_start, t_stop, color="orange", alpha=0.2, label=f"motion {t_start:.2f}-{t_stop:.2f}s")
    ax.set_xlabel("time (s)")
    ax.set_ylabel("% pixels changed vs 0.5 s earlier")
    ax.set_title(os.path.basename(a.video))
    ax.legend(loc="upper right")
    fig.tight_layout()
    fig.savefig(os.path.join(out, "motion.png"), dpi=110)

    if t_start is None:
        print("motion: none detected (change versus 0.5 s earlier stayed at the noise floor)")
    else:
        print(f"motion: {t_start:.2f}s -> {t_stop:.2f}s ({t_stop - t_start:.2f}s moving of {times[-1]:.2f}s)")
    if len(path) >= 2:
        pts = np.array([p[1] for p in path])
        seg = np.linalg.norm(np.diff(pts, axis=0), axis=1)
        net = pts[-1] - pts[0]
        print(f"trail: {len(path)} samples, centroid start ({pts[0][0]:.0f},{pts[0][1]:.0f})px -> end ({pts[-1][0]:.0f},{pts[-1][1]:.0f})px, "
              f"path {seg.sum():.0f}px, net {np.linalg.norm(net):.0f}px (dx={net[0]:+.0f}, dy={net[1]:+.0f}; image y grows downward)")
    else:
        print("trail: moving object not separable from background (static scene, or too small)")
    print(f"images: {out}/trail.png {out}/motion.png")


if __name__ == "__main__":
    main()
