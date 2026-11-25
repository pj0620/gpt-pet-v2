#!/usr/bin/env python3
"""ArUco robot tracker for a static camera that sees a floor with reference markers.

Floor layout JSON (metres, any frame you like; the floor markers define it):
  {"markers": {"10": [0.5, 0.5], "11": [3.5, 0.5], "12": [3.5, 2.5], "13": [0.5, 2.5]},
   "robot_marker_id": 0, "robot_marker_height_m": 0.2}
Robot marker: 4x4_50 dictionary, mounted flat with the marker image's top edge facing forward.

Usage: track.py VIDEO --floor floor.json [-o OUTDIR] [--hfov 60] [--step 1]
Writes OUTDIR/track.csv (t,x,y,yaw_deg), summary.json and track.png; prints the summary.
--hfov is the camera's horizontal field of view in degrees; it only matters for the marker-height
correction (an elevated marker looks farther from the camera's foot point than it is).
"""
import argparse
import csv
import json
import math
import os
import sys

import cv2
import numpy as np
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

DICT = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("video")
    ap.add_argument("--floor", required=True)
    ap.add_argument("-o", "--out")
    ap.add_argument("--hfov", type=float, default=60.0)
    ap.add_argument("--step", type=int, default=1)
    a = ap.parse_args()
    out = a.out or os.path.splitext(a.video)[0] + "_watch"
    os.makedirs(out, exist_ok=True)
    layout = json.load(open(a.floor))
    floor_ids = {int(k): np.array(v, float) for k, v in layout["markers"].items()}
    robot_id = int(layout.get("robot_marker_id", 0))
    h_robot = float(layout.get("robot_marker_height_m", 0.0))

    det = cv2.aruco.ArucoDetector(DICT, cv2.aruco.DetectorParameters())
    cap = cv2.VideoCapture(a.video)
    if not cap.isOpened():
        sys.exit(f"cannot open {a.video}")
    fps = cap.get(cv2.CAP_PROP_FPS) or 30.0
    seen_floor = {k: [] for k in floor_ids}
    robot_obs = []          # (t, corners(4,2))
    nframes = 0
    w = h = None
    i = 0
    while True:
        ok, frame = cap.read()
        if not ok:
            break
        if i % a.step == 0:
            if w is None:
                h, w = frame.shape[:2]
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            corners, ids, _ = det.detectMarkers(gray)
            nframes += 1
            if ids is not None:
                for c, mid in zip(corners, ids.flatten()):
                    c = c.reshape(4, 2)
                    if int(mid) in seen_floor:
                        seen_floor[int(mid)].append(c.mean(axis=0))
                    elif int(mid) == robot_id:
                        robot_obs.append((i / fps, c))
        i += 1

    found = {k: np.median(np.array(v), axis=0) for k, v in seen_floor.items() if len(v) >= 3}
    if len(found) < 4:
        sys.exit(f"need 4 floor markers, found {sorted(found)} (looking for {sorted(floor_ids)})")
    ids_sorted = sorted(found)
    img_pts = np.array([found[k] for k in ids_sorted], np.float32)
    world_pts = np.array([floor_ids[k] for k in ids_sorted], np.float32)
    H, _ = cv2.findHomography(img_pts, world_pts)

    # camera centre from the floor markers with an approximate pinhole model, for the height correction
    fx = (w / 2) / math.tan(math.radians(a.hfov) / 2)
    K = np.array([[fx, 0, w / 2], [0, fx, h / 2], [0, 0, 1]], float)
    obj = np.column_stack([world_pts, np.zeros(len(world_pts))]).astype(np.float32)
    ok, rvec, tvec = cv2.solvePnP(obj, img_pts, K, None, flags=cv2.SOLVEPNP_IPPE)
    Rm, _ = cv2.Rodrigues(rvec)
    C = (-Rm.T @ tvec).flatten()
    scale = (C[2] - h_robot) / C[2] if ok and C[2] > h_robot > 0 else 1.0

    rows = []
    for t, c in robot_obs:
        wc = cv2.perspectiveTransform(c.reshape(1, 4, 2).astype(np.float32), H).reshape(4, 2)
        centre = wc.mean(axis=0)
        if scale != 1.0:
            centre = C[:2] + (centre - C[:2]) * scale
        fwd = (wc[0] + wc[1]) / 2 - (wc[3] + wc[2]) / 2
        rows.append((t, centre[0], centre[1], math.atan2(fwd[1], fwd[0])))
    if len(rows) < 2:
        sys.exit(f"robot marker id {robot_id} seen in {len(rows)} frames; cannot track")

    T = np.array([r[0] for r in rows])
    P = np.array([[r[1], r[2]] for r in rows])
    yaw = np.unwrap([r[3] for r in rows])
    rate = len(T) / max(T[-1] - T[0], 1e-6)            # observations per second
    win = max(3, int(round(0.33 * rate)) | 1)           # ~1/3 s smoothing window, odd
    lag = max(1, int(round(0.5 * rate)))                # ~1/2 s baseline for speeds

    def smooth(v):
        pad = np.pad(v, win // 2, mode="edge")
        return np.convolve(pad, np.ones(win) / win, mode="valid")

    Ps = np.column_stack([smooth(P[:, 0]), smooth(P[:, 1])])
    yaws = smooth(yaw)
    dT = np.maximum(T[lag:] - T[:-lag], 1e-6)
    speed = np.linalg.norm(Ps[lag:] - Ps[:-lag], axis=1) / dT          # index i -> interval [i, i+lag]
    yaw_rate = np.abs(yaws[lag:] - yaws[:-lag]) / dT
    moving = (speed > 0.04) | (yaw_rate > math.radians(4))
    mi = np.where(moving)[0]
    path_len = float(np.linalg.norm(np.diff(Ps[::lag], axis=0), axis=1).sum())
    summary = dict(
        frames_analysed=nframes, robot_marker_detections=len(rows), detection_rate=round(len(rows) / max(1, nframes), 3),
        floor_markers_used=ids_sorted, camera_height_est_m=round(float(C[2]), 2), marker_height_correction=round(float(scale), 3),
        start=dict(t=round(float(T[0]), 2), x=round(float(Ps[0, 0]), 3), y=round(float(Ps[0, 1]), 3), yaw_deg=round(math.degrees(yaws[0]), 1)),
        end=dict(t=round(float(T[-1]), 2), x=round(float(Ps[-1, 0]), 3), y=round(float(Ps[-1, 1]), 3), yaw_deg=round(math.degrees(yaws[-1]), 1)),
        path_length_m=round(path_len, 3),
        net_displacement_m=round(float(np.linalg.norm(Ps[-1] - Ps[0])), 3),
        net_heading_change_deg=round(math.degrees(yaws[-1] - yaws[0]), 1),
        motion_start_s=round(float(T[mi[0]]), 2) if len(mi) else None,
        motion_stop_s=round(float(T[min(mi[-1] + lag, len(T) - 1)]), 2) if len(mi) else None,
        moving_fraction=round(float(moving.mean()), 3),
        max_speed_mps=round(float(speed.max()), 3),
        max_yaw_rate_dps=round(float(np.degrees(yaw_rate.max())), 1),
        heading_wobble_deg=round(float(np.degrees(np.std((yaw - yaws)[lag:][moving]))), 2) if moving.any() else 0.0,
    )
    with open(os.path.join(out, "track.csv"), "w", newline="") as f:
        wr = csv.writer(f)
        wr.writerow(["t", "x", "y", "yaw_deg"])
        for r in rows:
            wr.writerow([f"{r[0]:.3f}", f"{r[1]:.4f}", f"{r[2]:.4f}", f"{math.degrees(r[3]):.2f}"])
    with open(os.path.join(out, "summary.json"), "w") as f:
        json.dump(summary, f, indent=1)

    fig, ax = plt.subplots(figsize=(8, 6.5))
    for mid, (x, y) in floor_ids.items():
        ax.add_patch(plt.Rectangle((x - 0.125, y - 0.125), 0.25, 0.25, fc="none", ec="k"))
        ax.text(x, y + 0.17, f"id {mid}", ha="center", fontsize=8)
    sc = ax.scatter(P[:, 0], P[:, 1], c=T, cmap="turbo", s=6)
    for j in range(0, len(P), max(1, int(fps / a.step))):
        ax.arrow(Ps[j, 0], Ps[j, 1], 0.12 * math.cos(yaws[j]), 0.12 * math.sin(yaws[j]), head_width=0.04, color="k", lw=0.6)
    ax.plot(P[0, 0], P[0, 1], "go", ms=9, label=f"start {T[0]:.1f}s")
    ax.plot(P[-1, 0], P[-1, 1], "rs", ms=9, label=f"end {T[-1]:.1f}s")
    fig.colorbar(sc, ax=ax, label="time (s)")
    ax.set_aspect("equal")
    ax.grid(True, lw=0.3)
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    ax.set_title(f"path {summary['path_length_m']:.2f} m, net {summary['net_displacement_m']:.2f} m, "
                 f"heading {summary['net_heading_change_deg']:+.0f} deg, moving {summary['motion_start_s']}-{summary['motion_stop_s']} s")
    ax.legend(loc="upper right", fontsize=8)
    fig.tight_layout()
    fig.savefig(os.path.join(out, "track.png"), dpi=110)

    for key, val in summary.items():
        print(f"{key}: {val}")
    print(f"files: {out}/track.png {out}/summary.json {out}/track.csv")


if __name__ == "__main__":
    main()
