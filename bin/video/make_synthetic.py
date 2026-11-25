#!/usr/bin/env python3
"""Render a synthetic static-camera video of a robot driving on a floor with ArUco markers.

Self-test material for the bin/video tools: the clip looks like what an iPad on a tripod records
(static camera, perspective, H.264 compression, sensor noise) and the ground truth goes to
truth.json so the tools can be checked. The manoeuvre is picked from the seed and is NOT printed,
so a blind "watch it and say what happened" test is possible (use --reveal to print it).

Usage:
  make_synthetic.py OUT_DIR --seed 7 [--camera topdown|angled] [--scenario NAME] [--reveal]
Outputs in OUT_DIR: video.mp4, floor.json (marker layout for track.py), truth.json
"""
import argparse
import json
import math
import os
import random
import subprocess

import cv2
import numpy as np

DICT = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
FLOOR_W, FLOOR_H = 4.0, 3.0            # floor area, metres
PPM = 400                              # floor texture resolution, px per metre
FLOOR_MARKERS = {10: (0.5, 0.5), 11: (3.5, 0.5), 12: (3.5, 2.5), 13: (0.5, 2.5)}  # id -> centre (m)
FLOOR_MARKER_SIZE = 0.25               # black square side, m
ROBOT_ID = 0
ROBOT_MARKER_SIZE = 0.18
ROBOT_MARKER_BORDER = 0.03             # white quiet zone around the black square, m
ROBOT_L, ROBOT_W, ROBOT_H = 0.36, 0.30, 0.20   # body length (x, forward), width (y), height
SCENARIOS = ["forward", "forward_turn_forward", "spin", "stuck", "forward_reverse", "oscillate", "arc"]


def camera(kind, w, h):
    if kind == "topdown":
        fx = 0.7 * w                        # ~71 deg horizontal field of view
        C = np.array([FLOOR_W / 2, FLOOR_H / 2, 3.5])
        R = np.array([[1, 0, 0], [0, -1, 0], [0, 0, -1]], float)   # image up = +y, looking down
    else:
        fx = 0.8 * w                        # ~64 deg, tripod 2.6 m back from the floor edge, 2.6 m high
        C = np.array([FLOOR_W / 2, -2.6, 2.6])
        target = np.array([FLOOR_W / 2, FLOOR_H / 2, 0.0])
        f = target - C
        f /= np.linalg.norm(f)
        r = np.cross(f, np.array([0, 0, 1.0]))
        r /= np.linalg.norm(r)
        d = np.cross(f, r)
        R = np.stack([r, d, f])
    K = np.array([[fx, 0, w / 2], [0, fx, h / 2], [0, 0, 1]], float)
    t = -R @ C
    return K, R, t, C


def project(K, R, t, X):
    Xc = (R @ np.asarray(X, float).T).T + t
    p = (K @ Xc.T).T
    return p[:, :2] / p[:, 2:3]


def marker_tile(mid, size_m, border_m):
    px, b = int(size_m * PPM), int(border_m * PPM)
    tile = np.full((px + 2 * b, px + 2 * b), 255, np.uint8)
    tile[b:b + px, b:b + px] = cv2.aruco.generateImageMarker(DICT, mid, px)
    return tile


def floor_texture(rng):
    W, H = int(FLOOR_W * PPM), int(FLOOR_H * PPM)
    tex = np.full((H, W, 3), 150, np.int16)
    tex += rng.normal(0, 6, (H, W, 1)).astype(np.int16)
    tex = np.clip(tex, 0, 255).astype(np.uint8)
    for x in np.arange(0, FLOOR_W + 1e-6, 0.5):          # tile grout lines
        cv2.line(tex, (int(x * PPM), 0), (int(x * PPM), H - 1), (125, 125, 125), 2)
    for y in np.arange(0, FLOOR_H + 1e-6, 0.5):
        cv2.line(tex, (0, int(y * PPM)), (W - 1, int(y * PPM)), (125, 125, 125), 2)
    cv2.rectangle(tex, (int(0.2 * PPM), int(1.2 * PPM)), (int(0.7 * PPM), int(1.9 * PPM)), (90, 110, 130), -1)  # a rug
    for mid, (mx, my) in FLOOR_MARKERS.items():
        tile = marker_tile(mid, FLOOR_MARKER_SIZE, 0.04)
        cx, cy = int(mx * PPM), int((FLOOR_H - my) * PPM)   # texture row 0 is world y = FLOOR_H
        half = tile.shape[0] // 2
        tex[cy - half:cy - half + tile.shape[0], cx - half:cx - half + tile.shape[1]] = tile[..., None]
    return tex


def render_floor(tex, K, R, t, w, h):
    T_w2t = np.array([[PPM, 0, 0], [0, -PPM, FLOOR_H * PPM], [0, 0, 1]], float)
    H_floor = K @ np.column_stack([R[:, 0], R[:, 1], t])
    return cv2.warpPerspective(tex, H_floor @ np.linalg.inv(T_w2t), (w, h), flags=cv2.INTER_LINEAR)


ROBOT_TILE = marker_tile(ROBOT_ID, ROBOT_MARKER_SIZE, ROBOT_MARKER_BORDER)


def draw_robot(img, K, R, t, C, pose):
    x, y, yaw = pose
    c, s = math.cos(yaw), math.sin(yaw)

    def W(px, py, pz):
        return np.array([x + c * px - s * py, y + s * px + c * py, pz])

    L, Wd, H = ROBOT_L / 2, ROBOT_W / 2, ROBOT_H
    bot = [W(L, Wd, 0), W(L, -Wd, 0), W(-L, -Wd, 0), W(-L, Wd, 0)]
    top = [W(L, Wd, H), W(L, -Wd, H), W(-L, -Wd, H), W(-L, Wd, H)]
    faces = []
    for i in range(4):
        j = (i + 1) % 4
        faces.append((np.array([bot[i], bot[j], top[j], top[i]]), (60, 60, 75)))
    faces.append((np.array(top), (95, 95, 110)))
    faces.sort(key=lambda f: -np.linalg.norm(f[0].mean(axis=0) - C))     # farthest first
    for quad, color in faces:
        cv2.fillConvexPoly(img, project(K, R, t, quad).astype(np.int32), color, lineType=cv2.LINE_AA)
    # marker on the top face: marker-image up = robot forward (+x), marker-image right = robot right (-y)
    m = ROBOT_MARKER_SIZE / 2 + ROBOT_MARKER_BORDER
    quad = np.array([W(m, m, H), W(m, -m, H), W(-m, -m, H), W(-m, m, H)])   # TL, TR, BR, BL of the tile
    dst = project(K, R, t, quad).astype(np.float32)
    n = ROBOT_TILE.shape[0]
    src = np.array([[0, 0], [n - 1, 0], [n - 1, n - 1], [0, n - 1]], np.float32)
    warped = cv2.warpPerspective(ROBOT_TILE, cv2.getPerspectiveTransform(src, dst), (img.shape[1], img.shape[0]))
    mask = np.zeros(img.shape[:2], np.uint8)
    cv2.fillConvexPoly(mask, dst.astype(np.int32), 255)
    img[mask > 0] = np.repeat(warped[mask > 0][:, None], 3, axis=1)


def make_segments(name, rng):
    v = rng.uniform(0.15, 0.30)
    w = rng.uniform(0.4, 0.8)
    if name == "forward":
        d = rng.uniform(0.6, 1.5)
        return [("wait", 1.0), ("move", v, d / v), ("wait", 1.0)], dict(distance_m=d, speed_mps=v)
    if name == "forward_turn_forward":
        d1, d2 = rng.uniform(0.5, 1.0), rng.uniform(0.4, 0.8)
        ang = math.radians(rng.choice([90, -90, 60, -60, 120, -120]))
        return ([("wait", 1.0), ("move", v, d1 / v), ("wait", 0.5), ("turn", math.copysign(w, ang), abs(ang) / w),
                 ("wait", 0.5), ("move", v, d2 / v), ("wait", 1.0)],
                dict(distance1_m=d1, turn_deg=math.degrees(ang), distance2_m=d2, speed_mps=v))
    if name == "spin":
        ang = math.radians(rng.choice([90, 180, 270, 360])) * rng.choice([1, -1])
        return [("wait", 1.0), ("turn", math.copysign(w, ang), abs(ang) / w), ("wait", 1.0)], dict(turn_deg=math.degrees(ang))
    if name == "stuck":
        return [("wait", rng.uniform(6, 9))], {}
    if name == "forward_reverse":
        d = rng.uniform(0.5, 1.0)
        back = d * rng.uniform(0.4, 0.8)
        return ([("wait", 1.0), ("move", v, d / v), ("wait", 0.7), ("move", -v, back / v), ("wait", 1.0)],
                dict(forward_m=d, reverse_m=back, speed_mps=v))
    if name == "oscillate":
        d = rng.uniform(0.8, 1.4)
        amp, freq = math.radians(rng.uniform(12, 25)), rng.uniform(0.6, 1.2)
        return [("wait", 1.0), ("wobble", v, d / v, amp, freq), ("wait", 1.0)], dict(distance_m=d, wobble_amp_deg=math.degrees(amp), wobble_hz=freq, speed_mps=v)
    if name == "arc":
        d = rng.uniform(0.8, 1.5)
        wa = rng.choice([1, -1]) * rng.uniform(0.25, 0.5)
        return [("wait", 1.0), ("arc", v, wa, d / v), ("wait", 1.0)], dict(arc_length_m=d, yaw_rate_dps=math.degrees(wa), speed_mps=v)
    raise ValueError(name)


def simulate(segments, start, fps):
    x, y, yaw = start
    traj = [(0.0, x, y, yaw)]
    dt = 1.0 / fps
    t = 0.0
    for seg in segments:
        dur = seg[1] if seg[0] == "wait" else seg[2] if seg[0] == "wobble" else seg[-1]
        yaw_base, tau = yaw, 0.0
        n = max(1, int(round(dur * fps)))
        for _ in range(n):
            if seg[0] == "move":
                v = seg[1]
                x += v * math.cos(yaw) * dt
                y += v * math.sin(yaw) * dt
            elif seg[0] == "turn":
                yaw += seg[1] * dt
            elif seg[0] == "arc":
                v, wa = seg[1], seg[2]
                yaw += wa * dt
                x += v * math.cos(yaw) * dt
                y += v * math.sin(yaw) * dt
            elif seg[0] == "wobble":
                v, amp, freq = seg[1], seg[3], seg[4]
                tau += dt
                yaw = yaw_base + amp * math.sin(2 * math.pi * freq * tau)
                x += v * math.cos(yaw) * dt
                y += v * math.sin(yaw) * dt
            t += dt
            traj.append((t, x, y, yaw))
        if seg[0] == "wobble":
            yaw = yaw_base
    return traj


def summarize(traj, fps):
    P = np.array([[p[1], p[2]] for p in traj])
    yaw = np.unwrap([p[3] for p in traj])
    step = np.linalg.norm(np.diff(P, axis=0), axis=1)
    speed = step * fps
    yaw_rate = np.abs(np.diff(yaw)) * fps
    moving = (speed > 0.02) | (yaw_rate > math.radians(2))
    idx = np.where(moving)[0]
    return dict(
        path_length_m=float(step.sum()),
        net_displacement_m=float(np.linalg.norm(P[-1] - P[0])),
        net_heading_change_deg=float(math.degrees(yaw[-1] - yaw[0])),
        motion_start_s=float(idx[0] / fps) if len(idx) else None,
        motion_stop_s=float((idx[-1] + 1) / fps) if len(idx) else None,
        max_speed_mps=float(speed.max()),
        start_pose=dict(x=float(P[0, 0]), y=float(P[0, 1]), yaw_deg=float(math.degrees(yaw[0]))),
        end_pose=dict(x=float(P[-1, 0]), y=float(P[-1, 1]), yaw_deg=float(math.degrees(yaw[-1]))),
    )


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("out_dir")
    ap.add_argument("--seed", type=int, required=True)
    ap.add_argument("--camera", choices=["topdown", "angled"], default="topdown")
    ap.add_argument("--scenario", choices=SCENARIOS)
    ap.add_argument("--size", default="1280x720")
    ap.add_argument("--fps", type=float, default=30.0)
    ap.add_argument("--reveal", action="store_true", help="print the ground truth (breaks a blind test)")
    a = ap.parse_args()

    rng = random.Random(a.seed)
    nrng = np.random.default_rng(a.seed)
    w, h = (int(v) for v in a.size.split("x"))
    name = a.scenario or rng.choice(SCENARIOS)
    segments, params = make_segments(name, rng)

    # keep the whole trajectory on the floor and away from the floor markers
    for _ in range(500):
        start = (rng.uniform(1.0, 3.0), rng.uniform(1.0, 2.0), rng.uniform(-math.pi, math.pi))
        traj = simulate(segments, start, a.fps)
        xs, ys = [p[1] for p in traj], [p[2] for p in traj]
        if 0.9 < min(xs) and max(xs) < 3.1 and 0.9 < min(ys) and max(ys) < 2.1:
            break
    else:
        raise SystemExit("could not fit trajectory on the floor")

    K, R, t, C = camera(a.camera, w, h)
    floor_img = render_floor(floor_texture(nrng), K, R, t, w, h)

    os.makedirs(a.out_dir, exist_ok=True)
    tmp = os.path.join(a.out_dir, "_raw.mp4")
    vw = cv2.VideoWriter(tmp, cv2.VideoWriter_fourcc(*"mp4v"), a.fps, (w, h))
    for (_, x, y, yaw) in traj:
        img = floor_img.copy()
        draw_robot(img, K, R, t, C, (x, y, yaw))
        img = cv2.GaussianBlur(img, (3, 3), 0.7)
        img = np.clip(img.astype(np.int16) + nrng.normal(0, 3, img.shape).astype(np.int16), 0, 255).astype(np.uint8)
        vw.write(img)
    vw.release()
    video = os.path.join(a.out_dir, "video.mp4")
    subprocess.run(["ffmpeg", "-y", "-loglevel", "error", "-i", tmp, "-c:v", "libx264", "-preset", "veryfast",
                    "-crf", "23", "-pix_fmt", "yuv420p", video], check=True)
    os.remove(tmp)

    floor = dict(markers={str(k): list(v) for k, v in FLOOR_MARKERS.items()}, marker_size_m=FLOOR_MARKER_SIZE,
                 robot_marker_id=ROBOT_ID, robot_marker_size_m=ROBOT_MARKER_SIZE, robot_marker_height_m=ROBOT_H)
    with open(os.path.join(a.out_dir, "floor.json"), "w") as f:
        json.dump(floor, f, indent=2)
    truth = dict(scenario=name, params=params, camera=a.camera, camera_position_m=C.tolist(),
                 hfov_deg=math.degrees(2 * math.atan((w / 2) / K[0, 0])), fps=a.fps,
                 duration_s=traj[-1][0], **summarize(traj, a.fps),
                 trajectory=[[round(p[0], 3), round(p[1], 4), round(p[2], 4), round(math.degrees(p[3]), 2)] for p in traj])
    with open(os.path.join(a.out_dir, "truth.json"), "w") as f:
        json.dump(truth, f, indent=1)
    print(video, f"({len(traj)} frames, {traj[-1][0]:.1f}s, camera={a.camera})")
    if a.reveal:
        truth.pop("trajectory")
        print(json.dumps(truth, indent=1))


if __name__ == "__main__":
    main()
