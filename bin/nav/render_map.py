#!/usr/bin/env python3
"""Render a map_snapshot.py JSON to PNG: occupancy grid, latest scan (red), robot pose (blue arrow, the
robot's +x), 0.5 m grid. Optional extra JSON files add earlier robot poses as a trail.
Usage: render_map.py SNAP.json OUT.png [--trail a.json b.json ...] [--goal x y]"""
import base64, json, math, sys
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

args = sys.argv[1:]
snap, out = json.load(open(args[0])), args[1]
trail, goal = [], None
if "--trail" in args:
    i = args.index("--trail") + 1
    while i < len(args) and not args[i].startswith("--"):
        trail.append(json.load(open(args[i]))["robot"]); i += 1
if "--nav" in args:
    nav = json.load(open(args[args.index("--nav") + 1]))
    goal = (nav["goal"][0], nav["goal"][1])
    trail = [{"x": e["pose"][0], "y": e["pose"][1]} for e in nav["trail"] if e["pose"]]
if "--goal" in args:
    i = args.index("--goal"); goal = (float(args[i + 1]), float(args[i + 2]))
m = snap["map"]
g = np.frombuffer(base64.b64decode(m["data"]), dtype=np.uint8).reshape(m["h"], m["w"]).astype(int)
img = np.full(g.shape, 0.75)              # unknown
img[g == 0] = 1.0                          # free
occ = (g > 0) & (g <= 100)
img[occ] = 1.0 - g[occ] / 100.0            # occupied -> dark
ext = [m["ox"], m["ox"] + m["w"] * m["res"], m["oy"], m["oy"] + m["h"] * m["res"]]
fig, ax = plt.subplots(figsize=(8, 8), dpi=110)
ax.imshow(img, cmap="gray", vmin=0, vmax=1, origin="lower", extent=ext, interpolation="nearest")
if snap["scan"]:
    p = np.array(snap["scan"]); ax.scatter(p[:, 0], p[:, 1], s=2, c="red", label="latest scan")
if trail:
    t = np.array([[r["x"], r["y"]] for r in trail] + [[snap["robot"]["x"], snap["robot"]["y"]]])
    ax.plot(t[:, 0], t[:, 1], "-o", c="tab:green", ms=3, label="robot trail")
r = snap["robot"]
ax.arrow(r["x"], r["y"], 0.3 * math.cos(r["yaw"]), 0.3 * math.sin(r["yaw"]), width=0.03, color="blue",
         length_includes_head=True, label="robot +x")
ax.plot(snap["laser"]["x"], snap["laser"]["y"], "c^", ms=6, label="laser")
if goal:
    ax.plot(*goal, "m*", ms=16, label="goal")
ax.set_xticks(np.arange(math.floor(ext[0]), ext[1] + 0.5, 0.5), minor=False)
ax.set_yticks(np.arange(math.floor(ext[2]), ext[3] + 0.5, 0.5), minor=False)
ax.grid(alpha=0.35); ax.set_aspect("equal"); ax.legend(loc="upper right", fontsize=7)
ax.set_title(f"robot ({r['x']:.2f}, {r['y']:.2f}) m, yaw {math.degrees(r['yaw']):.0f} deg  [map frame, 0.5 m grid]")
pad = 0.5
xs = [ext[0], ext[1]]; ys = [ext[2], ext[3]]
known = np.argwhere(g != 255)
if len(known):
    (y0, x0), (y1, x1) = known.min(0), known.max(0)
    xs = [m["ox"] + x0 * m["res"] - pad, m["ox"] + (x1 + 1) * m["res"] + pad]
    ys = [m["oy"] + y0 * m["res"] - pad, m["oy"] + (y1 + 1) * m["res"] + pad]
ax.set_xlim(xs); ax.set_ylim(ys)
fig.tight_layout(); fig.savefig(out); print(out)
