"""Draw a trace (`gpt_pet.trace`) so a run can be reviewed at a glance.

    gpt-pet trace render DIR [--last N]

writes into DIR:

    timeline.txt        one line per decision, call, result and view; ground truth when it changes
    storyboard-NN.png   every view the brain received, 12 to a sheet, each captioned with time,
                        tick, who decided, the robot's pose and camera tilt, attention, the people
                        in view and the action that followed
    trajectory.png      top-down: the robot's path (magenta while attending), the person's path,
                        numbered views with the camera's heading, start and end
    index.html          all of the above on one page

Coordinates are AI2-THOR's: x to the right, z down the page (as in the portal's Top View).
"""
from __future__ import annotations

import html
import json
import math
from collections.abc import Iterable, Mapping
from pathlib import Path
from typing import Any

from PIL import Image, ImageDraw, ImageFont

from gpt_pet.trace import META_FILE, TRACE_FILE

TILE_PX = 300
CAPTION_LINES = 4
LINE_PX = 15
COLUMNS = 4
ROWS = 3
MAP_MAX_PX = 900
MAP_MARGIN_M = 0.6
SAMPLE_EVERY_S = 5.0
"""Timeline: a ground-truth line at most this often, plus one whenever a state changes."""

ROBOT = (59, 130, 246)
ATTENDING = (217, 70, 239)
PERSON = (245, 158, 11)
VIEW = (16, 185, 129)
GRID = (226, 232, 240)
INK = (30, 41, 59)
PAPER = (255, 255, 255)
CAPTION_BG = (15, 23, 42)
CAPTION_INK = (241, 245, 249)


def _font(size: int = 12) -> ImageFont.ImageFont:
    try:
        return ImageFont.load_default(size=size)
    except TypeError:  # older Pillow
        return ImageFont.load_default()


def load(directory: str | Path) -> tuple[dict[str, Any], list[dict[str, Any]]]:
    directory = Path(directory)
    meta_path = directory / META_FILE
    meta = json.loads(meta_path.read_text(encoding="utf-8")) if meta_path.is_file() else {}
    records = []
    trace_path = directory / TRACE_FILE
    if trace_path.is_file():
        for line in trace_path.read_text(encoding="utf-8").splitlines():
            if line.strip():
                try:
                    records.append(json.loads(line))
                except json.JSONDecodeError:
                    continue  # a line cut short by a crash
    return meta, records


# --- wording -------------------------------------------------------------------------------


def _side(bearing: Any) -> str:
    if not isinstance(bearing, (int, float)):
        return ""
    if abs(bearing) < 1:
        return "ahead"
    return f"{abs(bearing):.0f}°{'R' if bearing > 0 else 'L'}"


def describe_people(view: Mapping[str, Any]) -> str:
    people = view.get("people") or []
    if not people:
        return "no people"
    return ", ".join(f"Person {p.get('distance_m')} m {_side(p.get('bearing_deg'))}".strip() for p in people)


def describe_pose(view: Mapping[str, Any]) -> str:
    agent = view.get("agent") or {}
    camera = view.get("camera") or {}
    tilt = camera.get("tilt_deg")
    pose = f"({agent.get('x')}, {agent.get('z')}) {agent.get('yaw', 0):.0f}°" if agent else "(pose unknown)"
    return pose + (f" tilt {tilt:.0f}°" if isinstance(tilt, (int, float)) else "")


def describe_call(record: Mapping[str, Any], labels: Mapping[str, str] | None = None) -> str:
    args = record.get("args") or {}
    tool = record.get("tool")
    if tool in ("set_nav_goal", "set_attention") and args.get("destination_id"):
        target = str(args["destination_id"])
        return f"{tool} {labels.get(target, target) if labels else target}"
    if tool == "do_rotate":
        return f"do_rotate {args.get('action', '')} {args.get('degrees', '')}".strip()
    if tool == "do_move":
        return f"do_move {args.get('action', '')} {args.get('moveMagnitude', '')}".strip()
    if tool == "do_look":
        return f"do_look {args.get('direction', '')}"
    return f"{tool} {json.dumps(args)[:60]}" if args else str(tool)


def describe_result(record: Mapping[str, Any]) -> str:
    tool = record.get("tool")
    if record.get("error"):
        return f"{tool}: {str(record['error'])[:90]}"
    drive = record.get("drive")
    if isinstance(drive, Mapping):
        travelled = drive.get("traveled_m")
        parts = [f"{tool}: {drive.get('outcome')}"]
        if travelled is not None:
            parts.append(f"{travelled} m")
        if drive.get("waited_s") is not None:
            parts.append(f"in {drive['waited_s']} s")
        reason = drive.get("reason") or drive.get("note")
        return " ".join(parts) + (f" ({str(reason)[:80]})" if reason else "")
    result = record.get("result")
    if isinstance(result, Mapping) and "state" in result:
        return f"{tool}: {result.get('state')}"
    return f"{tool}: {str(result)[:90]}"


def describe_sample(sample: Mapping[str, Any]) -> str:
    parts = []
    robot = sample.get("robot") or {}
    if robot:
        parts.append(f"robot ({robot.get('x')}, {robot.get('z')}) {robot.get('yaw', 0):.0f}°")
    if sample.get("nav"):
        parts.append(f"nav {sample['nav']}" + (f" -> {sample['nav_goal']}" if sample.get("nav_goal") else ""))
    attention = sample.get("attention") or {}
    if attention.get("state") and attention["state"] != "off":
        parts.append(f"attention {attention['state']} {attention.get('distance_m')} m")
    person = sample.get("person") or {}
    if person:
        parts.append(f"person {person.get('mode')} ({person.get('x')}, {person.get('z')}) {person.get('robot_distance_m')} m from robot")
        activity = person.get("activity") or {}
        if activity:
            wants = " (wants the robot)" if activity.get("wants_robot") else ""
            parts.append(f"doing: {activity.get('kind')}{'/' + activity['style'] if activity.get('style') else ''} {activity.get('phase')}{wants}")
        if person.get("pose") and person["pose"] not in ("idle",) and not str(person["pose"]).startswith("walk_"):
            parts.append(f"pose {person['pose']}")
    return " | ".join(parts)


def _state_key(sample: Mapping[str, Any]) -> tuple[Any, ...]:
    person = sample.get("person") or {}
    return (sample.get("nav"), (sample.get("attention") or {}).get("state"), person.get("mode"), (person.get("activity") or {}).get("phase"))


def timeline(records: Iterable[Mapping[str, Any]], sample_every_s: float = SAMPLE_EVERY_S) -> str:
    """Pure: the trace as readable lines (time, tick, what happened)."""
    lines = ["  t(s)  tick  what"]
    last_sample_t, last_key = -math.inf, None
    for record in records:
        kind = record.get("type")
        t = float(record.get("t") or 0.0)
        tick = record.get("tick")
        prefix = f"{t:6.1f}  {tick if tick is not None else '':>4}  "
        if kind == "view":
            seen = record.get("seen") or []
            attention = (record.get("attention") or {}).get("state")
            extra = f" | attention {attention}" if attention and attention != "off" else ""
            lines.append(
                prefix + f"view #{record.get('seq')} ({record.get('by')}) at {describe_pose(record)} | "
                f"{describe_people(record)} | {len(seen)} destinations{extra}"
            )
        elif kind == "call":
            lines.append(prefix + f"call {describe_call(record)} ({record.get('by')})")
        elif kind == "result":
            lines.append(prefix + "  -> " + describe_result(record))
        elif kind == "text":
            lines.append(prefix + f"{record.get('author')} ({record.get('by')}): {str(record.get('text')).replace(chr(10), ' / ')[:300]}")
        elif kind == "tick":
            lines.append(
                prefix + f"tick {record.get('number')} done: {record.get('seconds', 0):.1f} s, {len(record.get('tools') or [])} tools, "
                f"{record.get('llm_calls', 0)} model calls" + (f", error {record['error']}" if record.get("error") else "")
            )
        elif kind == "notice":
            lines.append(prefix + f"notice {record.get('kind')}: {record.get('text')}")
        elif kind == "sample":
            key = _state_key(record)
            if key != last_key or t - last_sample_t >= sample_every_s:
                lines.append(f"{t:6.1f}        truth: " + describe_sample(record))
                last_sample_t, last_key = t, key
    return "\n".join(lines) + "\n"


# --- storyboard ------------------------------------------------------------------------------


def next_actions(records: list[Mapping[str, Any]]) -> dict[int, str]:
    """Pure: for each view (by seq), the next call made after it (in its tick), as words."""
    out: dict[int, str] = {}
    pending: list[tuple[int, dict[str, str]]] = []
    for record in records:
        if record.get("type") == "view":
            labels = {str(s.get("id")): str(s.get("label")) for s in record.get("seen") or []}
            pending.append((int(record.get("seq") or 0), labels))
        elif record.get("type") == "call" and pending:
            for seq, labels in pending:
                out[seq] = describe_call(record, labels)
            pending = []
        elif record.get("type") == "text" and record.get("author") == "executor" and pending:
            for seq, _ in pending:
                out[seq] = "report"
            pending = []
    return out


def truth_at(records: list[Mapping[str, Any]], t: float) -> str:
    """Pure: what the simulated person was really doing at time t (the latest sample before it)
    and the pet's latest interest decision (notices "social"/"ignore"), as one caption line."""
    activity, decision = None, None
    for record in records:
        if float(record.get("t") or 0.0) > t:
            break
        if record.get("type") == "sample" and (record.get("person") or {}).get("activity"):
            activity = record["person"]["activity"]
        elif record.get("type") == "sample" and record.get("person"):
            activity = None
        elif record.get("type") == "notice" and record.get("kind") in ("social", "ignore"):
            decision = "pet: wants to interact" if record["kind"] == "social" else "pet: not interested"
    parts = []
    if activity:
        style = f"/{activity['style']}" if activity.get("style") else ""
        parts.append(f"truth: {activity.get('kind')}{style} {activity.get('phase') or ''}".strip())
    if decision:
        parts.append(decision)
    return " | ".join(parts)


def storyboard(directory: str | Path, records: list[Mapping[str, Any]], *, last: int | None = None) -> list[Path]:
    """Sheets of the views' frames with captions; returns the files written."""
    directory = Path(directory)
    views = [r for r in records if r.get("type") == "view"]
    if last:
        views = views[-last:]
    actions = next_actions(records)
    font = _font(12)
    per_sheet = COLUMNS * ROWS
    tile_h = TILE_PX + CAPTION_LINES * LINE_PX + 6
    paths: list[Path] = []
    for old in directory.glob("storyboard-*.png"):
        old.unlink()
    for sheet_index in range(0, len(views), per_sheet):
        chunk = views[sheet_index : sheet_index + per_sheet]
        rows = math.ceil(len(chunk) / COLUMNS)
        sheet = Image.new("RGB", (COLUMNS * TILE_PX, rows * tile_h), CAPTION_BG)
        draw = ImageDraw.Draw(sheet)
        for i, view in enumerate(chunk):
            x0, y0 = (i % COLUMNS) * TILE_PX, (i // COLUMNS) * tile_h
            frame_path = directory / str(view.get("frame") or "")
            if view.get("frame") and frame_path.is_file():
                with Image.open(frame_path) as frame:
                    sheet.paste(frame.convert("RGB").resize((TILE_PX, TILE_PX)), (x0, y0))
            else:
                draw.rectangle((x0, y0, x0 + TILE_PX - 1, y0 + TILE_PX - 1), outline=CAPTION_INK)
                draw.text((x0 + 8, y0 + 8), "no frame", fill=CAPTION_INK, font=font)
            attention = (view.get("attention") or {}).get("state")
            captions = [
                f"#{view.get('seq')} t={float(view.get('t') or 0):.0f}s tick {view.get('tick')} {view.get('by')}"
                + (f" | att {attention}" if attention and attention != "off" else ""),
                describe_pose(view),
                describe_people(view) + f" | next: {actions.get(int(view.get('seq') or 0), '-')}",
                truth_at(records, float(view.get("t") or 0.0)),
            ]
            for line, text in enumerate(captions):
                draw.text((x0 + 4, y0 + TILE_PX + 3 + line * LINE_PX), text[:52], fill=CAPTION_INK, font=font)
            if view.get("people"):
                draw.rectangle((x0, y0, x0 + TILE_PX - 1, y0 + TILE_PX - 1), outline=PERSON, width=3)
        path = directory / f"storyboard-{len(paths) + 1:02d}.png"
        sheet.save(path)
        paths.append(path)
    return paths


# --- trajectory ------------------------------------------------------------------------------


def _points(records: list[Mapping[str, Any]]) -> dict[str, list[Any]]:
    robot, person, views = [], [], []
    for record in records:
        if record.get("type") == "sample":
            r = record.get("robot") or {}
            if isinstance(r.get("x"), (int, float)) and isinstance(r.get("z"), (int, float)):
                attending = (record.get("attention") or {}).get("state") in ("aiming", "approaching", "framed", "searching")
                robot.append((float(r["x"]), float(r["z"]), attending))
            p = record.get("person") or {}
            if isinstance(p.get("x"), (int, float)) and isinstance(p.get("z"), (int, float)):
                person.append((float(p["x"]), float(p["z"])))
        elif record.get("type") == "view":
            a = record.get("agent") or {}
            if isinstance(a.get("x"), (int, float)) and isinstance(a.get("z"), (int, float)):
                views.append((float(a["x"]), float(a["z"]), float(a.get("yaw") or 0.0), record.get("seq"), bool(record.get("people"))))
    return {"robot": robot, "person": person, "views": views}


def trajectory(directory: str | Path, records: list[Mapping[str, Any]], title: str = "") -> Path | None:
    """The top-down picture; None when the trace has no positions."""
    points = _points(records)
    xs = [p[0] for p in points["robot"]] + [p[0] for p in points["person"]] + [p[0] for p in points["views"]]
    zs = [p[1] for p in points["robot"]] + [p[1] for p in points["person"]] + [p[1] for p in points["views"]]
    if not xs:
        return None
    min_x, max_x = min(xs) - MAP_MARGIN_M, max(xs) + MAP_MARGIN_M
    min_z, max_z = min(zs) - MAP_MARGIN_M, max(zs) + MAP_MARGIN_M
    scale = max(40.0, min(150.0, MAP_MAX_PX / max(max_x - min_x, max_z - min_z, 1.0)))
    header = 46
    width = int((max_x - min_x) * scale) + 1
    height = int((max_z - min_z) * scale) + 1 + header
    image = Image.new("RGB", (max(width, 420), height), PAPER)
    draw = ImageDraw.Draw(image)
    font = _font(12)

    def px(x: float, z: float) -> tuple[float, float]:
        return (x - min_x) * scale, (z - min_z) * scale + header

    for gx in range(math.floor(min_x), math.ceil(max_x) + 1):
        draw.line([px(gx, min_z), px(gx, max_z)], fill=GRID)
        draw.text((px(gx, min_z)[0] + 2, header + 2), f"x={gx}", fill=(148, 163, 184), font=font)
    for gz in range(math.floor(min_z), math.ceil(max_z) + 1):
        draw.line([px(min_x, gz), px(max_x, gz)], fill=GRID)
        draw.text((2, px(min_x, gz)[1] + 2), f"z={gz}", fill=(148, 163, 184), font=font)

    person = points["person"]
    if len(person) > 1:
        draw.line([px(x, z) for x, z in person], fill=PERSON, width=2)
    for x, z in person[:: max(1, len(person) // 60)]:
        cx, cy = px(x, z)
        draw.ellipse((cx - 2, cy - 2, cx + 2, cy + 2), fill=PERSON)
    robot = points["robot"]
    for (x1, z1, a1), (x2, z2, a2) in zip(robot, robot[1:]):
        draw.line([px(x1, z1), px(x2, z2)], fill=ATTENDING if (a1 and a2) else ROBOT, width=3)
    views = points["views"]
    every = max(1, len(views) // 60)
    for x, z, yaw, seq, with_person in views[::every]:
        cx, cy = px(x, z)
        hx, hy = cx + 18 * math.sin(math.radians(yaw)), cy + 18 * math.cos(math.radians(yaw))
        draw.line([(cx, cy), (hx, hy)], fill=PERSON if with_person else VIEW, width=2)
        draw.ellipse((cx - 4, cy - 4, cx + 4, cy + 4), outline=VIEW, width=2)
        draw.text((cx + 5, cy - 14), str(seq), fill=INK, font=font)
    if robot:
        sx, sy = px(robot[0][0], robot[0][1])
        draw.rectangle((sx - 6, sy - 6, sx + 6, sy + 6), fill=(34, 197, 94))
        ex, ey = px(robot[-1][0], robot[-1][1])
        draw.polygon([(ex, ey - 8), (ex - 7, ey + 6), (ex + 7, ey + 6)], fill=(239, 68, 68))
    if person:
        px_, pz_ = px(person[-1][0], person[-1][1])
        draw.ellipse((px_ - 6, pz_ - 6, px_ + 6, pz_ + 6), outline=PERSON, width=3)

    draw.rectangle((0, 0, image.width, header - 4), fill=CAPTION_BG)
    draw.text((6, 4), title[:90] or "trajectory", fill=CAPTION_INK, font=font)
    legend = [(ROBOT, "robot"), (ATTENDING, "attending"), (PERSON, "person / view with a person"), (VIEW, "view (heading)")]
    x = 6
    for colour, label in legend:
        draw.rectangle((x, 24, x + 12, 34), fill=colour)
        draw.text((x + 16, 22), label, fill=CAPTION_INK, font=font)
        x += 22 + int(draw.textlength(label, font=font)) + 10
    path = Path(directory) / "trajectory.png"
    image.save(path)
    return path


# --- all together ----------------------------------------------------------------------------


def write_html(directory: Path, meta: Mapping[str, Any], records: list[Mapping[str, Any]], sheets: list[Path], map_path: Path | None, text: str) -> Path:
    views = sum(1 for r in records if r.get("type") == "view")
    with_people = sum(1 for r in records if r.get("type") == "view" and r.get("people"))
    ticks = sum(1 for r in records if r.get("type") == "tick")
    duration = max((float(r.get("t") or 0) for r in records), default=0.0)
    parts = [
        "<!doctype html><meta charset='utf-8'><title>GPTPet trace</title>",
        "<style>body{font:14px system-ui;margin:16px;background:#0f172a;color:#e2e8f0}img{max-width:100%;display:block;margin:8px 0}"
        "pre{white-space:pre-wrap;background:#020617;padding:12px;font-size:12px}h2{margin-top:24px}</style>",
        f"<h1>GPTPet trace: {html.escape(str(meta.get('run_id') or directory.name))}</h1>",
        f"<p>{views} views ({with_people} with a person), {ticks} ticks, {duration:.0f} s. "
        f"{html.escape(json.dumps({k: v for k, v in meta.items() if k != 'run_id'}, default=str))}</p>",
    ]
    if map_path is not None:
        parts.append(f"<h2>Where it went</h2><img src='{map_path.name}' alt='trajectory'>")
    if sheets:
        parts.append("<h2>What it saw</h2>" + "".join(f"<img src='{s.name}' alt='{s.stem}'>" for s in sheets))
    parts.append(f"<h2>Timeline</h2><pre>{html.escape(text)}</pre>")
    path = directory / "index.html"
    path.write_text("\n".join(parts), encoding="utf-8")
    return path


def render(directory: str | Path, *, last: int | None = None) -> dict[str, Any]:
    """Draw everything for one trace directory; returns the files written."""
    directory = Path(directory)
    meta, records = load(directory)
    text = timeline(records)
    (directory / "timeline.txt").write_text(text, encoding="utf-8")
    sheets = storyboard(directory, records, last=last)
    title = f"{meta.get('run_id') or directory.name}: {sum(1 for r in records if r.get('type') == 'view')} views"
    map_path = trajectory(directory, records, title=title)
    page = write_html(directory, meta, records, sheets, map_path, text)
    return {"timeline": directory / "timeline.txt", "storyboards": sheets, "trajectory": map_path, "html": page}
