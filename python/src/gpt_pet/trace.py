"""A flight recorder for the pet: what it saw, what it did, and where everything was.

`TraceRecorder` writes one directory per run, for reviewing the run afterwards, by a person or by
an agent reading the files (`gpt-pet trace render DIR` draws them; see `gpt_pet.trace_report`):

    meta.json     what was recorded (run id, profile, server URL, when)
    trace.jsonl   one JSON line per record; `t` is seconds since the trace started
    frames/       every camera frame the brain received, as the model saw it (with its markers)

Records (`type`):
- `view`: a frame the brain received: `frame` (file), `by` (rules, nimble, model or watch),
  the robot's pose (`agent`: x, z, yaw), `camera` (height, tilt), `attention` (state, distance,
  bearing), `people` (Person destinations: id, distance, bearing estimated from the box) and
  `seen` (every destination: id, label, kind, distance)
- `call` / `result`: every other tool call and how it ended (a drive: outcome, metres, seconds)
- `text`: what the goal setter decided and what the executor reported
- `tick` / `notice`: the runtime's tick summaries and notices
- `sample`: ground truth every `sample_every_s` from the robot's server: the robot's pose and nav
  state (`get_nav_status`), attention (`get_attention_status`) and, in the simulator, the person
  (`sim_person_state`). Tools the server lacks are skipped after their first refusal.

Hooks: `event` (PetRuntime.on_event), `tick` (on_tick), `notice` (on_notice), `view` (frames the
brain takes outside the tool flow, such as the person watch's), `run_sampler` (run as a task).
"""
from __future__ import annotations

import asyncio
import base64
import json
import logging
import math
import time
from collections.abc import Mapping
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

from google.adk.events.event import Event

from gpt_pet.agents.model_api import tool_result_images

log = logging.getLogger("gpt_pet.trace")

VIEW_TOOL = "get_current_view"
TRACE_FILE = "trace.jsonl"
META_FILE = "meta.json"
FRAMES_DIR = "frames"
SAMPLE_EVERY_S = 1.0
SAMPLE_TOOLS = ("get_nav_status", "get_attention_status", "sim_person_state")
PERSON_LABELS = ("person",)
FRAME_WIDTH_PX = 500
HALF_FOV_DEG = 30.0
"""The robot camera's horizontal half field of view (ai2thor-mcp: 60 degrees over 500 px), used
to turn a destination's box into a bearing. A rough estimate for the real Kinect (57 degrees)."""
TEXT_LIMIT = 2000


def _round(value: Any, digits: int = 2) -> Any:
    return round(float(value), digits) if isinstance(value, (int, float)) and not isinstance(value, bool) else value


def bearing_from_box(bbox: Any, width_px: int = FRAME_WIDTH_PX, half_fov_deg: float = HALF_FOV_DEG) -> float | None:
    """Pure: how far right (+) or left (-) of the view's centre a box is, in degrees."""
    if not isinstance(bbox, (list, tuple)) or len(bbox) != 4:
        return None
    centre = (float(bbox[0]) + float(bbox[2])) / 2
    focal = (width_px / 2) / math.tan(math.radians(half_fov_deg))
    return round(math.degrees(math.atan((centre - width_px / 2) / focal)), 1)


def is_person(destination: Mapping[str, Any]) -> bool:
    return str(destination.get("label") or "").strip().lower() in PERSON_LABELS


def view_summary(payload: Mapping[str, Any]) -> dict[str, Any]:
    """Pure: the parts of a get_current_view result worth keeping beside its frame."""
    destinations = [d for d in payload.get("destinations") or [] if isinstance(d, Mapping)]
    attention = payload.get("attention") if isinstance(payload.get("attention"), Mapping) else None
    out: dict[str, Any] = {
        "agent": {k: _round(v) for k, v in (payload.get("agent") or {}).items()},
        "camera": dict(payload.get("camera") or {}),
        "people": [
            {
                "id": d.get("id"),
                "distance_m": _round(d.get("distance_m")),
                "bearing_deg": bearing_from_box(d.get("bbox")),
                "description": d.get("description"),
            }
            for d in destinations
            if is_person(d)
        ],
        "seen": [
            {"id": d.get("id"), "label": d.get("label"), "kind": d.get("kind"), "distance_m": _round(d.get("distance_m"))}
            for d in destinations
        ],
    }
    if attention is not None:
        out["attention"] = {
            key: attention.get(key) for key in ("state", "target", "distance_m", "bearing_deg", "tilt_deg", "seen_s", "reason")
        }
    return out


def result_summary(tool: str, response: Any) -> dict[str, Any]:
    """Pure: how a (non-view) tool call ended, compactly. Refusals and errors keep their text."""
    if not isinstance(response, Mapping):
        return {"ok": True, "result": str(response)[:300]}
    if "error" in response and "result" not in response:
        return {"ok": False, "error": str(response["error"])[:300]}
    result = response.get("result")
    out: dict[str, Any] = {"ok": not bool(response.get("isError"))}
    if isinstance(result, Mapping):
        compact = {k: v for k, v in result.items() if k not in ("destinations", "path")}
        out["result"] = json.loads(json.dumps(compact, default=str))
        if isinstance(result.get("path"), Mapping):
            out["planned_m"] = result["path"].get("length_m")
    elif result is not None:
        out["result"] = str(result)[:300]
    drive = response.get("drive")
    if isinstance(drive, Mapping):
        status = drive.get("final_status") or {}
        progress = status.get("progress") or {}
        out["drive"] = {
            "outcome": drive.get("outcome"),
            "waited_s": drive.get("waited_s"),
            "traveled_m": progress.get("distance_traveled_m"),
            "agent": status.get("agent"),
            "reason": status.get("reason"),
            "note": drive.get("note"),
        }
    return out


def _by(event: Event) -> str:
    """Who took a turn: System 1 ("rules" or its model), a fallback, or the model."""
    metadata = event.custom_metadata or {}
    if isinstance(metadata.get("s1"), Mapping):
        return str(metadata["s1"].get("by") or "s1")
    if metadata.get("fallback"):
        return "fallback"
    return "model"


class TraceRecorder:
    """Writes a run's trace (see the module docstring). Cheap enough to leave on: one JSON line per
    event and one small JPEG per view."""

    def __init__(
        self,
        directory: str | Path,
        *,
        url: str | None = None,
        sample_every_s: float = SAMPLE_EVERY_S,
        meta: Mapping[str, Any] | None = None,
    ) -> None:
        self.directory = Path(directory)
        (self.directory / FRAMES_DIR).mkdir(parents=True, exist_ok=True)
        self.url = url
        self.sample_every_s = sample_every_s
        self._handle = (self.directory / TRACE_FILE).open("a", encoding="utf-8")
        self._clock = time.monotonic()
        self._views = len(list((self.directory / FRAMES_DIR).glob("*")))
        self._calls: dict[str, dict[str, Any]] = {}
        self.unavailable: set[str] = set()
        info = {"started_at": datetime.now(timezone.utc).isoformat(timespec="seconds"), "url": url, **(meta or {})}
        (self.directory / META_FILE).write_text(json.dumps(info, indent=2, default=str), encoding="utf-8")

    # --- writing ---------------------------------------------------------------------------

    def start(self) -> None:
        """Count `t` from now (the experiment harness starts the trace with its run clock)."""
        self._clock = time.monotonic()

    def write(self, kind: str, record: Mapping[str, Any]) -> None:
        if self._handle.closed:
            return
        line = {"t": round(time.monotonic() - self._clock, 2), "type": kind, **record}
        self._handle.write(json.dumps(line, default=str) + "\n")
        self._handle.flush()

    def close(self) -> None:
        self._handle.close()

    def view(self, by: str, image: bytes | None, mime_type: str | None, payload: Mapping[str, Any], tick: int | None = None) -> str | None:
        """Record one frame the brain received and what it showed; returns the frame's file."""
        self._views += 1
        frame = None
        if image:
            suffix = ".png" if "png" in str(mime_type or "") else ".jpg"
            frame = f"{FRAMES_DIR}/{self._views:05d}-t{tick if tick is not None else 0}{suffix}"
            (self.directory / frame).write_bytes(image)
        self.write("view", {"tick": tick, "seq": self._views, "by": by, "frame": frame, **view_summary(payload)})
        return frame

    # --- hooks -----------------------------------------------------------------------------

    def event(self, event: Event, tick: int) -> None:
        """PetRuntime.on_event: tool calls, their results (views with their frames) and texts."""
        if event.partial or event.content is None:
            return
        by = _by(event)
        calls = event.get_function_calls()
        responses = event.get_function_responses()
        for call in calls:
            self._calls[call.id or ""] = {"by": by, "tool": call.name}
            if call.name != VIEW_TOOL:
                self.write("call", {"tick": tick, "author": event.author, "by": by, "tool": call.name, "args": dict(call.args or {})})
        for response in responses:
            called = self._calls.pop(response.id or "", {})
            if response.name == VIEW_TOOL:
                payload = response.response.get("result") if isinstance(response.response, Mapping) else None
                if isinstance(payload, Mapping):
                    images = tool_result_images(response)
                    image = base64.b64decode(images[0][1]) if images else None
                    self.view(called.get("by", by), image, images[0][0] if images else None, payload, tick)
                    continue
            self.write("result", {"tick": tick, "tool": response.name, **result_summary(str(response.name), response.response)})
        if not calls and not responses:
            text = "".join(part.text for part in (event.content.parts or []) if part.text and not part.thought).strip()
            if text:
                self.write("text", {"tick": tick, "author": event.author, "by": by, "text": text[:TEXT_LIMIT]})

    def tick(self, result: Any) -> None:
        """PetRuntime.on_tick."""
        self.write("tick", result.summary_dict())

    def notice(self, notice: Mapping[str, Any]) -> None:
        """PetRuntime.on_notice."""
        self.write("notice", dict(notice))

    # --- ground truth ----------------------------------------------------------------------

    async def sample_once(self, proxy: Any) -> dict[str, Any] | None:
        """One `sample` record from the robot's server; None when nothing answered."""
        sample: dict[str, Any] = {}
        for tool in SAMPLE_TOOLS:
            if tool in self.unavailable:
                continue
            try:
                result = await proxy.call(tool, {})
            except Exception as exc:  # noqa: BLE001 - a missed sample must never disturb the run
                log.debug("trace sample %s failed: %s", tool, exc)
                continue
            data = result.data if isinstance(getattr(result, "data", None), Mapping) else None
            if result.is_error or data is None:
                if "unknown tool" in str(getattr(result, "text", "")).lower() or "not found" in str(getattr(result, "text", "")).lower():
                    self.unavailable.add(tool)
                continue
            if tool == "get_nav_status":
                sample["robot"] = {k: _round(v) for k, v in (data.get("agent") or {}).items()}
                sample["nav"] = data.get("state")
                goal = data.get("goal") or {}
                if goal:
                    sample["nav_goal"] = goal.get("label") or goal.get("destination_id")
            elif tool == "get_attention_status":
                sample["attention"] = {k: data.get(k) for k in ("state", "distance_m", "bearing_deg", "moving")}
                sample["tilt_deg"] = data.get("tilt_deg")
            elif tool == "sim_person_state":
                if data.get("mode") not in (None, "off"):
                    sample["person"] = {k: _round(data.get(k)) for k in ("mode", "x", "z", "yaw", "speed_mps")}
                    robot = data.get("robot") or {}
                    if robot.get("distance_m") is not None:
                        sample["person"]["robot_distance_m"] = _round(robot["distance_m"])
                    # what the simulated person is doing (ground truth the pet never sees)
                    if data.get("pose"):
                        sample["person"]["pose"] = data["pose"]
                    activity = data.get("activity")
                    if isinstance(activity, Mapping):
                        sample["person"]["activity"] = {
                            k: activity.get(k) for k in ("kind", "style", "phase", "wants_robot", "responded", "object_id", "started_s")
                        }
                if "robot" not in sample and isinstance(data.get("robot"), Mapping):
                    sample["robot"] = {k: _round(v) for k, v in data["robot"].items() if k in ("x", "z", "yaw")}
        if not sample:
            return None
        self.write("sample", sample)
        return sample

    async def run_sampler(self, proxy: Any = None) -> None:
        """Sample ground truth every `sample_every_s` until cancelled. Opens its own MCP session
        on `url` unless given one (never the brain's: samples must not queue behind its calls)."""
        own = proxy is None
        if own:
            if not self.url:
                return
            from gpt_pet.mcp_proxy import McpProxy

            proxy = McpProxy(self.url, timeout_s=10)
        try:
            while True:
                started = time.monotonic()
                await self.sample_once(proxy)
                await asyncio.sleep(max(0.0, self.sample_every_s - (time.monotonic() - started)))
        finally:
            if own:
                await proxy.close()
