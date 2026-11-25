"""Record a trace of a pet that is already running, from the outside (`gpt-pet trace attach`).

Subscribes to `gpt-pet serve`'s event stream (`/api/events`), writes its tool calls, results,
goal decisions, reports, ticks and notices into a `gpt_pet.trace` directory, fetches the camera
frame (`/api/frame.jpg`) for every view, and samples the robot's server for ground truth (the MCP
URL from `/api/status`, or `--sim`). Nothing about the running pet changes; its history before the
attach is skipped (the stream replays it first, then sends a status snapshot).
"""
from __future__ import annotations

import asyncio
import json
import logging
import time
from collections.abc import AsyncIterator, Mapping
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

import httpx

from gpt_pet.trace import VIEW_TOOL, TraceRecorder, result_summary

log = logging.getLogger("gpt_pet.trace")


async def sse_frames(response: httpx.Response) -> AsyncIterator[tuple[str, str]]:
    """(event, data) for every server-sent event in a streaming response."""
    event, data = "message", []
    async for line in response.aiter_lines():
        if not line:
            if data:
                yield event, "\n".join(data)
            event, data = "message", []
        elif line.startswith(":"):
            continue  # a keep-alive comment
        elif line.startswith("event:"):
            event = line[6:].strip()
        elif line.startswith("data:"):
            data.append(line[5:].lstrip())


def record_payload(recorder: TraceRecorder, payload: Mapping[str, Any], calls: dict[str, dict[str, Any]], image: bytes | None = None) -> str | None:
    """Write one `adk` payload (as `gpt_pet.runtime.event_payloads` shapes it); returns "view" when
    it was a view (the caller supplies its frame)."""
    tick = payload.get("tick")
    s1 = payload.get("s1") if isinstance(payload.get("s1"), Mapping) else None
    by = str(s1.get("by")) if s1 else ("fallback" if payload.get("fallback") else "model")
    call = payload.get("functionCall")
    response = payload.get("functionResponse")
    if isinstance(call, Mapping):
        calls[str(call.get("id") or "")] = {"by": by, "tool": call.get("name")}
        if call.get("name") != VIEW_TOOL:
            recorder.write("call", {"tick": tick, "author": payload.get("author"), "by": by, "tool": call.get("name"), "args": dict(call.get("args") or {})})
        return None
    if isinstance(response, Mapping):
        called = calls.pop(str(response.get("id") or ""), {})
        body = response.get("response")
        if response.get("name") == VIEW_TOOL and isinstance(body, Mapping) and isinstance(body.get("result"), Mapping):
            recorder.view(called.get("by", by), image, "image/jpeg" if image else None, body["result"], tick)
            return "view"
        recorder.write("result", {"tick": tick, "tool": response.get("name"), **result_summary(str(response.get("name")), body)})
        return None
    if payload.get("text"):
        recorder.write("text", {"tick": tick, "author": payload.get("author"), "by": by, "text": str(payload["text"])[:2000]})
    return None


async def attach(portal: str, out: Path, *, sim_url: str | None = None, seconds: float | None = None) -> Path:
    """Record until `seconds` pass (or forever); returns the trace directory."""
    base = portal.rstrip("/")
    async with httpx.AsyncClient(timeout=httpx.Timeout(10.0, read=None)) as client:
        status = (await client.get(f"{base}/api/status")).json()
        url = sim_url or status.get("mcp_url") or None
        directory = Path(out) / f"attach-{datetime.now(timezone.utc):%Y%m%dT%H%M%SZ}"
        recorder = TraceRecorder(directory, url=url, meta={"run_id": directory.name, "portal": base, "profile": status.get("profile"), "source": "attach"})
        sampler = asyncio.create_task(recorder.run_sampler(), name="trace-attach-sampler")
        calls: dict[str, dict[str, Any]] = {}
        live = False  # the stream replays its history first; record from the first status snapshot on
        started = time.monotonic()
        try:
            async with client.stream("GET", f"{base}/api/events", headers={"Accept": "text/event-stream"}) as response:
                async for event, data in sse_frames(response):
                    if seconds is not None and time.monotonic() - started >= seconds:
                        break
                    if event == "status":
                        live = True
                        continue
                    if not live:
                        continue
                    try:
                        payload = json.loads(data)
                    except json.JSONDecodeError:
                        continue
                    if event == "adk":
                        image = None
                        response_part = payload.get("functionResponse")
                        if isinstance(response_part, Mapping) and response_part.get("name") == VIEW_TOOL:
                            frame = await client.get(f"{base}/api/frame.jpg")
                            image = frame.content if frame.status_code == 200 else None
                        record_payload(recorder, payload, calls, image)
                    elif event == "tick":
                        recorder.write("tick", payload)
                    elif event == "notice":
                        recorder.notice(payload)
        finally:
            sampler.cancel()
            await asyncio.gather(sampler, return_exceptions=True)
            recorder.close()
    return directory
