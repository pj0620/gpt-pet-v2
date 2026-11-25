"""Live: the pet server watches the robot's attention and tells the portal (no LLM calls).

The simulator needs the person preload (AI2THOR_MCP_PERSON_PRELOAD=1): this module sets it before
the session fixture launches the server; a server launched earlier without it makes the test skip.
"""
from __future__ import annotations

import asyncio
import os
import time

os.environ.setdefault("AI2THOR_MCP_PERSON_PRELOAD", "1")  # before the mcp_server fixture launches the simulator

import pytest
from fastapi.testclient import TestClient

from gpt_pet.experiment import call_tool
from gpt_pet.server import create_app
from gpt_pet.settings import Settings

pytestmark = [pytest.mark.live, pytest.mark.timeout(600)]

ACTIVE = ("aiming", "approaching", "framed", "searching")


def sim(url: str, tool: str, args: dict | None = None):
    result = asyncio.run(call_tool(url, tool, args))
    return getattr(result, "data", None) if hasattr(result, "data") else result


def test_the_portal_learns_what_the_robots_attention_is_doing(mcp_server: str, sim_settings: Settings, gemini_key: str) -> None:
    try:
        spawned = sim(mcp_server, "sim_person_spawn")
    except Exception as exc:  # noqa: BLE001
        pytest.skip(f"no person in this simulator (launched without AI2THOR_MCP_PERSON_PRELOAD=1?): {exc}")
    object_id = spawned.get("object_id") if isinstance(spawned, dict) else None
    assert object_id, spawned
    app = create_app(sim_settings, start_paused=True, refresh_map=False)
    try:
        with TestClient(app) as client:
            assert client.get("/api/status").json()["features"]["attention"] is True
            assert client.get("/api/attention").json()["state"] == "off"

            sim(mcp_server, "set_attention", {"destination_id": f"obj:{object_id}"})
            deadline = time.monotonic() + 8.0
            seen: dict = {}
            while time.monotonic() < deadline:
                seen = client.get("/api/attention").json()
                if seen.get("state") in ACTIVE:
                    break
                time.sleep(0.25)
            assert seen.get("state") in ACTIVE, seen
            assert seen["target"] == f"obj:{object_id}" and seen["label"] == "Person"
            assert {"distance_m", "bearing_deg", "seen_s", "unseen_s", "moving", "tilt_deg", "reason"} <= set(seen)

            # the Events Log gets a line explaining the turn (kept for replay); the status frames are not replayed
            frames = app.state.broadcaster.replay(0)
            notices = [frame for frame in frames if frame.event == "notice" and '"kind": "attention"' in frame.data]
            assert notices and "Attention: " in notices[0].data, [frame.data for frame in frames][-5:]
            assert not [frame for frame in frames if frame.event == "attention"]

            sim(mcp_server, "cancel_attention")
            deadline = time.monotonic() + 8.0
            while time.monotonic() < deadline:
                seen = client.get("/api/attention").json()
                if seen.get("state") in ("off", "lost"):
                    break
                time.sleep(0.25)
            # canceled, or lost by itself first (the person spawns out of sight, 5 s away from "lost")
            assert seen.get("state") in ("off", "lost"), seen
            ended = [frame for frame in app.state.broadcaster.replay(0) if frame.event == "notice" and "Attention ended" in frame.data]
            assert ended, [frame.data for frame in app.state.broadcaster.replay(0)][-5:]
    finally:
        for tool in ("cancel_attention", "sim_person_despawn"):
            try:
                sim(mcp_server, tool)
            except Exception:  # noqa: BLE001
                pass
