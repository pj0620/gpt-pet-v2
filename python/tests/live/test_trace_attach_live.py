"""Live: `gpt-pet trace attach` records a pet that is already running, from the outside (a real
`gpt-pet serve` on a free port, the real simulator, a real model), and the recording draws."""
from __future__ import annotations

import socket
import threading
import time
from pathlib import Path

import pytest
import uvicorn

from gpt_pet.server import create_app
from gpt_pet.settings import Settings
from gpt_pet.trace_attach import attach
from gpt_pet.trace_report import load, render

pytestmark = [pytest.mark.live, pytest.mark.timeout(600)]


def free_port() -> int:
    with socket.socket() as sock:
        sock.bind(("127.0.0.1", 0))
        return sock.getsockname()[1]


async def test_attach_records_frames_actions_and_ground_truth_of_a_running_pet(mcp_server: str, sim_settings: Settings, tmp_path: Path) -> None:
    settings = sim_settings.model_copy(update={"brain": sim_settings.brain.model_copy(update={"goal_delay_s": 0, "max_goals_per_run": 0})})
    port = free_port()
    server = uvicorn.Server(uvicorn.Config(create_app(settings, refresh_map=False), host="127.0.0.1", port=port, log_level="warning"))
    thread = threading.Thread(target=server.run, daemon=True)
    thread.start()
    try:
        deadline = time.monotonic() + 60
        while not server.started and time.monotonic() < deadline:
            time.sleep(0.2)
        assert server.started, "the pet server did not start"
        directory = await attach(f"http://127.0.0.1:{port}", tmp_path, seconds=45)
    finally:
        server.should_exit = True
        thread.join(30)

    meta, records = load(directory)
    assert meta["source"] == "attach" and meta["profile"] == "sim"
    views = [r for r in records if r["type"] == "view"]
    samples = [r for r in records if r["type"] == "sample"]
    assert views, [r["type"] for r in records][:20]
    assert all(r["frame"] and (directory / r["frame"]).stat().st_size > 1000 for r in views)
    assert all(isinstance(r["agent"].get("x"), float) for r in views)
    assert len(samples) >= 20 and all("robot" in s for s in samples)
    assert any(r["type"] == "text" and r["author"] == "goal_setter" for r in records)
    out = render(directory)
    assert out["storyboards"] and out["trajectory"] is not None
