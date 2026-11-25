"""Attach to a running ai2thor-mcp server or launch one from the sibling checkout.

Shared by `gpt-pet serve`, the experiment harness and the live test suite. A launched server
serves on the port of its URL and runs in its own process group, so stopping it also stops its
Unity simulator and never touches any other simulator on the machine.
"""
from __future__ import annotations

import json
import logging
import os
import signal
import subprocess
import time
import urllib.error
import urllib.request
from pathlib import Path
from urllib.parse import urlsplit

log = logging.getLogger("gpt_pet.sim")

AI2THOR_MCP_DIR_ENV = "AI2THOR_MCP_DIR"
AI2THOR_SCENE_ENV = "AI2THOR_SCENE"
DEFAULT_SCENE = "FloorPlan_Train1_3"
START_TIMEOUT_S = 90.0

_HEALTH_BODY = json.dumps(
    {
        "jsonrpc": "2.0",
        "id": 1,
        "method": "initialize",
        "params": {
            "protocolVersion": "2025-03-26",
            "capabilities": {},
            "clientInfo": {"name": "gpt-pet", "version": "0"},
        },
    }
).encode()


def mcp_alive(url: str, timeout: float = 5.0) -> bool:
    """True when an MCP streamable-HTTP endpoint answers `initialize` with HTTP 200."""
    request = urllib.request.Request(
        url,
        data=_HEALTH_BODY,
        method="POST",
        headers={"Content-Type": "application/json", "Accept": "application/json, text/event-stream"},
    )
    try:
        with urllib.request.urlopen(request, timeout=timeout) as response:
            return response.status == 200
    except (urllib.error.URLError, TimeoutError, ConnectionError, OSError):
        return False


def ai2thor_mcp_dir() -> Path:
    configured = os.environ.get(AI2THOR_MCP_DIR_ENV)
    if configured:
        return Path(configured).expanduser().resolve()
    # src/gpt_pet/simlaunch.py -> gpt_pet -> src -> python -> gpt-pet-v2 -> gptpet-ws
    return (Path(__file__).resolve().parents[4] / "ai2thor-mcp").resolve()


def default_scene() -> str:
    return os.environ.get(AI2THOR_SCENE_ENV, DEFAULT_SCENE)


def launch_mcp_server(url: str, *, scene: str | None = None, log_path: Path | None = None) -> subprocess.Popen[bytes]:
    """Start `uv run python/ai2thor_mcp/main.py --http` in the checkout, serving on `url`'s port,
    and wait until it answers."""
    repo = ai2thor_mcp_dir()
    entry = repo / "python" / "ai2thor_mcp" / "main.py"
    if not entry.is_file():
        raise FileNotFoundError(f"ai2thor-mcp checkout not found at {repo} (set {AI2THOR_MCP_DIR_ENV})")
    scene = scene or default_scene()
    log_file = open(log_path, "ab") if log_path else subprocess.DEVNULL  # noqa: SIM115 - handed to Popen
    process = subprocess.Popen(
        ["uv", "run", "python/ai2thor_mcp/main.py", "--http", "--scene", scene],
        cwd=repo,
        stdout=log_file,
        stderr=subprocess.STDOUT,
        env={**os.environ, "FASTMCP_PORT": str(urlsplit(url).port or 8000)},
        start_new_session=True,
    )
    deadline = time.monotonic() + START_TIMEOUT_S
    while time.monotonic() < deadline:
        if process.poll() is not None:
            raise RuntimeError(f"ai2thor-mcp exited with {process.returncode} during startup (log: {log_path})")
        if mcp_alive(url):
            log.info("ai2thor-mcp launched at %s (scene %s, pid %d)", url, scene, process.pid)
            return process
        time.sleep(1.0)
    stop_mcp_server(process)
    raise TimeoutError(f"ai2thor-mcp did not answer at {url} within {START_TIMEOUT_S:.0f}s (log: {log_path})")


def ensure_mcp_server(url: str, *, scene: str | None = None, log_path: Path | None = None) -> subprocess.Popen[bytes] | None:
    """Return None when a server already answers at `url`, else the launched process."""
    if mcp_alive(url):
        log.info("attached to a running MCP server at %s", url)
        return None
    return launch_mcp_server(url, scene=scene, log_path=log_path)


def _group_alive(group: int) -> bool:
    try:
        os.killpg(group, 0)
    except ProcessLookupError:
        return False
    return True


def stop_mcp_server(process: subprocess.Popen[bytes] | None, timeout_s: float = 15.0) -> None:
    """Stop a server this process launched together with its Unity simulator (its process group)."""
    if process is None:
        return
    group = process.pid  # the leader of the session `launch_mcp_server` started
    for sig in (signal.SIGTERM, signal.SIGKILL):
        if not _group_alive(group):
            break
        os.killpg(group, sig)
        deadline = time.monotonic() + timeout_s
        while _group_alive(group) and time.monotonic() < deadline:
            if process.poll() is None:
                try:
                    process.wait(timeout=0.5)
                except subprocess.TimeoutExpired:
                    pass
            else:
                time.sleep(0.2)
    process.poll()
