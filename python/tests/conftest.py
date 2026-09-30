"""Shared fixtures. No mocks anywhere: `live` tests run the real ai2thor-mcp server and Gemini."""
from __future__ import annotations

import json
import os
import subprocess
import time
import urllib.error
import urllib.request
from importlib import resources
from pathlib import Path

import pytest
from dotenv import load_dotenv

from gpt_pet.settings import Settings, load_settings

# Same environment as the CLI and `adk web`: API key and default profile from the package .env.
load_dotenv(str(resources.files("gpt_pet").joinpath(".env")))

AI2THOR_MCP_DIR_ENV = "AI2THOR_MCP_DIR"
AI2THOR_SCENE_ENV = "AI2THOR_SCENE"
DEFAULT_SCENE = "FloorPlan_Train1_3"
SERVER_START_TIMEOUT_S = 90

HEALTH_BODY = json.dumps(
    {
        "jsonrpc": "2.0",
        "id": 1,
        "method": "initialize",
        "params": {
            "protocolVersion": "2025-03-26",
            "capabilities": {},
            "clientInfo": {"name": "gpt-pet-tests", "version": "0"},
        },
    }
).encode()


def pytest_configure(config: pytest.Config) -> None:
    config.addinivalue_line(
        "markers", "live: drives the real ai2thor-mcp server (boots Unity) and calls Gemini"
    )


def mcp_alive(url: str, timeout: float = 5.0) -> bool:
    """True when an MCP streamable-HTTP endpoint answers `initialize` with HTTP 200."""
    request = urllib.request.Request(
        url,
        data=HEALTH_BODY,
        method="POST",
        headers={
            "Content-Type": "application/json",
            "Accept": "application/json, text/event-stream",
        },
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
    return (Path(__file__).resolve().parents[3] / "ai2thor-mcp").resolve()  # tests -> python -> gpt-pet-v2 -> gptpet-ws


@pytest.fixture(scope="session")
def sim_settings() -> Settings:
    return load_settings("sim")


@pytest.fixture(scope="session")
def gemini_key() -> str:
    key = os.environ.get("GOOGLE_API_KEY")
    if not key:
        pytest.fail("GOOGLE_API_KEY is not set (expected in src/gpt_pet/.env); live tests need real Gemini")
    return key


@pytest.fixture(scope="session")
def mcp_server(sim_settings: Settings, tmp_path_factory: pytest.TempPathFactory):
    """The real ai2thor-mcp server URL: attach to a running one, else launch it from the checkout."""
    url = str(sim_settings.mcp.url)
    if mcp_alive(url):
        yield url
        return

    repo = ai2thor_mcp_dir()
    entry = repo / "python" / "ai2thor_mcp" / "main.py"
    if not entry.is_file():
        pytest.fail(
            f"no server answers at {url} and the ai2thor-mcp checkout was not found at {repo} "
            f"(set {AI2THOR_MCP_DIR_ENV})"
        )
    scene = os.environ.get(AI2THOR_SCENE_ENV, DEFAULT_SCENE)
    log_path = tmp_path_factory.mktemp("ai2thor-mcp") / "server.log"
    log_file = log_path.open("w", encoding="utf-8")
    process = subprocess.Popen(
        ["uv", "run", "python/ai2thor_mcp/main.py", "--http", "--scene", scene],
        cwd=repo,
        stdout=log_file,
        stderr=subprocess.STDOUT,
    )
    deadline = time.monotonic() + SERVER_START_TIMEOUT_S
    try:
        while time.monotonic() < deadline:
            if process.poll() is not None:
                pytest.fail(f"ai2thor-mcp exited with {process.returncode} during startup; log: {log_path}")
            if mcp_alive(url):
                break
            time.sleep(1.0)
        else:
            pytest.fail(f"ai2thor-mcp did not answer at {url} within {SERVER_START_TIMEOUT_S}s; log: {log_path}")
        yield url
    finally:
        if process.poll() is None:
            process.terminate()
            try:
                process.wait(timeout=15)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait(timeout=15)
        # The Unity child can outlive its parent; only clean it up when this fixture launched it.
        subprocess.run(["pkill", "-f", "thor-OSXIntel64"], check=False)
        log_file.close()
