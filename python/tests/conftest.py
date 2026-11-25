"""Shared fixtures. No mocks anywhere: `live` tests run the real ai2thor-mcp server and Gemini."""
from __future__ import annotations

import json
import os
import urllib.error
import urllib.request
from importlib import resources

import pytest
from dotenv import load_dotenv

from gpt_pet.settings import Settings, load_settings
from gpt_pet.simlaunch import (
    AI2THOR_MCP_DIR_ENV,
    AI2THOR_SCENE_ENV,
    DEFAULT_SCENE,
    ai2thor_mcp_dir,
    launch_mcp_server,
    mcp_alive,
    stop_mcp_server,
)

# Same environment as the CLI and `adk web`: API key and default profile from the package .env.
load_dotenv(str(resources.files("gpt_pet").joinpath(".env")))


def pytest_configure(config: pytest.Config) -> None:
    config.addinivalue_line(
        "markers", "live: drives the real ai2thor-mcp server (boots Unity) and calls Gemini"
    )


TEST_MODEL_ENV = "GPTPET_TEST_MODEL"


@pytest.fixture(scope="session")
def sim_settings() -> Settings:
    """The `sim` profile. `GPTPET_TEST_MODEL` puts another model in its place for the goal setter
    and the executor, e.g. `openai/gpt-5.6-luna` (OpenAI's without reasoning, as the experiments
    run them) for a fraction of Gemini 3.5 Flash's cost."""
    settings = load_settings("sim")
    name = os.environ.get(TEST_MODEL_ENV)
    if not name:
        return settings
    effort = "none" if name.startswith("openai/") else settings.model.reasoning_effort
    return settings.model_copy(update={"model": settings.model.model_copy(update={"goal_setter": name, "executor": name, "reasoning_effort": effort})})


@pytest.fixture(scope="session")
def gemini_key() -> str:
    key = os.environ.get("GOOGLE_API_KEY")
    if not key:
        pytest.fail("GOOGLE_API_KEY is not set (expected in src/gpt_pet/.env); live tests need real Gemini")
    return key


@pytest.fixture(scope="session")
def nimble(sim_settings: Settings) -> str:
    """The real System 1 decision model: Ollama >= 0.35 serving `[s1] model`."""
    url = str(sim_settings.s1.base_url).rstrip("/")
    try:
        with urllib.request.urlopen(f"{url}/api/tags", timeout=5) as response:
            names = [model.get("name", "") for model in json.load(response).get("models", [])]
    except (urllib.error.URLError, TimeoutError, OSError) as exc:
        pytest.fail(f"no Ollama at {url} ({exc}); System 1 needs Ollama >= 0.35 running")
    if not any(name.split(":")[0] == sim_settings.s1.model for name in names):
        pytest.fail(f"Ollama at {url} does not have {sim_settings.s1.model!r}; run `ollama pull {sim_settings.s1.model}`")
    return url


@pytest.fixture(scope="session")
def mcp_server(sim_settings: Settings, tmp_path_factory: pytest.TempPathFactory):
    """The real ai2thor-mcp server URL: attach to a running one, else launch it from the checkout
    on the URL's port. A launched server is stopped with its own process group only, so no other
    simulator on the machine is touched."""
    url = str(sim_settings.mcp.url)
    if mcp_alive(url):
        yield url
        return

    repo = ai2thor_mcp_dir()
    if not (repo / "python" / "ai2thor_mcp" / "main.py").is_file():
        pytest.fail(
            f"no server answers at {url} and the ai2thor-mcp checkout was not found at {repo} "
            f"(set {AI2THOR_MCP_DIR_ENV})"
        )
    log_path = tmp_path_factory.mktemp("ai2thor-mcp") / "server.log"
    try:
        process = launch_mcp_server(url, scene=os.environ.get(AI2THOR_SCENE_ENV, DEFAULT_SCENE), log_path=log_path)
    except (RuntimeError, TimeoutError) as exc:
        pytest.fail(f"could not launch ai2thor-mcp at {url}: {exc}")
    try:
        yield url
    finally:
        stop_mcp_server(process)
