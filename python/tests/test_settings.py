"""Offline: the ini loader and its schema, on the real shipped files."""
from __future__ import annotations

import shutil
from pathlib import Path

import pytest
from pydantic import ValidationError

from gpt_pet.settings import (
    CONFIG_DIR_ENV,
    PROFILE_ENV,
    StdioMcpSettings,
    StreamableHttpMcpSettings,
    default_config_dir,
    load_settings,
)

EXPECTED_TOOLS = [
    "get_current_view",
    "set_nav_goal",
    "get_nav_status",
    "cancel_nav",
    "list_destinations",
    "get_map",
    "do_move",
    "do_rotate",
]


def test_sim_profile_loads() -> None:
    settings = load_settings("sim")
    assert settings.profile.name == "sim"
    assert isinstance(settings.mcp, StreamableHttpMcpSettings)
    assert str(settings.mcp.url) == "http://localhost:8000/mcp"
    assert settings.mcp.tool_filter == EXPECTED_TOOLS
    assert settings.mcp.read_timeout_s == 300
    assert settings.model.goal_setter.startswith("gemini-")
    assert settings.brain.max_goal_attempts == 3
    assert settings.brain.max_llm_calls_per_tick == 40
    assert settings.brain.goal_delay_s == 60
    assert settings.brain.tick_delay_s == 0
    assert settings.brain.max_goals_per_run == 10
    assert settings.tool_limits["get_nav_status"] == 6
    assert settings.tool_limits["do_rotate"] == 2
    assert set(settings.tool_limits) <= set(EXPECTED_TOOLS)
    assert settings.features.free_camera is True


def test_real_profile_defines_the_same_tool_contract() -> None:
    settings = load_settings("real")
    assert settings.profile.name == "real"
    assert isinstance(settings.mcp, StreamableHttpMcpSettings)
    assert settings.mcp.url.host == "pj-ubuntu.local"
    assert settings.mcp.tool_filter == load_settings("sim").mcp.tool_filter
    assert settings.features.free_camera is False  # no free camera on the real robot


def test_stdio_transport_parses_command_line(tmp_path: Path) -> None:
    (tmp_path / "sim.ini").write_text(
        "[profile]\nname = sim\n"
        "[mcp]\ntransport = stdio\ncommand = uv\n"
        "args = run python/ai2thor_mcp/main.py --scene FloorPlan_Train1_3\n"
        "cwd = /tmp/ai2thor-mcp\ntimeout_s = 120\ntool_filter = get_current_view, do_move\n",
        encoding="utf-8",
    )
    settings = load_settings("sim", config_dir=tmp_path)
    assert isinstance(settings.mcp, StdioMcpSettings)
    assert settings.mcp.args == ["run", "python/ai2thor_mcp/main.py", "--scene", "FloorPlan_Train1_3"]
    assert settings.mcp.cwd == Path("/tmp/ai2thor-mcp")
    assert settings.mcp.timeout_s == 120
    assert settings.mcp.tool_filter == ["get_current_view", "do_move"]
    # Sections with defaults may be omitted entirely.
    assert settings.model.executor.startswith("gemini-")
    assert settings.brain.goal_history_limit == 10


def test_unknown_key_is_rejected(tmp_path: Path) -> None:
    (tmp_path / "sim.ini").write_text(
        "[profile]\nname = sim\n[mcp]\ntransport = streamable_http\nurl = http://localhost:8000/mcp\nbogus = 1\n",
        encoding="utf-8",
    )
    with pytest.raises(ValidationError, match="bogus"):
        load_settings("sim", config_dir=tmp_path)


def test_unknown_transport_is_rejected(tmp_path: Path) -> None:
    (tmp_path / "sim.ini").write_text(
        "[profile]\nname = sim\n[mcp]\ntransport = carrier_pigeon\nurl = http://localhost:8000/mcp\n",
        encoding="utf-8",
    )
    with pytest.raises(ValidationError, match="transport"):
        load_settings("sim", config_dir=tmp_path)


def test_profile_and_config_dir_come_from_the_environment(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    shutil.copy(default_config_dir() / "real.ini", tmp_path / "real.ini")
    text = (tmp_path / "real.ini").read_text(encoding="utf-8")
    (tmp_path / "real.ini").write_text(text.replace("pj-ubuntu.local", "robot.example"), encoding="utf-8")
    monkeypatch.setenv(PROFILE_ENV, "real")
    monkeypatch.setenv(CONFIG_DIR_ENV, str(tmp_path))
    settings = load_settings()
    assert settings.profile.name == "real"
    assert settings.mcp.url.host == "robot.example"


def test_missing_profile_file_names_the_path(tmp_path: Path) -> None:
    with pytest.raises(FileNotFoundError, match=str(tmp_path / "real.ini")):
        load_settings("real", config_dir=tmp_path)


def test_tool_limits_keep_case_and_must_be_positive(tmp_path: Path) -> None:
    base = "[profile]\nname = sim\n[mcp]\ntransport = streamable_http\nurl = http://localhost:8000/mcp\n"
    (tmp_path / "sim.ini").write_text(base + "[tool_limits]\nGetView = 2\n", encoding="utf-8")
    assert load_settings("sim", config_dir=tmp_path).tool_limits == {"GetView": 2}
    (tmp_path / "sim.ini").write_text(base + "[tool_limits]\nget_nav_status = 0\n", encoding="utf-8")
    with pytest.raises(ValidationError, match="tool_limits"):
        load_settings("sim", config_dir=tmp_path)
