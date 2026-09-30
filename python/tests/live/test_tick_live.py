"""Live: the whole stack. Real ai2thor-mcp server, real Gemini, two ticks, then the CLI.

These assert structure and state transitions, never wording: the model is free to pick any goal.
"""
from __future__ import annotations

import logging
import os
import subprocess
import sys

import pytest
from google.genai import types

from gpt_pet.agents.executor import is_budget_refusal
from gpt_pet.brain import build_brain
from gpt_pet.runtime import PetRuntime, TickResult, function_calls, function_responses, model_call_count, response_is_error
from gpt_pet.settings import Settings

pytestmark = [pytest.mark.live, pytest.mark.timeout(1200)]

log = logging.getLogger("gpt_pet.tests")

AGENT_ORDER = ["goal_setter", "executor"]


def assert_tool_caps_respected(result: TickResult, limits: dict[str, int]) -> None:
    """Executed (not refused) calls per tool never exceed the profile's [tool_limits]."""
    executed: dict[str, int] = {}
    for fr in function_responses(result.events):
        if not is_budget_refusal(fr.response):
            executed[fr.name] = executed.get(fr.name, 0) + 1
    over = {name: n for name, n in executed.items() if name in limits and n > limits[name]}
    assert not over, f"tool caps exceeded: {over} (limits {limits})"
    assert not result.truncated, "tick hit the LLM call budget"


def has_inline_image(response: types.FunctionResponse) -> bool:
    parts = getattr(response, "parts", None) or []
    return any(
        part.inline_data is not None and (part.inline_data.mime_type or "").startswith("image/")
        for part in parts
    )


async def test_two_ticks_drive_the_goal_memory(mcp_server: str, gemini_key: str, sim_settings: Settings) -> None:
    brain = build_brain(sim_settings)
    async with PetRuntime(brain, user_id="live-test") as pet:
        first = await pet.run_tick()
        log.info("tick 1: %s", first.summary())

        authors = [event.author for event in first.events]
        positions = [authors.index(name) for name in AGENT_ORDER if name in authors]
        assert len(positions) == len(AGENT_ORDER), f"missing agent events; authors were {authors}"
        assert positions == sorted(positions), f"agents ran out of order: {authors}"

        calls = [fc.name for fc in function_calls(first.events)]
        assert calls, "the executor made no tool calls"
        assert calls[0] == "get_current_view", f"first tool call should look around, got {calls}"
        assert set(calls) <= set(sim_settings.mcp.tool_filter), f"unexpected tool names: {calls}"
        assert set(calls) & {"set_nav_goal", "do_rotate", "do_move"}, f"no actuation call in {calls}"

        views = [fr for fr in function_responses(first.events) if fr.name == "get_current_view"]
        assert views, "no get_current_view response recorded"
        assert not any(response_is_error(fr) for fr in views), "get_current_view returned an error"
        assert any(has_inline_image(fr) for fr in views), "the camera frame did not reach the model as an image"

        state = first.state
        assert state["tick"] == 1
        assert state["goal_decision"]["previous_goal_status"] == "none"
        goal = state["current_goal"]
        assert goal["id"] == 1
        assert goal["status"] == "active"
        assert goal["goal"].strip()
        assert goal["success_criteria"].strip()
        assert state["goal_history"] == []
        assert state["last_report"].strip(), "the executor left no report"
        assert model_call_count(first.events) <= sim_settings.brain.max_llm_calls_per_tick
        assert_tool_caps_respected(first, sim_settings.tool_limits)

        second = await pet.run_tick()
        log.info("tick 2: %s", second.summary())
        state = second.state
        assert state["tick"] == 2
        status = state["goal_decision"]["previous_goal_status"]
        assert status in {"continue", "done", "abandoned"}, f"unexpected decision {status!r}"
        if status == "continue":
            assert state["current_goal"]["id"] == 1
            assert state["current_goal"]["attempts"] == 1
            assert state["goal_history"] == []
        else:
            assert state["goal_history"][0]["id"] == 1
            assert state["goal_history"][0]["status"] == status
            assert state["goal_history"][0]["finished_tick"] == 2
            assert state["current_goal"]["id"] == 2
        assert state["last_report"].strip()
        assert model_call_count(second.events) <= sim_settings.brain.max_llm_calls_per_tick
        assert_tool_caps_respected(second, sim_settings.tool_limits)
        log.info("goal memory after two ticks: %s", state["current_goal"])


def test_cli_runs_one_tick(mcp_server: str, gemini_key: str) -> None:
    env = dict(os.environ)
    env.pop("GPTPET_CONFIG_DIR", None)
    completed = subprocess.run(
        [sys.executable, "-m", "gpt_pet.cli", "run", "--profile", "sim", "--ticks", "1"],
        capture_output=True,
        text=True,
        timeout=900,
        env=env,
        check=False,
    )
    output = completed.stdout + completed.stderr
    assert completed.returncode == 0, output[-4000:]
    assert "tick=1" in output, output[-4000:]
    assert "goal_id=1" in output, output[-4000:]
    assert "status=none" in output, output[-4000:]
    assert "truncated=0" in output, output[-4000:]
