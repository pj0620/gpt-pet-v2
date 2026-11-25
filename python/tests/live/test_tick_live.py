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
from gpt_pet.navwait import DRIVE_OUTCOMES, TERMINAL_STATES
from gpt_pet.runtime import (
    PetRuntime,
    TickResult,
    function_calls,
    function_responses,
    is_s1_event,
    model_call_count,
    response_is_error,
    s1_turn_count,
)
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


def assert_drives_waited_out(result: TickResult) -> None:
    """Every drive the executor started came back to it finished, so it never had to poll."""
    for fr in function_responses(result.events):
        if fr.name != "set_nav_goal" or response_is_error(fr) or is_budget_refusal(fr.response):
            continue
        drive = (fr.response or {}).get("drive")
        assert drive, f"set_nav_goal came back before its drive ended: {fr.response}"
        assert drive["outcome"] in DRIVE_OUTCOMES, drive
        if drive["outcome"] in TERMINAL_STATES:
            assert drive["final_status"]["state"] == drive["outcome"], drive
        log.info("drive %s after %.1f s", drive["outcome"], drive["waited_s"])


def assert_s1_took_the_first_look(result: TickResult, settings: Settings) -> None:
    """With [s1] enabled, System 1 takes each tick's first executor turn by rule: look first."""
    if not settings.s1.enabled:
        return
    calls = [event for event in result.events if event.author == "executor" and event.get_function_calls()]
    assert calls, "the executor made no tool calls"
    first = calls[0]
    assert is_s1_event(first), "the first look should be System 1's rule, not a Gemini call"
    assert first.custom_metadata["s1"] == {"by": "rules", "action": "get_current_view"}
    log.info("tick %d: %d S1 turns, %d Gemini calls", result.number, s1_turn_count(result.events), model_call_count(result.events))


def assert_stats_match_the_ticks(snapshot: dict, results: list[TickResult]) -> None:
    """The live stats agree with what the ticks' own events show."""
    events = [event for result in results for event in result.events]
    assert snapshot["ticks"]["count"] == len(results)
    assert snapshot["llm"]["calls"] == model_call_count(events)
    assert snapshot["s1"]["turns"] == s1_turn_count(events)
    assert set(snapshot["llm"]["by_agent"]) <= {"goal_setter", "executor"} and snapshot["llm"]["by_agent"]
    assert snapshot["tokens"]["input"] > 0 and snapshot["tokens"]["total"] >= snapshot["tokens"]["input"]
    assert snapshot["tools"]["calls"] == len(function_calls(events))
    drives = [
        fr for fr in function_responses(events)
        if fr.name == "set_nav_goal" and isinstance(fr.response, dict) and "drive" in fr.response
    ]
    assert snapshot["drives"]["count"] == len(drives)
    assert snapshot["goals"]["started"] >= 1
    log.info("stats after %d ticks: %s", len(results), snapshot)


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
        assert_drives_waited_out(first)
        assert_s1_took_the_first_look(first, sim_settings)

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
        assert_drives_waited_out(second)
        assert_s1_took_the_first_look(second, sim_settings)
        assert_stats_match_the_ticks(pet.stats.snapshot(), [first, second])
        log.info("goal memory after two ticks: %s", state["current_goal"])


async def test_gemini_takes_over_a_tick_after_s1s_own_calls(
    mcp_server: str, gemini_key: str, nimble: str, sim_settings: Settings
) -> None:
    """S1 looks first (a rule), then an impossible certainty bar hands every judgment to Gemini,
    which must accept a history that starts with S1's call. Gemini 3 rejected unsigned calls."""
    strict = sim_settings.model_copy(update={"s1": sim_settings.s1.model_copy(update={"enabled": True, "min_confidence": 1.0})})
    async with PetRuntime(build_brain(strict), user_id="live-handoff") as pet:
        result = await pet.run_tick()
        snapshot = pet.stats.snapshot()
    log.info("hand-off tick: %s", result.summary())
    executor_turns = [e for e in result.events if e.author == "executor" and e.content is not None and e.content.role == "model"]
    assert executor_turns and is_s1_event(executor_turns[0]), "S1 should take the first look"
    assert any(not is_s1_event(event) for event in executor_turns[1:]), "Gemini should take over after the look"
    assert not result.truncated
    assert result.state["last_report"].strip()
    assert any(key.startswith("executor: ") for key in snapshot["s1"]["escalations"]), snapshot["s1"]


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
