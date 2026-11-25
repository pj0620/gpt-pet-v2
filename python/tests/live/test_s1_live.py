"""Live: System 1's questions answered by the real nimble on Ollama, about a real simulator view.

These assert structure (an offered option, a certainty in range), never which option: the model
is free to judge. Its answers are logged for tuning `[s1] min_confidence`.
"""
from __future__ import annotations

import logging
from datetime import timedelta

import pytest
from mcp import ClientSession
from mcp.client.streamable_http import streamablehttp_client

from gpt_pet.mcp import mcp_media_callback
from gpt_pet.s1 import (
    GOAL_QUESTIONS,
    DecisionClient,
    Step,
    action_options,
    executor_questions,
    executor_state,
    goal_state,
    judge_turn,
)
from gpt_pet.settings import Settings

pytestmark = [pytest.mark.live, pytest.mark.timeout(600)]

log = logging.getLogger("gpt_pet.tests")

FIRST_CALL_TIMEOUT_S = 600  # the first simulator call may boot Unity


async def real_view(url: str) -> dict:
    async with streamablehttp_client(url, timeout=timedelta(seconds=FIRST_CALL_TIMEOUT_S)) as (read_stream, write_stream, _):
        async with ClientSession(read_stream, write_stream, read_timeout_seconds=timedelta(seconds=FIRST_CALL_TIMEOUT_S)) as session:
            await session.initialize()
            result = await session.call_tool("get_current_view", {})
    return mcp_media_callback(None, {}, None, result.model_dump(mode="json", by_alias=True, exclude_none=True))["result"]


async def test_nimble_picks_an_offered_action_for_a_real_view(mcp_server: str, nimble: str, sim_settings: Settings) -> None:
    view = await real_view(mcp_server)
    goal = {
        "goal": "go through an open doorway and see the next room",
        "success_criteria": "The pet has passed through a doorway",
        "sub_goals": [],
    }
    steps = [Step("get_current_view", {}, {"result": view, "isError": False})]
    options = action_options(view)
    answers = await DecisionClient(sim_settings.s1).decide(
        executor_state(goal, steps), executor_questions(options, ask_met=True)
    )
    step, met = answers["next"], answers["criteria_met"]
    assert step.choice in options
    assert 0.0 <= step.certainty <= 1.0 and 0.0 <= met.certainty <= 1.0
    assert met.choice in {"true", "false"}
    s1 = sim_settings.s1
    turn = judge_turn(answers, options, steps, sim_settings.tool_limits, s1.min_confidence, s1.model)
    assert turn.kind in {"call", "report", "escalate"}
    log.info("nimble: next=%s (%s, certainty %.2f) met p=%.2f -> %s %s", step.choice,
             options[step.choice]["text"], step.certainty, met.probabilities["true"], turn.kind, turn.tool or turn.reason or "")


async def test_nimble_judges_a_goal_from_the_executors_report(nimble: str, sim_settings: Settings) -> None:
    state = {
        "current_goal": {
            "goal": "get a close look at the sofa",
            "success_criteria": "The pet is within 1 m of the sofa",
            "sub_goals": [],
            "status": "active",
            "attempts": 0,
        },
        "last_report": (
            "Saw: Sofa (0.6 m), CoffeeTable (1.4 m).\nDid: drove to Sofa (succeeded after 6.2 s).\n"
            "Final nav state: succeeded.\nThe pet is 0.6 m from the sofa."
        ),
    }
    answers = await DecisionClient(sim_settings.s1).decide(goal_state(state), GOAL_QUESTIONS)
    status = answers["status"]
    assert status.choice in {"continue", "done", "abandoned"}
    assert 0.0 <= status.certainty <= 1.0
    log.info("nimble goal status: %s (certainty %.2f) %s", status.choice, status.certainty, status.probabilities)
