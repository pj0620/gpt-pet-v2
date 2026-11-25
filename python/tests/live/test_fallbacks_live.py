"""Live: model calls that really fail. A 1 ms request timeout makes every call to the real model
(Gemini, or `GPTPET_TEST_MODEL`'s) time out; the pet must survive that (fallbacks, failed ticks,
the loop's backoff) and carry on once the timeout is lifted. System 1 is off so that every model
turn is the model's."""
from __future__ import annotations

import logging
from typing import Any

import pytest
from google.adk.agents.llm_agent import LlmAgent

from gpt_pet.agents.content import content_config
from gpt_pet.brain import Brain, build_brain
from gpt_pet.runtime import PetRuntime, TickResult, model_call_count
from gpt_pet.settings import Settings

pytestmark = [pytest.mark.live, pytest.mark.timeout(900)]

log = logging.getLogger("gpt_pet.tests")

INSTANT_S = 0.001
"""A request timeout no real Gemini call can meet."""


def gemini_only(settings: Settings) -> Settings:
    """S1 off, so every model turn is Gemini's, and no pacing rests between ticks."""
    return settings.model_copy(
        update={
            "s1": settings.s1.model_copy(update={"enabled": False}),
            "brain": settings.brain.model_copy(update={"goal_delay_s": 0, "tick_delay_s": 0}),
        }
    )


def set_request_timeout(brain: Brain, seconds: float) -> None:
    """Every agent's request timeout: Gemini's through its request config, another provider's
    adapter (it took `[model] request_timeout_s` when it was built) directly."""
    model = brain.settings.model.model_copy(update={"request_timeout_s": seconds})
    for node in brain.workflow.graph.nodes:
        if isinstance(node, LlmAgent):
            node.generate_content_config = content_config(model)
            if hasattr(node.model, "timeout_s"):
                node.model.timeout_s = seconds


async def test_ticks_that_time_out_fail_and_the_loop_carries_on(
    mcp_server: str, gemini_key: str, sim_settings: Settings
) -> None:
    """No goal yet, so no fallback may stand in: each tick fails, the loop backs off and goes on,
    and the first tick after the timeout is lifted starts a goal as usual."""
    settings = gemini_only(sim_settings)
    brain = build_brain(settings)
    set_request_timeout(brain, INSTANT_S)
    ticks: list[TickResult] = []
    notices: list[dict[str, Any]] = []
    async with PetRuntime(brain, user_id="live-timeouts", on_tick=ticks.append, on_notice=notices.append) as pet:
        assert await pet.run_loop(max_ticks=2) == 2
        for result in ticks:
            log.info("timed-out tick: %s", result.summary())
            assert result.failed and "Timeout" in (result.error or ""), result.error
            assert result.goal is None, "a fallback must never write a goal"
            assert model_call_count(result.events) == 0
        failures = [notice for notice in notices if notice["kind"] == "failure"]
        assert [(n["tick"], n["seconds"]) for n in failures] == [(1, 2.0)]  # the loop stops after tick 2
        snapshot = pet.stats.snapshot()
        assert snapshot["ticks"]["failed"] == 2
        assert (snapshot["llm"]["calls"], snapshot["llm"]["errors"], snapshot["llm"]["fallbacks"]) == (0, 2, 0)
        assert snapshot["llm"]["failed_latency"]["max_ms"] < 5_000

        set_request_timeout(brain, settings.model.request_timeout_s)
        recovered = await pet.run_tick()
        log.info("timeout lifted: %s", recovered.summary())
        assert not recovered.failed
        assert recovered.goal is not None and recovered.goal["id"] == 1 and recovered.goal["status"] == "active"
        assert recovered.state["last_report"].strip()
        assert pet.stats.snapshot()["ticks"]["failed"] == 2


async def test_fallbacks_answer_for_gemini_mid_goal_without_writing_a_goal(
    mcp_server: str, gemini_key: str, sim_settings: Settings
) -> None:
    """With a goal active, both agents' fallbacks stand in: the goal continues, the executor
    reports that it stopped early, and the tick does not fail."""
    brain = build_brain(gemini_only(sim_settings))
    async with PetRuntime(brain, user_id="live-fallbacks") as pet:
        first = await pet.run_tick()
        log.info("tick 1: %s", first.summary())
        assert not first.failed and first.goal is not None and first.goal["status"] == "active"

        set_request_timeout(brain, INSTANT_S)
        second = await pet.run_tick()
        log.info("tick 2 (every call times out): %s", second.summary())
        assert not second.failed, "the fallbacks should carry a tick with an active goal"
        assert second.fallbacks == ["goal_setter", "executor"]
        assert model_call_count(second.events) == 0
        state = second.state
        assert state["goal_decision"]["previous_goal_status"] == "continue"
        assert state["current_goal"]["id"] == first.goal["id"]
        assert state["current_goal"]["attempts"] == first.goal["attempts"] + 1
        report = state["last_report"]
        executor = sim_settings.model.executor  # the report names Gemini, or another model by its id
        who = "Gemini" if executor.startswith("gemini") else executor.split("/", 1)[-1]
        assert report.startswith(f"{who} did not answer (") and "Did: nothing yet." in report, report
        snapshot = pet.stats.snapshot()
        assert (snapshot["llm"]["errors"], snapshot["llm"]["fallbacks"], snapshot["ticks"]["failed"]) == (2, 2, 0)
