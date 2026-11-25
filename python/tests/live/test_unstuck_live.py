"""Live: what the brain does about drives that keep failing, in real ticks against ai2thor-mcp
and a real model (GPT-5.6 Luna; `GPTPET_UNSTUCK_TEST_MODEL` overrides). System 1 runs its rules
only (`[s1] rules_only`), so these tests need no nimble. The places memory is told the robot is
stuck, or that a drive failed from where it stands, the way two failed drives would leave it."""
from __future__ import annotations

import logging
import os

import pytest

from gpt_pet.brain import build_brain
from gpt_pet.runtime import PetRuntime, is_s1_event
from gpt_pet.s1 import BACK_UP_M, MOVE, STUCK_DRIVES
from gpt_pet.settings import Settings

pytestmark = [pytest.mark.live, pytest.mark.timeout(900)]

log = logging.getLogger("gpt_pet.tests")
MODEL = os.environ.get("GPTPET_UNSTUCK_TEST_MODEL", "openai/gpt-5.6-luna")


@pytest.fixture(scope="module")
def openai_key() -> str:
    key = os.environ.get("OPENAI_API_KEY")
    if not key:
        pytest.fail("OPENAI_API_KEY is not set (python/src/gpt_pet/.env); these tests drive GPT-5.6 Luna")
    return key


def guarded(settings: Settings) -> Settings:
    model = settings.model.model_copy(update={"goal_setter": MODEL, "executor": MODEL, "reasoning_effort": "none"})
    brain = settings.brain.model_copy(
        update={"places_memory": True, "map_for_goals": True, "failed_guard": True, "goal_delay_s": 0, "tick_delay_s": 0}
    )
    s1 = settings.s1.model_copy(update={"enabled": True, "rules_only": True, "scan_rule": True, "explore_policy": True, "unstuck_rule": True})
    return settings.model_copy(update={"model": model, "brain": brain, "s1": s1})


async def test_when_stuck_system_1_backs_up_before_anything_else(mcp_server: str, openai_key: str, sim_settings: Settings) -> None:
    brain = build_brain(guarded(sim_settings))
    async with PetRuntime(brain, user_id="live-unstuck") as pet:
        first = await pet.run_tick()
        places = brain.places
        assert places is not None and places.pose is not None
        places.stuck_drives, places.stuck_at = STUCK_DRIVES, (places.pose["x"], places.pose["z"])
        second = await pet.run_tick()
    log.info("tick 1: %s", first.summary())
    log.info("tick 2: %s", second.summary())
    assert not second.failed, second.error
    executor_calls = [
        (call.name, dict(call.args or {}), is_s1_event(event))
        for event in second.events
        if event.author == "executor"
        for call in event.get_function_calls()
    ]
    assert executor_calls[0][0] == "get_current_view"  # look first, as always
    name, args, by_s1 = executor_calls[1]
    assert (name, args, by_s1) == (MOVE, {"action": "MoveBack", "moveMagnitude": BACK_UP_M}, True), executor_calls
    assert executor_calls[2][0] == "get_current_view"  # and look again after moving


async def test_a_drive_that_failed_from_here_is_marked_and_refused(mcp_server: str, openai_key: str, sim_settings: Settings) -> None:
    brain = build_brain(guarded(sim_settings))
    async with PetRuntime(brain, user_id="live-failed-guard") as pet:
        first = await pet.run_tick()
        places = brain.places
        assert places is not None and places.pose is not None and places.destinations
        here = (places.pose["x"], places.pose["z"])
        for place in places.destinations.values():  # every drive from here has failed before
            place.failed, place.failed_from = 1, [here]
        second = await pet.run_tick()
    log.info("tick 1: %s", first.summary())
    log.info("tick 2: %s", second.summary())
    assert not second.failed, second.error
    views = [
        response.response["result"]
        for event in second.events
        for response in event.get_function_responses()
        if response.name == "get_current_view" and isinstance(response.response, dict) and isinstance(response.response.get("result"), dict)
    ]
    first_view = views[0]
    remembered = [d for d in first_view["destinations"] if d["id"] in places.destinations and places.destinations[d["id"]].failed_from == [here]]
    assert all(d.get("failed_here") for d in remembered), first_view["destinations"]
    for event in second.events:
        for response in event.get_function_responses():
            if response.name != "set_nav_goal":
                continue
            body = response.response if isinstance(response.response, dict) else {}
            destination = str((body.get("result") or {}).get("destination_id") or "")
            if destination in [d["id"] for d in remembered]:
                assert "failed from here" in str(body.get("error", "")), body  # refused, not driven
