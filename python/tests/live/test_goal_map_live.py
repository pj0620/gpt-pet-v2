"""Live: Gemini decides goals with the robot's map and the places memory in view (`[brain]
map_for_goals`), and System 1 carries a targeted goal out by rule (`[s1] explore_policy`). Real
ai2thor-mcp, Gemini and nimble. Asserts structure, never wording: Gemini may or may not name a
target."""
from __future__ import annotations

import logging

import pytest

from gpt_pet.brain import build_brain
from gpt_pet.runtime import PetRuntime, function_calls, is_s1_event
from gpt_pet.settings import Settings

pytestmark = [pytest.mark.live, pytest.mark.timeout(900)]

log = logging.getLogger("gpt_pet.tests")


def with_goal_map(settings: Settings) -> Settings:
    brain = settings.brain.model_copy(update={"places_memory": True, "map_for_goals": True, "goal_delay_s": 0, "tick_delay_s": 0})
    s1 = settings.s1.model_copy(update={"enabled": True, "scan_rule": True, "explore_policy": True})
    return settings.model_copy(update={"brain": brain, "s1": s1})


async def test_gemini_sets_goals_with_the_map_and_s1_drives_to_the_target(
    mcp_server: str, gemini_key: str, nimble: str, sim_settings: Settings
) -> None:
    brain = build_brain(with_goal_map(sim_settings))
    async with PetRuntime(brain, user_id="live-goal-map") as pet:
        first = await pet.run_tick()  # no goal yet: Gemini decides, with the map
        log.info("tick 1: %s", first.summary())
        assert pet.stats.maps_shown == 1, "Gemini's first goal decision should carry the map"
        assert brain.places is not None and brain.places.destinations, "the executor's views were remembered"
        second = await pet.run_tick()  # Gemini now also reads what the pet has done
        log.info("tick 2: %s (goal %s)", second.summary(), second.goal)
        assert not first.failed and not second.failed
        maps = pet.stats.maps_shown
        gemini_goal_turns = sum(1 for r in (first, second) for e in r.events if e.author == "goal_setter" and not is_s1_event(e) and e.content)
        assert maps == gemini_goal_turns, (maps, gemini_goal_turns)  # a map exactly when Gemini decided

    for result in (first, second):
        goal = result.goal or {}
        target = goal.get("target_id")
        log.info("tick %d goal %s: kind=%s target=%s", result.number, goal.get("id"), goal.get("kind"), target)
        if not target or goal.get("created_tick") != result.state.get("tick"):
            continue
        assert target in brain.places.destinations, "a target must be a remembered place"
        first_view = next(
            (fr.response["result"] for e in result.events for fr in e.get_function_responses() if fr.name == "get_current_view"),
            {},
        )
        if any(d.get("id") == target and d.get("here") for d in first_view.get("destinations", [])):
            continue  # already there: nothing to drive to
        drives = [fc for fc in function_calls(result.events) if fc.name == "set_nav_goal"]
        assert drives and drives[0].args.get("destination_id") == target, "S1 drives to the target first"
