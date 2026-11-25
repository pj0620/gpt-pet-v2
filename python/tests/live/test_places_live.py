"""Live: the places memory in real ticks (real ai2thor-mcp, Gemini and nimble). With
`[brain] places_memory` on, every view the executor reads carries the memory's marks and the
memory holds what the robot saw; System 1 never drives to a destination its view marked `here`."""
from __future__ import annotations

import logging
from typing import Any

import pytest

from gpt_pet.brain import build_brain
from gpt_pet.places import PASSAGE_HERE_M, Places
from gpt_pet.runtime import PetRuntime, TickResult, function_responses, is_s1_event
from gpt_pet.settings import Settings

pytestmark = [pytest.mark.live, pytest.mark.timeout(900)]

log = logging.getLogger("gpt_pet.tests")


def remembering(settings: Settings, *, s1: bool) -> Settings:
    """The places memory and the scan rule only: no map or targets, so every S1 drive is picked
    from the view it saw (tests/live/test_goal_map_live.py covers targets)."""
    brain = settings.brain.model_copy(update={"places_memory": True, "map_for_goals": False, "goal_delay_s": 0, "tick_delay_s": 0})
    return settings.model_copy(
        update={
            "brain": brain,
            "s1": settings.s1.model_copy(update={"enabled": s1, "scan_rule": s1, "explore_policy": False}),
        }
    )


def views(result: TickResult) -> list[dict[str, Any]]:
    return [
        fr.response["result"]
        for fr in function_responses(result.events)
        if fr.name == "get_current_view" and isinstance(fr.response, dict) and isinstance(fr.response.get("result"), dict)
    ]


def assert_marks_match_distances(result: TickResult, places: Places) -> None:
    """Every destination within reach by its distance carries `here`."""
    for view in views(result):
        for destination in view["destinations"]:
            radius = PASSAGE_HERE_M if destination.get("kind") == "passage" else places.here_m
            if destination.get("distance_m") is not None and destination["distance_m"] <= radius:
                assert destination.get("here") is True, destination
        if any(d.get("here") or d.get("visited") or d.get("failed") for d in view["destinations"]):
            assert view.get("marks"), "a marked view explains its marks"


def s1_drives_with_their_views(result: TickResult) -> list[tuple[str, dict[str, Any]]]:
    """(destination id, the view S1 chose it from) for every drive S1 decided."""
    latest: dict[str, Any] = {}
    found = []
    for event in result.events:
        for response in event.get_function_responses():
            if response.name == "get_current_view" and isinstance(response.response, dict):
                latest = response.response.get("result") or latest
        for call in event.get_function_calls():
            if call.name == "set_nav_goal" and is_s1_event(event):
                found.append((str((call.args or {}).get("destination_id")), latest))
    return found


async def test_system_1_drives_only_to_what_is_not_here_yet(
    mcp_server: str, gemini_key: str, nimble: str, sim_settings: Settings
) -> None:
    brain = build_brain(remembering(sim_settings, s1=True))
    async with PetRuntime(brain, user_id="live-places") as pet:
        results = [await pet.run_tick(), await pet.run_tick()]
    places = brain.places
    assert places is not None and places.pose is not None and places.destinations
    for result in results:
        log.info("tick %d: %s", result.number, result.summary())
        assert views(result), "the executor looked"
        assert_marks_match_distances(result, places)
        for destination_id, view in s1_drives_with_their_views(result):
            [chosen] = [d for d in view["destinations"] if d["id"] == destination_id]
            assert not chosen.get("here") and int(chosen.get("failed") or 0) < 2, chosen
    seen = [p for p in places.destinations.values() if p.seen]
    log.info("remembered %d places from %d views; reached %s", len(seen), sum(len(views(r)) for r in results),
             [p.label for p in places.destinations.values() if p.reached_tick])


async def test_gemini_reads_the_same_marked_views(mcp_server: str, gemini_key: str, sim_settings: Settings) -> None:
    brain = build_brain(remembering(sim_settings, s1=False))
    async with PetRuntime(brain, user_id="live-places-gemini") as pet:
        result = await pet.run_tick()
    log.info("tick 1: %s", result.summary())
    assert views(result) and brain.places is not None and brain.places.destinations
    assert_marks_match_distances(result, brain.places)
    for response in function_responses(result.events):
        drive = (response.response or {}).get("drive") if response.name == "set_nav_goal" and isinstance(response.response, dict) else None
        planned = ((response.response or {}).get("result") or {}).get("path", {}).get("length_m") if drive else None
        if drive and drive.get("outcome") == "succeeded" and planned is not None and planned < 0.3:
            assert drive.get("note", "").startswith("Already there"), drive
