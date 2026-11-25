"""Offline: what the goal setter reads, with and without `[brain] map_for_goals`."""
from __future__ import annotations

from google.genai import types

from gpt_pet.agents.goal_setter import MAP_CAPTION, render_goal_setter_prompt, with_map
from gpt_pet.goals import STATE_CURRENT_GOAL
from gpt_pet.places import Places
from gpt_pet.settings import load_settings

PNG = b"\x89PNG\r\n\x1a\nnot-really-a-map"


def test_without_the_places_memory_the_prompt_is_as_before() -> None:
    settings = load_settings("sim")
    settings = settings.model_copy(update={"brain": settings.brain.model_copy(update={"failure_guidance": False})})
    prompt = render_goal_setter_prompt({STATE_CURRENT_GOAL: None}, settings)
    assert "What the pet has done so far" not in prompt and "$places_section" not in prompt
    assert "\nnone\n\nDecide:" in prompt  # the queued goals ("none"), then straight on


def test_with_the_places_memory_the_prompt_lists_what_was_done_and_offers_targets() -> None:
    settings = load_settings("sim")
    places = Places(started=0.0)
    view = {"agent": {"x": 0.0, "z": 0.0, "yaw": 0.0}, "destinations": [{"id": "psg:2", "kind": "passage", "label": "doorway", "description": "passage: doorway ahead, ~1.0 m wide", "distance_m": 3.0}]}
    places.remember("get_current_view", {}, {"result": view, "isError": False}, tick=1)
    prompt = render_goal_setter_prompt({STATE_CURRENT_GOAL: None}, settings, places)
    assert "What the pet has done so far, from its memory of places." in prompt
    assert "target_id: the id of\none place listed below" in prompt
    assert "- psg:2: passage: doorway ahead, ~1.0 m wide (3 m)" in prompt
    assert prompt.index("What the pet has done so far") < prompt.index("Decide:")


def test_the_map_joins_the_last_user_turn_without_touching_the_original() -> None:
    tick = types.Content(role="user", parts=[types.Part(text="tick 4")])
    contents = [tick]
    shown = with_map(contents, PNG, "image/png")
    assert contents == [tick] and [p.text for p in tick.parts] == ["tick 4"]  # the session's event is untouched
    [turn] = shown
    assert turn.role == "user" and [p.text for p in turn.parts[:2]] == ["tick 4", MAP_CAPTION]
    assert turn.parts[2].inline_data.data == PNG and turn.parts[2].inline_data.mime_type == "image/png"
    [alone] = with_map([], PNG, "image/png")
    assert alone.role == "user" and alone.parts[0].text == MAP_CAPTION
