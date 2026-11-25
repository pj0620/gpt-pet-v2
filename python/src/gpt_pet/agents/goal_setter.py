"""The goal setter: decides what the pet wants next and closes finished goals.

It has no tools. It reads the goal memory and the executor's last report from session state and
emits a `GoalDecision`. The prompt is rendered by a callable instruction (so ADK's `{key}` state
templating is bypassed and the prompt file may contain braces).

With `[brain] map_for_goals` it also sees the robot's top-down map (`get_map`, fetched only when
Gemini actually decides, after System 1's gate) and, with the places memory, what the pet has
done and the places it has seen but not visited; a new goal may then name one of them as its
`target_id` (`TargetedGoalDecision`).
"""
from __future__ import annotations

import asyncio
import json
import logging
from collections.abc import Callable, Mapping
from string import Template
from typing import TYPE_CHECKING, Any

from google.adk.agents.llm_agent import LlmAgent
from google.adk.agents.readonly_context import ReadonlyContext
from google.genai import types

from gpt_pet.agents.content import content_config
from gpt_pet.agents.models import agent_model
from gpt_pet.goals import (
    STATE_CURRENT_GOAL,
    STATE_GOAL_DECISION,
    STATE_GOAL_HISTORY,
    STATE_LAST_REPORT,
    STATE_PENDING_GOALS,
    GoalDecision,
    TargetedGoalDecision,
)
from gpt_pet.prompts import load_prompt
from gpt_pet.settings import Settings

if TYPE_CHECKING:
    from gpt_pet.mcp_proxy import McpProxy
    from gpt_pet.people import People
    from gpt_pet.places import Places

GOAL_SETTER_NAME = "goal_setter"
MAP_TOOL = "get_map"
MAP_TIMEOUT_S = 10.0
MAP_CAPTION = (
    "The robot's top-down map so far: unexplored grey, free floor white, obstacles black, with "
    "the robot's pose and the destinations it knows marked. Use it to choose where to go next."
)
PLACES_SECTION = """

What the pet has done so far, from its memory of places. Prefer goals that take it somewhere it
has not been. A new goal may set kind (explore, visit, find or social) and target_id: the id of
one place listed below, to go there first.
$summary"""
FAILURE_SECTION = """

A failed drive alone is no reason to abandon a goal: try another route to the same area first
(another passage, or open floor the map shows). Abandon a goal only when it keeps failing."""

PEOPLE_SECTION = """

People the pet has seen (people move: where they were when last seen):
$people
People are the most interesting things for the pet, worth more than finishing an exploration or
going through another doorway. When someone is near and the current goal is not about them, switch
to them now: end the current goal ("done" if its criteria are met, otherwise "abandoned") and make
the new goal social. For example:
{"previous_goal_status": "abandoned", "goal": "go to the person and see what they are doing", "success_criteria": "The report says what the person is doing", "kind": "social", "target_id": "$person_id", "reasoning": "A person is near; people come first."}
The robot keeps them in view and walks up to them by itself. Only when the pet engaged this person
in one of its last two goals and nothing about them changed is going back to exploring fine."""
PEOPLE_SWITCH = ', or a person is near and the pet switches to them (see "People" above)'
PEOPLE_SECTION_ENGAGE = """

People the pet has seen (people move: where they were when last seen), and whether they want to
interact with it:
$people
The pet finds out by itself whether someone wants to interact: it turns to them and reads their
body language for a few seconds. People it found not interested ("leave them be") get no goals
until that time is up; do not go back to them. When someone wants to interact with the pet, that
comes first: end the current goal ("done" if its criteria are met, otherwise "abandoned") and make
a social goal to interact with them (kind social, the person as target_id), guided by what they did:
- they waved, clapped, crouched down or reached out: greet them, for example
{"previous_goal_status": "abandoned", "goal": "greet the person who wants to interact: go close, look up at them and say hello", "success_criteria": "The robot is close to the person, facing them, and the report says how they respond", "kind": "social", "target_id": "$person_id", "reasoning": "They waved at me; someone who wants to interact comes first."}
- they beckoned and walk off: follow them ("follow the person who beckoned me");
- they point at something: go and look at it.
Keep interacting while they keep showing interest (they stay near, face the pet, gesture, crouch);
when they stop or leave, close the goal ("done") and go back to exploring.$lessons"""
PEOPLE_SWITCH_ENGAGE = ', or a person wants to interact with the pet (see "People" above)'

log = logging.getLogger("gpt_pet.goal_setter")


def _dump(value: Any) -> str:
    if value is None or value == "" or value == [] or value == {}:
        return "none"
    if isinstance(value, str):
        return value
    return json.dumps(value, ensure_ascii=False, indent=2)


def render_goal_setter_prompt(
    state: Mapping[str, Any], settings: Settings, places: "Places | None" = None, people: "People | None" = None
) -> str:
    template = Template(load_prompt("goal_setter"))
    section = Template(PLACES_SECTION).substitute(summary="\n".join(places.summary())) if places is not None else ""
    seen_people = people.summary() if people is not None else []
    switch = PEOPLE_SWITCH if seen_people else ""
    if seen_people and people.social is not None:
        keen = [pid for pid in people.last if people.wants_to_interact(pid)]
        nearest = keen or [s.id for s in people.recent(within_s=people.react_every_s * 5)] or list(people.last)
        lessons = people.social.lessons()
        lesson_text = ("\nWhat the pet has learned about people's signals: " + "; ".join(lessons) + ".") if lessons else ""
        section += Template(PEOPLE_SECTION_ENGAGE).substitute(people="\n".join(seen_people), person_id=nearest[0], lessons=lesson_text)
        switch = PEOPLE_SWITCH_ENGAGE if keen else ""
    elif seen_people:
        nearest = people.recent(within_s=people.react_every_s * 5) or list(people.last.values())
        section += Template(PEOPLE_SECTION).substitute(people="\n".join(seen_people), person_id=nearest[0].id)
    return template.safe_substitute(
        current_goal=_dump(state.get(STATE_CURRENT_GOAL)),
        last_report=_dump(state.get(STATE_LAST_REPORT)),
        goal_history=_dump(state.get(STATE_GOAL_HISTORY)),
        pending_goals=_dump(state.get(STATE_PENDING_GOALS)),
        places_section=section + (FAILURE_SECTION if settings.brain.failure_guidance else ""),
        people_switch=switch,
        max_goal_attempts=str(settings.brain.max_goal_attempts),
        default_goal=settings.brain.default_goal,
    )


def with_map(contents: list[types.Content], image: bytes, mime_type: str) -> list[types.Content]:
    """`contents` with the map added to the last user turn. A new Content replaces that turn: the
    original belongs to the session's events and must not grow an image every tick."""
    parts = [types.Part(text=MAP_CAPTION), types.Part.from_bytes(data=image, mime_type=mime_type)]
    if contents and contents[-1].role == "user":
        last = contents[-1]
        return [*contents[:-1], types.Content(role="user", parts=[*(last.parts or []), *parts])]
    return [*contents, types.Content(role="user", parts=parts)]


def make_map_callback(proxy: "McpProxy", on_map: Callable[[], None] | None = None) -> Callable[..., Any]:
    """`before_model_callback` that shows Gemini the map. Placed after System 1's gate, it runs
    only when Gemini decides; a map that cannot be fetched is skipped, never fatal. `on_map` is
    told about every map attached."""

    async def attach_map(callback_context: Any, llm_request: Any) -> None:
        try:
            result = await asyncio.wait_for(proxy.call(MAP_TOOL), timeout=MAP_TIMEOUT_S)
        except Exception as exc:  # noqa: BLE001 - the decision goes ahead without the map
            log.warning("no map for the goal setter: %s", exc)
            return None
        if result.is_error or not result.image:
            log.warning("no map for the goal setter: %s", result.text[:200])
            return None
        llm_request.contents = with_map(list(llm_request.contents or []), result.image, result.mime_type or "image/png")
        if on_map is not None:
            on_map()
        return None

    return attach_map


def build_goal_setter(
    settings: Settings,
    before_model_callback: Callable[..., Any] | None = None,
    on_model_error_callback: Callable[..., Any] | None = None,
    *,
    places: "Places | None" = None,
    map_proxy: "McpProxy | None" = None,
    on_map: Callable[[], None] | None = None,
    people: "People | None" = None,
) -> LlmAgent:
    """`before_model_callback` is System 1's gate (`gpt_pet.s1`), when enabled;
    `on_model_error_callback` stands in for a failed Gemini call (`gpt_pet.agents.fallbacks`).
    `places` adds what the pet has done to the prompt and lets goals name a target;
    `map_proxy` shows Gemini the map (both `[brain] map_for_goals`)."""

    def instruction(ctx: ReadonlyContext) -> str:
        return render_goal_setter_prompt(ctx.state, settings, places, people)

    map_callback = make_map_callback(map_proxy, on_map) if map_proxy is not None else None
    callbacks = [cb for cb in (before_model_callback, map_callback) if cb is not None]
    return LlmAgent(
        name=GOAL_SETTER_NAME,
        model=agent_model(settings.model.goal_setter, settings.model),
        description="Decides what the pet wants to do next and closes finished goals.",
        instruction=instruction,
        output_schema=TargetedGoalDecision if places is not None else GoalDecision,
        output_key=STATE_GOAL_DECISION,
        include_contents="none",
        generate_content_config=content_config(settings.model, settings.model.goal_setter),
        before_model_callback=callbacks if len(callbacks) > 1 else (callbacks[0] if callbacks else None),
        on_model_error_callback=on_model_error_callback,
    )
