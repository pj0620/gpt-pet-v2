"""Offline: the pet and people (`[people]`): who was seen, when to react, System 1's people rules,
the person watch (drives and rests), the attention wait, places and the goal setter's prompt."""
from __future__ import annotations

import asyncio
import json
from types import SimpleNamespace

import pytest

from gpt_pet.agents.goal_setter import render_goal_setter_prompt
from gpt_pet.brain import build_brain
from gpt_pet.frames import FrameStore
from gpt_pet.goals import GoalInterrupt
from gpt_pet.navwait import wait_for_drive
from gpt_pet.people import (
    EXECUTOR,
    WATCH,
    People,
    PersonWatch,
    Sighting,
    make_attention_wait,
)
from gpt_pet.places import Places
from gpt_pet.runtime import PetRuntime
from gpt_pet.s1 import LOOK, Step, SystemOne, people_turn
from gpt_pet.settings import NavSettings, PeopleSettings, load_settings
from gpt_pet.stats import RunStats

PERSON = {"marker": 1, "id": "obj:person_0", "kind": "object", "label": "Person", "description": "Person, 1.8 m ahead-left, walking", "distance_m": 1.8, "bbox": [100, 0, 220, 380]}
SOFA = {"marker": 2, "id": "obj:Sofa|1", "kind": "object", "label": "Sofa", "description": "Sofa, 3.0 m ahead", "distance_m": 3.0}
VIEW = {"agent": {"x": 1.0, "z": 2.0, "yaw": 90.0}, "destinations": [PERSON, SOFA]}
EXPLORE = {"id": 3, "goal": "explore the hallway", "success_criteria": "a new room is in view", "status": "active", "kind": "explore"}
SOCIAL = {"id": 4, "goal": "go to the person and see what they are doing", "success_criteria": "the report says what they do", "status": "active", "kind": "social", "target_id": "obj:person_0"}
LIMITS = {"get_current_view": 4, "set_nav_goal": 2, "set_attention": 2, "do_rotate": 2}


class Clock:
    def __init__(self) -> None:
        self.t = 100.0

    def __call__(self) -> float:
        return self.t


def people(clock: Clock | None = None) -> People:
    return People(["Person"], react_every_s=60.0, clock=clock or Clock())


def look(view: dict = VIEW) -> Step:
    return Step(LOOK, {}, {"result": view, "isError": False})


def attended(person_id: str = "obj:person_0", state: str = "framed") -> Step:
    return Step("set_attention", {"destination_id": person_id}, {"result": {"state": state, "target": person_id, "distance_m": 1.4}, "isError": False})


# --- the people memory ---------------------------------------------------------------------------


def test_people_are_recognised_remembered_and_never_places() -> None:
    memory = people()
    assert memory.is_person(PERSON) and not memory.is_person(SOFA)
    [sighting] = memory.saw({**VIEW, "attention": {"state": "framed", "target": "obj:person_0"}}, "executor")
    assert (sighting.id, sighting.distance_m, sighting.source) == ("obj:person_0", 1.8, "executor")
    assert sighting.bearing_deg is not None and sighting.bearing_deg < 0  # left of centre
    assert memory.attending("obj:person_0") and memory.counts["sightings"] == 1
    assert [d["id"] for d in memory.without_people(VIEW)["destinations"]] == ["obj:Sofa|1"]

    places = Places(people_labels=frozenset({"person"}))
    places.saw(VIEW, tick=1)
    assert "obj:person_0" not in places.destinations and "obj:Sofa|1" in places.destinations
    assert places.marks(PERSON) == {}  # never "here", "visited" or "failed": people move


def test_a_goal_is_social_by_its_kind_its_target_or_its_words() -> None:
    memory = people()
    memory.saw(VIEW, "executor")
    assert memory.is_social(SOCIAL)
    assert memory.is_social({**EXPLORE, "kind": None, "target_id": "obj:person_0"})
    assert memory.is_social({"goal": "find the owner and say hi", "status": "active"})
    assert not memory.is_social(EXPLORE) and not memory.is_social(None)
    # live run: GPT-6 Luna's exploration "identified an interesting person, animal, room, or object"
    criteria = "The robot has driven into newly revealed floor and identified an interesting person, animal, room, or object."
    assert not memory.is_social({**EXPLORE, "kind": "explore", "success_criteria": criteria})
    assert not memory.is_social({"goal": "explore your surroundings", "success_criteria": criteria, "status": "active"})


def test_reactions_wait_their_turn_per_person_and_kind() -> None:
    clock = Clock()
    memory = people(clock)
    sightings = memory.saw(VIEW, "watch")
    assert memory.to_react(WATCH, EXPLORE, sightings).id == "obj:person_0"
    memory.react(WATCH, "obj:person_0")
    assert memory.to_react(WATCH, EXPLORE, sightings) is None  # not again soon
    assert memory.to_react(EXECUTOR, EXPLORE, sightings) is not None  # another kind of reaction
    assert memory.to_react(EXECUTOR, SOCIAL, sightings) is None  # busy with people already
    clock.t += 61
    assert memory.to_react(WATCH, EXPLORE, sightings) is not None

    memory.saw(VIEW, "executor")  # seen again just now: news for the goal setter
    assert memory.news_for_setter(EXPLORE).id == "obj:person_0"
    assert memory.news_for_setter(EXPLORE) is None and memory.counts["escalations"] == 1
    clock.t += 30  # too old to be news, whatever the cooldown says
    memory.reacted.clear()
    assert memory.news_for_setter(EXPLORE) is None


def test_the_goal_setter_reads_who_was_seen_and_is_steered_toward_them() -> None:
    clock = Clock()
    memory = people(clock)
    memory.saw({**VIEW, "attention": {"state": "framed", "target": "obj:person_0", "distance_m": 1.4, "tilt_deg": 27.0}}, "executor")
    clock.t += 3
    prompt = render_goal_setter_prompt({"current_goal": EXPLORE}, load_settings("sim"), None, memory)
    assert "People the pet has seen" in prompt
    assert "- Person (obj:person_0): Person, 1.8 m ahead-left, walking (seen 3 s ago)" in prompt
    assert "keeping obj:person_0 in view by itself (attention framed, 1.4 m, camera tilted up)" in prompt
    assert "switch to them now: end the current goal" in " ".join(prompt.split())
    assert '"kind": "social", "target_id": "obj:person_0"' in prompt  # a decision to copy, with the real id
    assert 'attempts have reached 4, or a person is near and the pet switches to them (see "People" above).' in prompt
    quiet = render_goal_setter_prompt({}, load_settings("sim"), None, people())
    assert "People the pet has seen" not in quiet and "attempts have reached 4." in quiet and "$people_switch" not in quiet


# --- System 1's people rules --------------------------------------------------------------------


def test_a_person_coming_into_view_ends_an_exploring_stretch_after_turning_to_them() -> None:
    memory = people()
    memory.saw(VIEW, "executor")
    turn = people_turn(EXPLORE, [look()], memory, LIMITS)
    assert (turn.kind, turn.tool, turn.args) == ("call", "set_attention", {"destination_id": "obj:person_0"})
    memory.attention = {"state": "framed", "target": "obj:person_0"}
    report = people_turn(EXPLORE, [look(), attended()], memory, LIMITS)
    assert report.kind == "report"
    assert "Success criteria not judged: a person came into view (S1 rules)." in report.text
    assert report.text.endswith("Person: Person (obj:person_0): Person, 1.8 m ahead-left, walking; the robot turned to them (attention framed).")
    assert people_turn(EXPLORE, [look(), attended(), look()], memory, LIMITS) is None  # reacted lately: carry on exploring
    assert people_turn(EXPLORE, [look({**VIEW, "destinations": [SOFA]})], people(), LIMITS) is None


def test_on_a_social_goal_s1_attends_looks_and_lets_the_model_describe_the_person() -> None:
    memory = people()
    memory.saw(VIEW, "executor")
    turn = people_turn(SOCIAL, [look()], memory, LIMITS)
    assert (turn.tool, turn.args) == ("set_attention", {"destination_id": "obj:person_0"})
    memory.attention = {"state": "framed", "target": "obj:person_0"}
    assert people_turn(SOCIAL, [look(), attended()], memory, LIMITS).tool == LOOK
    described = people_turn(SOCIAL, [look(), attended(), look()], memory, LIMITS)
    assert (described.kind, described.reason) == ("escalate", "a person to describe")
    refused = Step("set_attention", {"destination_id": "obj:person_0"}, {"error": "budget exhausted: set_attention"})
    assert people_turn(SOCIAL, [look(), refused], memory, LIMITS) is None  # the generic rules take refusals
    nobody = look({**VIEW, "destinations": [SOFA]})
    searching = people_turn(SOCIAL, [nobody], memory, LIMITS)  # out of view: turn toward where they were
    assert (searching.tool, searching.args) == ("set_attention", {"destination_id": "obj:person_0"})
    assert people_turn(SOCIAL, [nobody, attended(state="lost"), nobody], memory, LIMITS) is None  # once a stretch


def test_the_goal_gate_hands_a_person_nearby_to_the_model_once() -> None:
    settings = load_settings("sim")
    stats = RunStats()
    memory = people()
    s1 = SystemOne(settings, stats, None, memory)
    memory.saw(VIEW, "executor")
    state = {"current_goal": {**EXPLORE, "attempts": 0}, "pending_goals": [], "last_report": "Saw: Person (1.8 m)."}
    assert asyncio.run(s1.goal_setter_turn(SimpleNamespace(state=state), None)) is None  # escalated
    assert stats.snapshot()["s1"]["escalations"] == {"goal_setter: person nearby": 1}
    assert "set_attention" in s1.limits  # the attention tools have caps


# --- the person watch ---------------------------------------------------------------------------


def test_the_watch_shows_traces_and_raises_a_person_once() -> None:
    frames, seen = FrameStore(), []
    memory = people()
    watch = PersonWatch(memory, 1.5, frames=frames, on_view=lambda *args: seen.append(args))
    sighting = watch.note(VIEW, b"jpeg-bytes", "image/jpeg", 3, EXPLORE)
    assert sighting.id == "obj:person_0" and frames.get("camera").data == b"jpeg-bytes"
    assert seen[0][0] == "watch" and seen[0][3] == VIEW and seen[0][4] == 3
    assert watch.note(VIEW, None, None, 3, EXPLORE) is None  # already raised
    assert watch.note(VIEW, None, None, 3, SOCIAL) is None and memory.counts["watch_looks"] == 3


def test_a_person_in_view_stops_the_drive_being_waited_out() -> None:
    async def scenario():
        looks, canceled = 0, []

        async def get_status() -> dict:
            return {"state": "following", "goal": {"goal_id": "g1"}}

        async def watch() -> str | None:
            nonlocal looks
            looks += 1
            return "a person came into view: Person (obj:person_0)" if looks == 2 else None

        async def cancel() -> None:
            canceled.append(True)

        nav = NavSettings(poll_interval_s=0.01, max_wait_s=30)
        return await wait_for_drive(get_status, "g1", nav, cancel=cancel, watch=watch, watch_every_s=0.02), looks, canceled

    wait, looks, canceled = asyncio.run(scenario())
    assert wait.outcome == "interrupted" and looks == 2 and canceled == [True]
    assert wait.note == "Stopped: a person came into view: Person (obj:person_0). The robot is no longer driving."


def test_set_attention_returns_once_the_person_is_framed() -> None:
    class Tool:
        def __init__(self, name: str, states: list[str]) -> None:
            self.name, self.states = name, states

        async def run_async(self, args: dict, tool_context: object) -> dict:
            state = self.states.pop(0)
            return {"content": [{"type": "text", "text": json.dumps({"state": state, "target": "obj:person_0"})}], "isError": False}

    status_tool = Tool("get_attention_status", ["aiming", "approaching", "framed"])
    toolset = SimpleNamespace(get_tools=lambda: asyncio.sleep(0, result=[status_tool]))
    memory = people()
    wait = make_attention_wait(toolset, PeopleSettings(attention_wait_s=10), memory, GoalInterrupt())
    started = {"content": [{"type": "text", "text": json.dumps({"state": "aiming", "target": "obj:person_0"})}], "isError": False}
    response = asyncio.run(wait(SimpleNamespace(name="set_attention"), {"destination_id": "obj:person_0"}, None, started))
    assert response["result"]["state"] == "framed" and memory.attention["state"] == "framed" and memory.counts["attended"] == 1
    assert asyncio.run(wait(SimpleNamespace(name="get_current_view"), {}, None, started)) is None


def test_a_person_wakes_the_resting_pet() -> None:
    async def scenario():
        settings = load_settings("sim")
        settings = settings.model_copy(update={"people": settings.people.model_copy(update={"enabled": True, "watch_every_s": 0.02})})
        notices: list[dict] = []
        async with PetRuntime(build_brain(settings), user_id="test", on_notice=notices.append) as pet:
            looks = 0

            async def look_between_ticks(tick, goal):
                nonlocal looks
                looks += 1
                return Sighting("obj:person_0", "Person", 1.8, -12.0, "Person, 1.8 m ahead-left", 0.0, "watch") if looks == 3 else None

            pet.brain.watch.look_between_ticks = look_between_ticks
            started = asyncio.get_running_loop().time()
            await pet._rest(5.0, watch=True)
            return asyncio.get_running_loop().time() - started, looks, notices

    waited, looks, notices = asyncio.run(scenario())
    assert waited < 1.0 and looks == 3
    assert notices[-1]["kind"] == "person" and "A person came into view while resting: Person (obj:person_0)" in notices[-1]["text"]


def test_a_person_the_goal_setter_has_not_heard_about_cuts_the_rest_short_or_skips_it() -> None:
    # suite C, rest bot: the drive watch stopped a drive for the person and System 1 ended the
    # tick; the pet then rested 22 s (the watch's own cooldown) while the person walked up to it
    async def scenario(seen_during_tick: bool):
        settings = load_settings("sim")
        settings = settings.model_copy(update={"people": settings.people.model_copy(update={"enabled": True, "watch_every_s": 0.02})})
        notices: list[dict] = []
        async with PetRuntime(build_brain(settings), user_id="test", on_notice=notices.append) as pet:
            memory = pet.brain.people
            looks = 0

            async def look_between_ticks(tick, goal):  # the watch reacted already: never a sighting
                nonlocal looks
                looks += 1
                if looks == 3:
                    memory.saw(VIEW, "watch")

            pet.brain.watch.look_between_ticks = look_between_ticks
            if seen_during_tick:
                memory.saw(VIEW, "executor")
            started = asyncio.get_running_loop().time()
            await pet._rest(5.0, watch=True)
            waited = asyncio.get_running_loop().time() - started
            heard_after = memory.news_for_setter(None) is not None  # the goal setter still hears about them
            return waited, looks, notices, memory.counts["rest_wakes"], heard_after

    waited, looks, notices, wakes, heard = asyncio.run(scenario(seen_during_tick=True))
    assert waited < 0.5 and looks == 0 and wakes == 1 and heard
    assert notices[-1]["kind"] == "person" and "The pet does not rest" in notices[-1]["text"]
    waited, looks, notices, wakes, heard = asyncio.run(scenario(seen_during_tick=False))
    assert waited < 1.0 and looks == 3 and wakes == 1 and heard
    assert "while resting" in notices[-1]["text"]


@pytest.mark.parametrize("enabled", [False, True])
def test_the_attention_tools_are_shown_only_with_people_on(enabled: bool) -> None:
    settings = load_settings("sim")
    settings = settings.model_copy(update={"people": settings.people.model_copy(update={"enabled": enabled})})
    brain = build_brain(settings)
    shown = brain.toolset.tool_filter
    assert ("set_attention" in shown) is enabled and ("get_attention_status" in shown) is enabled
    assert (brain.people is not None) is enabled and (brain.watch is not None) is enabled
