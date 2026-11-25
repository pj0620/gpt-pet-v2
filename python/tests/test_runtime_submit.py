"""Offline: submitting an owner goal wakes a loop that paused itself on the goal limit.

Uses the real brain and an in-memory session; the MCP toolset never connects because no tick runs.
"""
from __future__ import annotations

import asyncio

from google.adk.events.event import Event
from google.adk.events.event_actions import EventActions

from gpt_pet.brain import build_brain
from gpt_pet.goals import CANCELED_STATUS, STATE_CURRENT_GOAL, STATE_GOAL_HISTORY, STATE_PENDING_GOALS, goal_record, rest_record
from gpt_pet.runtime import PetRuntime, cancel_notice
from gpt_pet.settings import load_settings


async def set_state(pet: PetRuntime, **delta) -> None:
    """Write session state the way a finished tick would (offline: no tick runs)."""
    session = await pet.session_service.get_session(app_name=pet.app_name, user_id=pet.user_id, session_id=pet.session_id)
    await pet.session_service.append_event(session, Event(author="test", invocation_id="setup", actions=EventActions(state_delta=delta)))


def test_submit_goal_resumes_a_limit_paused_loop_but_not_an_explicit_pause() -> None:
    async def scenario() -> None:
        async with PetRuntime(build_brain(load_settings("sim")), user_id="test") as pet:
            # paused by the limit: a submission resumes
            pet.pause()
            pet.limit_reached = True
            record = await pet.submit_goal("find the owner", ["look left"])
            assert record["goal"] == "find the owner" and record["sub_goals"] == ["look left"]
            assert pet.paused is False and pet.limit_reached is False
            assert (await pet.state())[STATE_PENDING_GOALS] == [record]
            # paused by the owner: a submission only queues
            pet.pause()
            await pet.submit_goal("say hi")
            assert pet.paused is True
            assert [g["goal"] for g in (await pet.state())[STATE_PENDING_GOALS]] == ["find the owner", "say hi"]
            assert [g["id"] for g in (await pet.state())[STATE_PENDING_GOALS]] == [1, 2]

    asyncio.run(scenario())


def test_submitting_a_goal_cancels_the_goal_in_progress_at_the_next_tick() -> None:
    async def scenario() -> None:
        notices: list[dict] = []
        async with PetRuntime(build_brain(load_settings("sim")), user_id="test", on_notice=notices.append) as pet:
            await set_state(pet, current_goal=goal_record(4, "look at the sofa", "the sofa is in view", [], 7), next_goal_id=5, tick=7)
            record = await pet.submit_goal("find the owner and say hi")
            assert record["id"] == 5
            assert not pet.brain.interrupt.requested  # no tick is running: nothing to stop
            assert [n["kind"] for n in notices] == ["cancel"]
            assert notices[0]["text"] == 'Canceled goal #4 (look at the sofa): the owner\'s goal "find the owner and say hi" starts now.'
            await pet.submit_goal("then play")  # a second goal queues behind the first: one cancel, one notice
            assert len(notices) == 1

            delta = pet._take_queued_delta(await pet._raw_state())  # what the next tick starts with
            assert delta[STATE_CURRENT_GOAL] is None
            canceled = delta[STATE_GOAL_HISTORY][0]
            assert (canceled["id"], canceled["status"], canceled["finished_tick"]) == (4, CANCELED_STATUS, 8)
            assert [g["goal"] for g in delta[STATE_PENDING_GOALS]] == ["find the owner and say hi", "then play"]
            again = pet._take_queued_delta(await pet._raw_state()) or {}
            assert STATE_CURRENT_GOAL not in again and STATE_PENDING_GOALS not in again  # consumed

    asyncio.run(scenario())


def test_a_tick_running_when_the_goal_arrives_is_interrupted() -> None:
    async def scenario() -> None:
        async with PetRuntime(build_brain(load_settings("sim")), user_id="test") as pet:
            await set_state(pet, current_goal=goal_record(2, "explore the hallway", "a new room is in view", [], 3))
            pet.running_tick = True  # as during a drive
            await pet.submit_goal("come here")
            assert pet.brain.interrupt.requested
            assert pet.brain.interrupt.reason == 'the owner gave a new goal ("come here")'

    asyncio.run(scenario())


def test_nothing_is_canceled_without_a_goal_in_progress_or_when_asked_not_to() -> None:
    async def scenario() -> None:
        notices: list[dict] = []
        async with PetRuntime(build_brain(load_settings("sim")), user_id="test", on_notice=notices.append) as pet:
            await pet.submit_goal("first goal")  # nothing in progress yet
            await set_state(pet, current_goal=rest_record(9))  # the goal-limit stand-in is not a goal
            await pet.submit_goal("second goal")
            await set_state(pet, current_goal=goal_record(3, "explore", "done", [], 9))
            pet.running_tick = True
            await pet.submit_goal("third goal", cancel_current=False)
            assert notices == [] and not pet.brain.interrupt.requested
            delta = pet._take_queued_delta(await pet._raw_state())
            assert STATE_CURRENT_GOAL not in delta and [g["goal"] for g in delta[STATE_PENDING_GOALS]] == ["first goal", "second goal", "third goal"]

    asyncio.run(scenario())


def test_the_cancel_notice_names_both_goals() -> None:
    notice = cancel_notice({"id": 11, "goal": "rest: the goal limit for this run has been reached"}, {"goal": "explore the bedroom"}, 29)
    assert notice["kind"] == "cancel" and notice["tick"] == 29
    assert notice["text"] == 'Canceled goal #11 (rest: the goal limit for this run has been reached): the owner\'s goal "explore the bedroom" starts now.'
