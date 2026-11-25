"""Live: submitting a goal cancels the goal in progress (real simulator, real model, no mocks).

The pet runs ticks until one is driving, and the owner submits a goal a second into the drive.
That tick admits no further tool calls, the drive stops (canceled), and the tick ends within
seconds; the next tick starts the owner's goal with the old one in the history as canceled.
"""
from __future__ import annotations

import asyncio
import logging
import time

import pytest
from google.adk.events.event import Event

from gpt_pet.agents.executor import EXECUTOR_NAME, is_cancel_refusal
from gpt_pet.brain import build_brain
from gpt_pet.goals import CANCELED_STATUS
from gpt_pet.mcp_proxy import McpProxy
from gpt_pet.runtime import PetRuntime
from gpt_pet.settings import Settings

pytestmark = [pytest.mark.live, pytest.mark.timeout(900)]

log = logging.getLogger("gpt_pet.tests")

CANCEL_WITHIN_S = 30.0
"""The interrupted tick must end this soon after the submission (a drive alone may take 120 s)."""
OWNER_GOAL = "turn around and look at whatever is behind you"


async def test_a_submitted_goal_cancels_the_goal_in_progress(mcp_server: str, gemini_key: str, sim_settings: Settings) -> None:
    seen: list[tuple[int, Event]] = []
    notices: list[dict] = []
    brain = build_brain(sim_settings)
    async with PetRuntime(brain, user_id="live-cancel", on_event=lambda event, tick: seen.append((tick, event)), on_notice=notices.append) as pet:
        first = await pet.run_tick()
        assert (first.state.get("current_goal") or {}).get("status") == "active", first.state

        # the owner submits while the robot is driving: ticks run in the background until one
        # starts a drive (most do), and the goal arrives a second into it
        tick_no, drive_call, running = 0, None, None
        for _ in range(4):
            tick_no = pet.ticks + 1
            running = asyncio.create_task(pet.run_tick())
            while not running.done():
                calls = [call for tick, event in seen if tick == tick_no for call in event.get_function_calls()]
                drive_call = next((call for call in calls if call.name == "set_nav_goal"), None)
                if drive_call is not None:
                    break
                await asyncio.sleep(0.05)
            if drive_call is not None:
                break
            await running
        assert drive_call is not None and running is not None, "no tick drove in four tries"
        await asyncio.sleep(1.0)  # well into the drive
        responded = {r.id for tick, event in seen if tick == tick_no for r in event.get_function_responses()}
        mid_drive = drive_call.id not in responded  # False if it was a drive to where the robot already was
        submitted_at = time.time()
        await pet.submit_goal(OWNER_GOAL)
        assert pet.brain.interrupt.requested
        second = await asyncio.wait_for(running, timeout=CANCEL_WITHIN_S)
        ended_after_s = time.time() - submitted_at
        log.info("tick %d ended %.1f s after the submission (mid-drive: %s)", tick_no, ended_after_s, mid_drive)
        assert ended_after_s < CANCEL_WITHIN_S

        # no tool call was admitted after the submission; the drive under way was stopped
        called_at = {call.id: event.timestamp for tick, event in seen if tick == tick_no for call in event.get_function_calls()}
        for tick, event in seen:
            if tick != tick_no:
                continue
            for response in event.get_function_responses():
                if called_at.get(response.id, 0.0) > submitted_at:
                    assert is_cancel_refusal(response.response), f"{response.name} ran after the cancel: {response.response}"
                if response.id == drive_call.id and mid_drive:
                    drive = response.response["drive"]
                    log.info("the drive under way ended %s after %.1f s: %s", drive["outcome"], drive["waited_s"], drive.get("note"))
                    assert drive["outcome"] == "canceled" and "the owner gave a new goal" in drive["note"], drive
        proxy = McpProxy(mcp_server)
        try:
            nav = (await proxy.call("get_nav_status", {})).data or {}
        finally:
            await proxy.close()
        assert nav.get("state") not in ("planning", "following"), f"the robot is still driving: {nav}"

        in_progress = second.state["current_goal"]
        assert in_progress and in_progress["status"] == "active"
        assert [n["kind"] for n in notices if n["kind"] == "cancel"] == ["cancel"]

        # the next tick: the owner's goal starts at once, the old one is canceled
        third = await pet.run_tick()
        assert not pet.brain.interrupt.requested
        current = third.state["current_goal"]
        assert current["goal"] == OWNER_GOAL and current["status"] == "active"
        canceled = third.state["goal_history"][0]
        assert (canceled["id"], canceled["status"]) == (in_progress["id"], CANCELED_STATUS)
        if sim_settings.s1.enabled:
            setter = [event for tick, event in seen if tick == tick_no + 1 and event.author == "goal_setter" and event.content]
            assert setter and (setter[0].custom_metadata or {}).get("s1", {}).get("action") == "owner goal"
        assert pet.stats.snapshot()["goals"]["canceled"] == 1
