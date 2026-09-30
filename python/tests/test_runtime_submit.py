"""Offline: submitting an owner goal wakes a loop that paused itself on the goal limit.

Uses the real brain and an in-memory session; the MCP toolset never connects because no tick runs.
"""
from __future__ import annotations

import asyncio

from gpt_pet.brain import build_brain
from gpt_pet.goals import STATE_PENDING_GOALS
from gpt_pet.runtime import PetRuntime
from gpt_pet.settings import load_settings


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
