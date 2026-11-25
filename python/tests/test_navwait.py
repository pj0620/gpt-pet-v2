"""Offline: reading nav statuses and deciding when a drive is over. The waiting itself is live."""
from __future__ import annotations

import json

import pytest

import asyncio

from gpt_pet.navwait import DRIVE_OUTCOMES, DriveWait, NavStatusError, drive_outcome, status_from_tool_result, wait_for_drive
from gpt_pet.settings import NavSettings


def dumped_result(payload: object, is_error: bool = False) -> dict:
    """A get_nav_status result as ADK hands it over: the dumped MCP CallToolResult."""
    return {"content": [{"type": "text", "text": json.dumps(payload)}], "isError": is_error}


def status(state: str, goal_id: str | None = "g1") -> dict:
    goal = {"goal_id": goal_id, "destination_id": "psg:1", "label": "doorway"} if goal_id else None
    return {"state": state, "goal": goal, "progress": None, "collisions": 0, "reason": None}


def test_status_is_read_from_the_dumped_tool_result() -> None:
    assert status_from_tool_result(dumped_result(status("following")))["state"] == "following"


@pytest.mark.parametrize(
    "raw",
    [
        dumped_result("server exploded", is_error=True),
        {"error": "MCP tool execution failed: connection reset"},
        dumped_result({"progress": None}),
        "not a tool result",
    ],
)
def test_unreadable_results_raise(raw: object) -> None:
    with pytest.raises(NavStatusError):
        status_from_tool_result(raw)


def test_a_drive_is_underway_until_a_terminal_state() -> None:
    assert drive_outcome(status("planning"), "g1") is None
    assert drive_outcome(status("following"), "g1") is None
    for state in ("succeeded", "failed", "canceled"):
        assert drive_outcome(status(state), "g1") == state


def test_idle_and_replaced_goals_end_the_wait() -> None:
    assert drive_outcome(status("idle", goal_id=None), "g1") == "idle"
    assert drive_outcome(status("following", goal_id="g2"), "g1") == "preempted"
    assert drive_outcome(status("following", goal_id="g2"), None) is None


def test_a_drive_knows_its_destination_planned_length_and_distance_driven() -> None:
    arrived = DriveWait("succeeded", 6.0, {**status("succeeded"), "progress": {"distance_traveled_m": 2.31}}, destination_id="psg:2", planned_m=2.4)
    assert arrived.traveled_m == 2.31 and not arrived.null
    already_there = DriveWait("succeeded", 0.5, status("succeeded"), destination_id="obj:DiningTable|1", planned_m=0.12)
    assert already_there.null and already_there.traveled_m is None
    assert not DriveWait("error", 0.2, None).null  # no plan known, so not counted as going nowhere
    assert "destination_id" not in arrived.as_response()  # the model already knows where it sent the robot


def test_the_model_reads_outcome_status_and_note() -> None:
    finished = DriveWait("failed", 7.04, status("failed")).as_response()
    assert finished == {"outcome": "failed", "waited_s": 7.0, "final_status": status("failed")}
    lost = DriveWait("error", 0.2, None, "get_nav_status failed").as_response()
    assert lost == {"outcome": "error", "waited_s": 0.2, "note": "get_nav_status failed"}
    assert {"succeeded", "failed", "canceled", "timeout", "error"} <= DRIVE_OUTCOMES


def test_a_canceled_goal_stops_the_drive_being_waited_out() -> None:
    async def scenario() -> tuple[DriveWait, int, list[str]]:
        polls, canceled = 0, []
        reason: list[str] = []

        async def get_status() -> dict:
            nonlocal polls
            polls += 1
            if polls == 3:
                reason.append('the owner gave a new goal ("come here")')  # submitted mid-drive
            return status("following")

        async def cancel() -> None:
            canceled.append("cancel_nav")

        nav = NavSettings(poll_interval_s=0.01, max_wait_s=30)
        wait = await wait_for_drive(get_status, "g1", nav, stop=lambda: reason[0] if reason else None, cancel=cancel)
        return wait, polls, canceled

    wait, polls, canceled = asyncio.run(scenario())
    assert wait.outcome == "canceled" and canceled == ["cancel_nav"] and polls == 3
    assert wait.note == 'Stopped: the owner gave a new goal ("come here"). The robot is no longer driving.'
    assert wait.status["state"] == "following"  # the last status seen before stopping
