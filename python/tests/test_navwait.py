"""Offline: reading nav statuses and deciding when a drive is over. The waiting itself is live."""
from __future__ import annotations

import json

import pytest

from gpt_pet.navwait import DRIVE_OUTCOMES, DriveWait, NavStatusError, drive_outcome, status_from_tool_result


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


def test_the_model_reads_outcome_status_and_note() -> None:
    finished = DriveWait("failed", 7.04, status("failed")).as_response()
    assert finished == {"outcome": "failed", "waited_s": 7.0, "final_status": status("failed")}
    lost = DriveWait("error", 0.2, None, "get_nav_status failed").as_response()
    assert lost == {"outcome": "error", "waited_s": 0.2, "note": "get_nav_status failed"}
    assert {"succeeded", "failed", "canceled", "timeout", "error"} <= DRIVE_OUTCOMES
