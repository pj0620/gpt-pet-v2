"""Offline: the executor's per-tick tool budget, as a pure counter."""
from __future__ import annotations

from types import SimpleNamespace

from gpt_pet.agents.executor import (
    BUDGET_MARKER,
    CANCEL_MARKER,
    HERE_MARKER,
    ToolBudget,
    here_refusal,
    is_budget_refusal,
    is_cancel_refusal,
    make_tool_budget_callback,
)
from gpt_pet.goals import GoalInterrupt
from gpt_pet.places import Places


def test_calls_are_admitted_up_to_the_limit_then_refused() -> None:
    budget = ToolBudget({"get_nav_status": 2})
    assert budget.admit("inv-1", "get_nav_status") is None
    assert budget.admit("inv-1", "get_nav_status") is None
    refusal = budget.admit("inv-1", "get_nav_status")
    assert refusal is not None
    assert refusal.startswith(BUDGET_MARKER)
    assert "limit 2" in refusal
    assert is_budget_refusal({"error": refusal})
    assert not is_budget_refusal({"error": "MCP tool execution failed"})
    assert not is_budget_refusal("text")


def test_unlisted_tools_are_uncapped() -> None:
    budget = ToolBudget({"do_rotate": 1})
    for _ in range(50):
        assert budget.admit("inv-1", "get_current_view") is None


def test_counts_reset_for_a_new_invocation() -> None:
    budget = ToolBudget({"do_rotate": 1})
    assert budget.admit("tick-1", "do_rotate") is None
    assert budget.admit("tick-1", "do_rotate") is not None
    assert budget.admit("tick-2", "do_rotate") is None
    assert budget.counts_for("tick-2") == {"do_rotate": 1}


def test_a_drive_to_where_the_robot_already_is_is_refused_as_a_guard_not_an_error() -> None:
    places = Places(here_m=2.0)
    view = {
        "agent": {"x": 1.0, "z": 2.0, "yaw": 90.0},
        "destinations": [
            {"id": "obj:Plate|1", "kind": "object", "label": "Plate", "distance_m": 1.6},
            {"id": "obj:Sofa|2", "kind": "object", "label": "Sofa", "distance_m": 3.4},
        ],
    }
    places.remember("get_current_view", {}, {"result": view, "isError": False}, tick=1)
    refusal = here_refusal(places, "obj:Plate|1")
    assert refusal is not None and refusal.startswith(f"{HERE_MARKER}: Plate is within reach")
    assert is_budget_refusal({"error": refusal})  # counted as the brain's own refusal, not a tool failure
    assert here_refusal(places, "obj:Sofa|2") is None and here_refusal(places, "psg:9") is None


def test_every_call_is_refused_once_the_owner_cancels_the_goal() -> None:
    interrupt = GoalInterrupt()
    before_tool = make_tool_budget_callback({"get_current_view": 2}, interrupt=interrupt)
    tool, context = SimpleNamespace(name="get_current_view"), SimpleNamespace(invocation_id="tick-1")
    assert before_tool(tool, {}, context) is None  # admitted, one of two used
    interrupt.request('the owner gave a new goal ("come here")')
    refused = before_tool(SimpleNamespace(name="set_nav_goal"), {"destination_id": "psg:1"}, context)
    assert refused["error"].startswith(f'{CANCEL_MARKER}: the owner gave a new goal ("come here")')
    assert "Do not call any more tools" in refused["error"]
    assert is_cancel_refusal(refused) and is_budget_refusal(refused)  # the brain's own refusal, not a tool error
    assert not is_cancel_refusal({"error": f"{BUDGET_MARKER}: limit"})
    interrupt.clear()
    assert before_tool(tool, {}, context) is None  # the refused calls spent no budget
    assert before_tool(tool, {}, context)["error"].startswith(BUDGET_MARKER)
