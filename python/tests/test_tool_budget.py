"""Offline: the executor's per-tick tool budget, as a pure counter."""
from __future__ import annotations

from gpt_pet.agents.executor import BUDGET_MARKER, ToolBudget, is_budget_refusal


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
