"""Goal memory: the goal setter's decision schema and the deterministic step that applies it.

The goal setter never edits memory itself. It emits a `GoalDecision`; `apply_decision` (pure)
turns that into the next goal-memory state, and `make_update_goal_memory` wraps it as a workflow
function node that reads and writes session state. The bookkeeping stays inspectable and
testable without an LLM.

Goals queued from the portal (`pending_goals`) are adopted deterministically: whenever a new goal
is needed, the first queued goal wins over the LLM's own proposal, and it starts even when the
run's goal limit has been reached (only the pet's own goals are limited).
"""
from __future__ import annotations

import copy
from collections.abc import Iterable
from typing import Any, Callable, Literal

from google.adk.agents.context import Context
from pydantic import BaseModel, Field, model_validator

from gpt_pet.settings import BrainSettings

STATE_TICK = "tick"
STATE_NEXT_GOAL_ID = "next_goal_id"
STATE_CURRENT_GOAL = "current_goal"
STATE_GOAL_HISTORY = "goal_history"
STATE_PENDING_GOALS = "pending_goals"
STATE_GOAL_DECISION = "goal_decision"
STATE_LAST_REPORT = "last_report"
STATE_GOAL_LIMIT = "goal_limit"
"""How many goals this run may start (set by the runtime from `max_goals_per_run` plus any
extensions from the portal); None = unlimited."""
STATE_GOALS_STARTED = "goals_started"
"""Goals started so far this run (the history is capped, so this is a separate counter)."""
STATE_LIMIT_REACHED = "goal_limit_reached"

MEMORY_KEYS = (
    STATE_TICK,
    STATE_NEXT_GOAL_ID,
    STATE_CURRENT_GOAL,
    STATE_GOAL_HISTORY,
    STATE_PENDING_GOALS,
    STATE_GOAL_LIMIT,
    STATE_GOALS_STARTED,
    STATE_LIMIT_REACHED,
)

REST_STATUS = "limit"

MAX_SUB_GOALS = 3


def initial_state() -> dict[str, Any]:
    """Session state for a fresh pet. Every reader also tolerates missing keys."""
    return {
        STATE_TICK: 0,
        STATE_NEXT_GOAL_ID: 1,
        STATE_CURRENT_GOAL: None,
        STATE_GOAL_HISTORY: [],
        STATE_PENDING_GOALS: [],
        STATE_LAST_REPORT: "",
        STATE_GOAL_LIMIT: None,
        STATE_GOALS_STARTED: 0,
        STATE_LIMIT_REACHED: False,
    }


class GoalDecision(BaseModel):
    """What the goal setter emits each tick."""

    previous_goal_status: Literal["none", "continue", "done", "abandoned"] = Field(
        description=(
            "'none' if there was no current goal, 'continue' to keep working on it, "
            "'done' when its success criteria are met, 'abandoned' when it is not achievable."
        )
    )
    goal: str | None = Field(
        default=None,
        description="The new goal. Required unless previous_goal_status is 'continue'.",
    )
    success_criteria: str | None = Field(
        default=None,
        description="One observable sentence the executor's report can confirm.",
    )
    sub_goals: list[str] = Field(
        default_factory=list,
        description="Optional ordered steps, at most three, one navigation action each.",
    )
    reasoning: str = Field(description="One sentence.")

    @model_validator(mode="after")
    def _goal_required_unless_continue(self) -> "GoalDecision":
        if self.previous_goal_status != "continue" and not (self.goal or "").strip():
            raise ValueError("goal is required unless previous_goal_status is 'continue'")
        return self


def _clean_steps(steps: Iterable[str] | None) -> list[str]:
    return [step.strip() for step in (steps or []) if step and step.strip()][:MAX_SUB_GOALS]


def goal_record(goal_id: int, goal: str, success_criteria: str, sub_goals: Iterable[str] | None, tick: int) -> dict[str, Any]:
    return {
        "id": goal_id,
        "goal": goal.strip(),
        "success_criteria": success_criteria.strip(),
        "sub_goals": _clean_steps(sub_goals),
        "status": "active",
        "created_tick": tick,
        "attempts": 0,
    }


def default_criteria(goal: str) -> str:
    return f"The executor's report shows the pet did this: {goal.strip()}"


def new_goal_record(goal_id: int, decision: GoalDecision, brain: BrainSettings, tick: int) -> dict[str, Any]:
    goal_text = (decision.goal or "").strip() or brain.default_goal
    criteria = (decision.success_criteria or "").strip() or default_criteria(goal_text)
    return goal_record(goal_id, goal_text, criteria, decision.sub_goals, tick)


def pending_goal_record(goal_id: int, goal: str, sub_goals: Iterable[str] | None = None, source: str = "portal") -> dict[str, Any]:
    """A goal waiting in the queue (submitted from the portal)."""
    return {"id": goal_id, "goal": goal.strip(), "sub_goals": _clean_steps(sub_goals), "source": source}


def rest_record(tick: int) -> dict[str, Any]:
    """Stands in for a goal once the run's goal limit is reached; the executor does nothing with it."""
    return {
        "id": 0,
        "goal": "rest: the goal limit for this run has been reached",
        "success_criteria": "none",
        "sub_goals": [],
        "status": REST_STATUS,
        "created_tick": tick,
        "attempts": 0,
    }


def record_from_pending(pending: dict[str, Any], tick: int) -> dict[str, Any]:
    goal_text = str(pending.get("goal", "")).strip()
    return goal_record(int(pending.get("id") or 0), goal_text, default_criteria(goal_text), pending.get("sub_goals"), tick)


def apply_decision(state: dict[str, Any], decision: GoalDecision, brain: BrainSettings) -> dict[str, Any]:
    """Pure transition of the goal-memory keys. Returns a new dict; the input is not mutated.

    `attempts` counts executor stretches already spent on the current goal. Continuing a goal
    whose attempts reach `brain.max_goal_attempts` abandons it automatically. A new goal comes
    from the pending queue first, then from the decision.
    """
    new = copy.deepcopy(state)
    tick = int(new.get(STATE_TICK) or 0) + 1
    history: list[dict[str, Any]] = list(new.get(STATE_GOAL_HISTORY) or [])
    pending: list[dict[str, Any]] = list(new.get(STATE_PENDING_GOALS) or [])
    current: dict[str, Any] | None = new.get(STATE_CURRENT_GOAL)
    next_id = int(new.get(STATE_NEXT_GOAL_ID) or 1)
    limit_raw = new.get(STATE_GOAL_LIMIT)
    limit = int(limit_raw) if limit_raw else None
    started = int(new.get(STATE_GOALS_STARTED) or 0)
    status = decision.previous_goal_status

    if current is not None and current.get("status") == REST_STATUS:
        current = None  # a rest stand-in is not a goal: never continue it or record it
    if current is None:
        status = "none"
    elif status == "none":
        status = "abandoned"  # the setter ignored an open goal: close it rather than lose it

    if status == "continue":
        current = dict(current)
        current["attempts"] = int(current.get("attempts") or 0) + 1
        if current["attempts"] >= brain.max_goal_attempts:
            status = "abandoned"

    if status in ("done", "abandoned"):
        finished = dict(current)
        finished["status"] = status
        finished["finished_tick"] = tick
        history.insert(0, finished)
        current = None

    limit_reached = False
    if current is None:
        if pending:  # owner goals bypass the limit
            first = pending.pop(0)
            current = record_from_pending(first, tick)
            if current["id"] <= 0:
                current["id"] = next_id
            next_id = max(next_id, current["id"] + 1)
            started += 1
        elif limit is not None and started + 1 > limit:
            current = rest_record(tick)
            limit_reached = True
        else:
            current = new_goal_record(next_id, decision, brain, tick)
            next_id += 1
            started += 1

    new[STATE_GOALS_STARTED] = started
    new[STATE_LIMIT_REACHED] = limit_reached
    new[STATE_TICK] = tick
    new[STATE_NEXT_GOAL_ID] = next_id
    new[STATE_CURRENT_GOAL] = current
    new[STATE_GOAL_HISTORY] = history[: brain.goal_history_limit]
    new[STATE_PENDING_GOALS] = pending
    return new


def coerce_decision(value: Any) -> GoalDecision:
    if isinstance(value, GoalDecision):
        return value
    if isinstance(value, str):
        return GoalDecision.model_validate_json(value)
    return GoalDecision.model_validate(value)


def make_update_goal_memory(brain: BrainSettings) -> Callable[[Context, Any], dict[str, Any]]:
    """Build the workflow function node body bound to the brain settings."""

    def update_goal_memory(ctx: Context, node_input: Any) -> dict[str, Any]:
        """Apply the goal setter's decision to the goal memory; returns the active goal."""
        decision = coerce_decision(node_input)
        snapshot = {key: ctx.state.get(key) for key in MEMORY_KEYS}
        updated = apply_decision(snapshot, decision, brain)
        for key in MEMORY_KEYS:
            ctx.state[key] = updated[key]
        return dict(updated[STATE_CURRENT_GOAL])

    return update_goal_memory
