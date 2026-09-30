"""Offline: the goal-memory transitions, as pure functions."""
from __future__ import annotations

import pytest
from pydantic import ValidationError

from gpt_pet.goals import (
    STATE_CURRENT_GOAL,
    STATE_GOAL_HISTORY,
    STATE_NEXT_GOAL_ID,
    STATE_TICK,
    GoalDecision,
    apply_decision,
    initial_state,
)
from gpt_pet.settings import BrainSettings

BRAIN = BrainSettings(max_goal_attempts=3, goal_history_limit=2, default_goal="look around")


def decide(status: str, goal: str | None = None, **extra) -> GoalDecision:
    return GoalDecision(previous_goal_status=status, goal=goal, reasoning="because", **extra)


def test_goal_is_required_unless_continuing() -> None:
    with pytest.raises(ValidationError, match="goal is required"):
        decide("done")
    assert decide("continue").goal is None
    assert decide("none", "find a person").goal == "find a person"


def test_first_tick_creates_goal_one() -> None:
    state = apply_decision(initial_state(), decide("none", "see the kitchen", success_criteria="kitchen in view", sub_goals=["go to psg:1"]), BRAIN)
    goal = state[STATE_CURRENT_GOAL]
    assert state[STATE_TICK] == 1
    assert goal["id"] == 1
    assert goal["goal"] == "see the kitchen"
    assert goal["success_criteria"] == "kitchen in view"
    assert goal["sub_goals"] == ["go to psg:1"]
    assert goal["status"] == "active"
    assert goal["attempts"] == 0
    assert goal["created_tick"] == 1
    assert state[STATE_NEXT_GOAL_ID] == 2
    assert state[STATE_GOAL_HISTORY] == []


def test_input_state_is_not_mutated() -> None:
    before = initial_state()
    apply_decision(before, decide("none", "x"), BRAIN)
    assert before == initial_state()


def test_continue_keeps_the_goal_and_counts_attempts() -> None:
    state = apply_decision(initial_state(), decide("none", "see the kitchen"), BRAIN)
    state = apply_decision(state, decide("continue"), BRAIN)
    assert state[STATE_TICK] == 2
    assert state[STATE_CURRENT_GOAL]["id"] == 1
    assert state[STATE_CURRENT_GOAL]["attempts"] == 1
    assert state[STATE_GOAL_HISTORY] == []


def test_done_moves_the_goal_to_history_and_starts_the_next() -> None:
    state = apply_decision(initial_state(), decide("none", "see the kitchen"), BRAIN)
    state = apply_decision(state, decide("done", "greet the person"), BRAIN)
    assert state[STATE_CURRENT_GOAL]["id"] == 2
    assert state[STATE_CURRENT_GOAL]["goal"] == "greet the person"
    finished = state[STATE_GOAL_HISTORY][0]
    assert finished["id"] == 1
    assert finished["status"] == "done"
    assert finished["finished_tick"] == 2


def test_continuing_at_the_attempt_cap_abandons_and_falls_back_to_the_default_goal() -> None:
    state = apply_decision(initial_state(), decide("none", "reach the sofa"), BRAIN)
    state = apply_decision(state, decide("continue"), BRAIN)  # attempts 1
    state = apply_decision(state, decide("continue"), BRAIN)  # attempts 2
    assert state[STATE_CURRENT_GOAL]["id"] == 1
    state = apply_decision(state, decide("continue"), BRAIN)  # attempts 3 == cap -> abandoned
    assert state[STATE_GOAL_HISTORY][0]["status"] == "abandoned"
    assert state[STATE_GOAL_HISTORY][0]["attempts"] == 3
    assert state[STATE_CURRENT_GOAL]["id"] == 2
    assert state[STATE_CURRENT_GOAL]["goal"] == "look around"


def test_none_with_an_open_goal_closes_it_as_abandoned() -> None:
    state = apply_decision(initial_state(), decide("none", "reach the sofa"), BRAIN)
    state = apply_decision(state, decide("none", "find the dog"), BRAIN)
    assert state[STATE_GOAL_HISTORY][0]["status"] == "abandoned"
    assert state[STATE_CURRENT_GOAL]["goal"] == "find the dog"


def test_history_is_capped_newest_first() -> None:
    state = initial_state()
    for n in range(4):
        state = apply_decision(state, decide("done" if n else "none", f"goal {n}"), BRAIN)
    assert [g["goal"] for g in state[STATE_GOAL_HISTORY]] == ["goal 2", "goal 1"]
    assert state[STATE_CURRENT_GOAL]["goal"] == "goal 3"
    assert state[STATE_CURRENT_GOAL]["id"] == 4


def test_pending_goal_is_adopted_before_the_llm_proposal() -> None:
    from gpt_pet.goals import STATE_PENDING_GOALS, pending_goal_record

    state = initial_state()
    state[STATE_PENDING_GOALS] = [pending_goal_record(5, "find the owner", ["look in the living room"])]
    state[STATE_NEXT_GOAL_ID] = 6
    state = apply_decision(state, decide("none", "sniff the sofa"), BRAIN)
    goal = state[STATE_CURRENT_GOAL]
    assert goal["id"] == 5
    assert goal["goal"] == "find the owner"
    assert goal["sub_goals"] == ["look in the living room"]
    assert goal["success_criteria"]
    assert state[STATE_PENDING_GOALS] == []
    assert state[STATE_NEXT_GOAL_ID] == 6
    # the queue is untouched while a goal is continued
    state[STATE_PENDING_GOALS] = [pending_goal_record(7, "greet the dog")]
    state = apply_decision(state, decide("continue"), BRAIN)
    assert state[STATE_CURRENT_GOAL]["id"] == 5
    assert [g["goal"] for g in state[STATE_PENDING_GOALS]] == ["greet the dog"]
    # ...and consumed when the goal finishes
    state = apply_decision(state, decide("done", "whatever the model wanted"), BRAIN)
    assert state[STATE_CURRENT_GOAL]["goal"] == "greet the dog"
    assert state[STATE_GOAL_HISTORY][0]["id"] == 5


def test_goal_limit_counts_goals_started_and_a_raised_limit_continues_the_run() -> None:
    from gpt_pet.goals import (
        REST_STATUS,
        STATE_GOAL_LIMIT,
        STATE_GOALS_STARTED,
        STATE_LIMIT_REACHED,
        STATE_PENDING_GOALS,
        pending_goal_record,
    )

    state = initial_state()
    state[STATE_GOAL_LIMIT] = 2
    state = apply_decision(state, decide("none", "goal one"), BRAIN)
    assert state[STATE_GOALS_STARTED] == 1
    state = apply_decision(state, decide("done", "goal two"), BRAIN)
    assert state[STATE_CURRENT_GOAL]["id"] == 2 and state[STATE_GOALS_STARTED] == 2 and not state[STATE_LIMIT_REACHED]
    state = apply_decision(state, decide("done", "goal three"), BRAIN)
    assert state[STATE_LIMIT_REACHED] is True
    assert state[STATE_CURRENT_GOAL]["status"] == REST_STATUS
    assert state[STATE_GOALS_STARTED] == 2
    assert [g["id"] for g in state[STATE_GOAL_HISTORY]] == [2, 1]
    # the rest stand-in is never continued or recorded; one more goal lets the run go on
    state = apply_decision(state, decide("continue"), BRAIN)
    assert state[STATE_CURRENT_GOAL]["status"] == REST_STATUS and len(state[STATE_GOAL_HISTORY]) == 2
    state[STATE_GOAL_LIMIT] = 3
    state[STATE_PENDING_GOALS] = [pending_goal_record(9, "owner goal")]
    state = apply_decision(state, decide("none", "model goal"), BRAIN)
    assert state[STATE_CURRENT_GOAL]["goal"] == "owner goal" and state[STATE_LIMIT_REACHED] is False
    assert state[STATE_GOALS_STARTED] == 3 and len(state[STATE_GOAL_HISTORY]) == 2
    state = apply_decision(state, decide("done", "another"), BRAIN)
    assert state[STATE_LIMIT_REACHED] is True and state[STATE_GOALS_STARTED] == 3


def test_owner_goals_bypass_the_goal_limit() -> None:
    from gpt_pet.goals import REST_STATUS, STATE_GOAL_LIMIT, STATE_GOALS_STARTED, STATE_LIMIT_REACHED, STATE_PENDING_GOALS, pending_goal_record

    state = initial_state()
    state[STATE_GOAL_LIMIT] = 1
    state = apply_decision(state, decide("none", "pet goal"), BRAIN)
    state = apply_decision(state, decide("done", "another pet goal"), BRAIN)
    assert state[STATE_LIMIT_REACHED] and state[STATE_CURRENT_GOAL]["status"] == REST_STATUS
    # a goal from the portal starts despite the limit, counts as used, and clears the flag
    state[STATE_PENDING_GOALS] = [pending_goal_record(5, "owner goal", ["step one"])]
    state = apply_decision(state, decide("continue"), BRAIN)
    assert state[STATE_LIMIT_REACHED] is False
    assert state[STATE_CURRENT_GOAL]["goal"] == "owner goal" and state[STATE_CURRENT_GOAL]["id"] == 5
    assert state[STATE_GOALS_STARTED] == 2
    assert state[STATE_PENDING_GOALS] == []
    # once it is done, the pet's own goals are limited again
    state = apply_decision(state, decide("done", "pet goal three"), BRAIN)
    assert state[STATE_LIMIT_REACHED] is True and state[STATE_GOAL_HISTORY][0]["goal"] == "owner goal"
