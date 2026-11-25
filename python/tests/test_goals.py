"""Offline: the goal-memory transitions, as pure functions."""
from __future__ import annotations

import pytest
from pydantic import ValidationError

from gpt_pet.goals import (
    CANCELED_STATUS,
    STATE_CURRENT_GOAL,
    STATE_GOAL_HISTORY,
    STATE_NEXT_GOAL_ID,
    STATE_PENDING_GOALS,
    STATE_TICK,
    GoalDecision,
    GoalInterrupt,
    TargetedGoalDecision,
    apply_decision,
    cancel_current_goal,
    checked_target,
    coerce_decision,
    initial_state,
    pending_goal_record,
    rest_record,
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


def test_with_explore_ticks_an_explore_goal_at_the_attempt_cap_counts_as_done() -> None:
    explore = TargetedGoalDecision(previous_goal_status="none", goal="explore the hallway", kind="explore", reasoning="because")
    visit = TargetedGoalDecision(previous_goal_status="none", goal="reach the sofa", kind="visit", reasoning="because")
    for first, explore_done, expected in ((explore, True, "done"), (explore, False, "abandoned"), (visit, True, "abandoned")):
        state = apply_decision(initial_state(), first, BRAIN, explore_done=explore_done)
        for _ in range(BRAIN.max_goal_attempts):
            state = apply_decision(state, decide("continue"), BRAIN, explore_done=explore_done)
        assert state[STATE_GOAL_HISTORY][0]["status"] == expected, (first.kind, explore_done)


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


def test_a_targeted_goal_carries_its_kind_and_a_target_the_pet_remembers() -> None:
    decision = TargetedGoalDecision(
        previous_goal_status="none", goal="go through the doorway", kind="explore", target_id="psg:4", reasoning="unseen room"
    )
    goal = apply_decision(initial_state(), checked_target(decision, {"psg:4"}.__contains__), BRAIN)[STATE_CURRENT_GOAL]
    assert (goal["kind"], goal["target_id"]) == ("explore", "psg:4")
    unknown = checked_target(decision, {"psg:1"}.__contains__)  # Gemini named an id the pet never saw
    goal = apply_decision(initial_state(), unknown, BRAIN)[STATE_CURRENT_GOAL]
    assert goal["kind"] == "explore" and "target_id" not in goal
    plain = apply_decision(initial_state(), decide("none", "see the kitchen"), BRAIN)[STATE_CURRENT_GOAL]
    assert "kind" not in plain and "target_id" not in plain


def test_decisions_are_read_with_the_schema_the_goal_setter_answered_in() -> None:
    raw = '{"previous_goal_status": "none", "goal": "visit the sofa", "kind": "visit", "target_id": "obj:Sofa|2", "reasoning": "r"}'
    assert coerce_decision(raw).model_dump().get("target_id") is None  # the plain schema ignores the fields
    targeted = coerce_decision(raw, TargetedGoalDecision)
    assert isinstance(targeted, TargetedGoalDecision) and targeted.target_id == "obj:Sofa|2"
    s1_continue = GoalDecision(previous_goal_status="continue", reasoning="S1: still in progress")
    assert coerce_decision(s1_continue, TargetedGoalDecision).target_id is None  # S1 and fallbacks answer plainly
    with pytest.raises(ValidationError):
        coerce_decision('{"previous_goal_status": "none", "goal": "x", "kind": "dance", "reasoning": "r"}', TargetedGoalDecision)


def test_canceling_moves_the_goal_in_progress_to_the_history_and_the_owner_goal_starts() -> None:
    state = apply_decision(initial_state(), decide("none", "explore the hallway"), BRAIN)
    state = apply_decision(state, decide("done", "look at the sofa"), BRAIN)  # goal 1 done, goal 2 active
    delta = cancel_current_goal(state, history_limit=BRAIN.goal_history_limit)
    assert delta is not None and delta[STATE_CURRENT_GOAL] is None
    canceled = delta[STATE_GOAL_HISTORY][0]
    assert (canceled["id"], canceled["goal"], canceled["status"]) == (2, "look at the sofa", CANCELED_STATUS)
    assert canceled["finished_tick"] == 3  # closed by the tick that starts the owner's goal
    assert [g["id"] for g in delta[STATE_GOAL_HISTORY]] == [2, 1]
    assert state[STATE_CURRENT_GOAL]["status"] == "active"  # the input is not mutated

    owner = pending_goal_record(3, "find the owner and say hi")
    state = {**state, **delta, STATE_PENDING_GOALS: [owner]}
    state = apply_decision(state, decide("none", "find the owner and say hi"), BRAIN)
    assert state[STATE_CURRENT_GOAL]["goal"] == "find the owner and say hi" and state[STATE_CURRENT_GOAL]["status"] == "active"
    assert [g["status"] for g in state[STATE_GOAL_HISTORY]] == [CANCELED_STATUS, "done"]
    assert state[STATE_PENDING_GOALS] == []


def test_nothing_to_cancel_without_a_goal_in_progress() -> None:
    assert cancel_current_goal(initial_state(), history_limit=5) is None
    assert cancel_current_goal({**initial_state(), STATE_CURRENT_GOAL: rest_record(4)}, history_limit=5) is None  # the limit stand-in is no goal


def test_the_interrupt_carries_its_reason_until_cleared() -> None:
    interrupt = GoalInterrupt()
    assert not interrupt.requested and interrupt.reason is None
    interrupt.request("the owner gave a new goal")
    assert interrupt.requested and interrupt.reason == "the owner gave a new goal"
    interrupt.clear()
    assert not interrupt.requested
