"""Offline: System 1's decisions as pure functions over real genai objects and nimble's real reply
shapes. The model itself is exercised live (tests/live/test_s1_live.py)."""
from __future__ import annotations

import pytest
from google.genai import types

from gpt_pet.agents.executor import BUDGET_MARKER
from gpt_pet.goals import GoalDecision
from gpt_pet.s1 import (
    ASK_GEMINI,
    INJECTED_CALL_SIGNATURE,
    REST_REPLY,
    Answer,
    S1Error,
    Step,
    action_options,
    certainty,
    executor_questions,
    executor_report,
    executor_state,
    goal_escalation,
    judge_turn,
    parse_answer,
    rule_turn,
    tool_steps,
)

LIMITS = {"get_current_view": 4, "set_nav_goal": 2, "do_rotate": 2}
GOAL = {
    "id": 2,
    "goal": "get a close look at the sofa",
    "success_criteria": "The pet is within 1 m of the sofa",
    "sub_goals": [],
    "status": "active",
    "attempts": 0,
}
VIEW = {
    "agent": {"x": 1.0, "z": 2.0, "yaw": 90.0},
    "destinations": [
        {"marker": 1, "id": "obj:Painting|1", "label": "Painting", "description": "Painting, 1.3 m ahead-left", "distance_m": 1.3},
        {"marker": 2, "id": "obj:Sofa|2", "label": "Sofa", "description": "Sofa, 2.0 m ahead", "distance_m": 2.0},
        {"marker": 3, "id": "psg:4", "label": "opening", "description": "passage: opening ahead, ~2.0 m wide", "distance_m": 3.1},
    ],
}
# Exactly what Ollama 0.35 + nimble answered in a probe on this machine.
NIMBLE_CHOICE = {
    "type": "choice",
    "choice": "go_1",
    "probabilities": {"go_1": 0.8858, "go_2": 0.0267, "go_3": 0.0014, "turn_left": 0.0570, "ask_gemini": 0.0290},
    "confidence": 0.7021,
}
NIMBLE_NOUL = {"type": "noul", "noul": 0.0102}


def look() -> Step:
    return Step("get_current_view", {}, {"result": VIEW, "isError": False})


def drive(outcome: str = "succeeded") -> Step:
    response = {"result": {"goal_id": "g1", "label": "Sofa"}, "isError": False, "drive": {"outcome": outcome, "waited_s": 5.7}}
    return Step("set_nav_goal", {"destination_id": "obj:Sofa|2"}, response)


def turn() -> Step:
    return Step("do_rotate", {"action": "RotateLeft", "degrees": 90}, {"result": "Rotated", "isError": False})


def answer(choice: str, sure: float, p: float | None = None) -> Answer:
    return Answer(choice, sure, {choice: sure if p is None else p})


def test_nimbles_reply_parses_into_a_choice_and_a_yes_no() -> None:
    step = parse_answer(NIMBLE_CHOICE)
    assert (step.choice, step.certainty) == ("go_1", 0.7021)
    met = parse_answer(NIMBLE_NOUL)
    assert met.choice == "false" and met.probabilities == {"true": 0.0102, "false": 0.9898}
    assert met.certainty == pytest.approx(0.9796)  # the margin |p(yes) - p(no)|
    assert parse_answer({"type": "noul", "noul": 0.942}).certainty == pytest.approx(0.884)  # at the armchair: met
    no_choice = parse_answer({"type": "choice", "probabilities": {"a": 0.2, "b": 0.8}})
    assert no_choice.choice == "b" and 0 < no_choice.certainty < 1
    for bad in ({"type": "noul"}, {"type": "choice"}, "text", None):
        with pytest.raises(S1Error):
            parse_answer(bad)


def test_certainty_is_one_minus_normalized_entropy() -> None:
    assert certainty({"a": 1.0, "b": 0.0}) == 1.0
    assert certainty({"a": 0.5, "b": 0.5}) == pytest.approx(0.0)
    assert certainty({"only": 1.0}) == 1.0
    assert certainty(NIMBLE_CHOICE["probabilities"]) == pytest.approx(0.7021, abs=0.001)  # the API's own number


def test_tool_steps_pair_calls_and_responses_by_id_then_by_name() -> None:
    contents = [
        types.Content(role="user", parts=[types.Part(text='{"goal": "get a close look at the sofa"}')]),
        types.Content(role="model", parts=[types.Part(function_call=types.FunctionCall(id="fc-1", name="get_current_view", args={}))]),
        types.Content(
            role="user",
            parts=[types.Part(function_response=types.FunctionResponse(id="fc-1", name="get_current_view", response={"result": VIEW, "isError": False}))],
        ),
        types.Content(role="model", parts=[types.Part(function_call=types.FunctionCall(name="set_nav_goal", args={"destination_id": "obj:Sofa|2"}))]),
        types.Content(role="user", parts=[types.Part(function_response=types.FunctionResponse(name="set_nav_goal", response=drive().response))]),
    ]
    steps = tool_steps(contents)
    assert [s.name for s in steps] == ["get_current_view", "set_nav_goal"]
    assert steps[0].result == VIEW and steps[1].response["drive"]["outcome"] == "succeeded"


def test_rules_take_the_mechanical_turns() -> None:
    assert rule_turn({**GOAL, "status": "limit"}, [], LIMITS).text == REST_REPLY
    first = rule_turn(GOAL, [], LIMITS)
    assert (first.kind, first.tool, first.by) == ("call", "get_current_view", "rules")
    assert rule_turn(GOAL, [look()], LIMITS) is None  # a judgment: ask nimble
    assert rule_turn(GOAL, [look(), drive()], LIMITS).tool == "get_current_view"
    assert rule_turn(GOAL, [look(), turn()], LIMITS).tool == "get_current_view"


def test_rules_hand_errors_refusals_and_dead_ends_to_gemini() -> None:
    refused = Step("do_rotate", {}, {"error": f"{BUDGET_MARKER}: do_rotate was already called 2 times"})
    failed = Step("set_nav_goal", {}, {"result": "Unknown destination id", "isError": True})
    assert rule_turn(GOAL, [look(), refused], LIMITS).reason == "tool budget"
    assert rule_turn(GOAL, [look(), failed], LIMITS).reason == "tool error"
    assert rule_turn(GOAL, [look(), Step("get_map", {}, {"result": "png", "isError": False})], LIMITS).reason == "after get_map"
    out_of_looks = [look(), turn(), look(), turn(), look(), drive(), look(), drive()]
    assert rule_turn(GOAL, out_of_looks, LIMITS).reason == "no looks left"


def test_questions_offer_the_destinations_in_view_turns_and_gemini() -> None:
    options = action_options(VIEW)
    assert list(options) == ["go_1", "go_2", "go_3", "turn_left", "turn_right", ASK_GEMINI]
    assert options["go_2"]["text"] == "Drive to Sofa, 2.0 m ahead"
    assert options["go_2"]["destination"]["id"] == "obj:Sofa|2"
    assert "go_2" not in action_options(VIEW, exclude={"obj:Sofa|2"})  # no repeat drives in one stretch
    assert list(executor_questions(options, ask_met=False)) == ["next"]
    both = executor_questions(options, ask_met=True)
    assert both["criteria_met"]["type"] == "noul" and set(both["next"]["criteria"]) == set(options)
    state = executor_state(GOAL, [look(), drive(), look()])
    assert state["done_this_stretch"] == ["drove to Sofa (succeeded after 5.7 s)"]
    no_move = drive()
    no_move.response["result"]["path"] = {"length_m": 0.0}  # planned from where the pet already stood
    assert executor_state(GOAL, [look(), no_move, look()])["done_this_stretch"] == ["was already at Sofa"]
    assert state["in_view"] == ["Painting, 1.3 m ahead-left", "Sofa, 2.0 m ahead", "passage: opening ahead, ~2.0 m wide"]


def test_a_confident_choice_becomes_the_matching_tool_call() -> None:
    options = action_options(VIEW)
    go = judge_turn({"next": answer("go_2", 0.9)}, options, [look()], LIMITS, 0.75, "nimble")
    assert (go.kind, go.tool, go.args, go.by) == ("call", "set_nav_goal", {"destination_id": "obj:Sofa|2"}, "nimble")
    left = judge_turn({"next": answer("turn_left", 0.8)}, options, [look()], LIMITS, 0.75, "nimble")
    assert (left.tool, left.args) == ("do_rotate", {"action": "RotateLeft", "degrees": 90})


def test_uncertain_or_impossible_choices_go_to_gemini() -> None:
    options = action_options(VIEW)
    assert judge_turn({"next": answer("go_2", 0.6)}, options, [look()], LIMITS, 0.75, "nimble").reason == "unsure"
    assert judge_turn({"next": answer(ASK_GEMINI, 0.9)}, options, [look()], LIMITS, 0.75, "nimble").reason == "asked for Gemini"
    turned_twice = [look(), turn(), look(), turn(), look()]
    assert judge_turn({"next": answer("turn_right", 0.9)}, options, turned_twice, LIMITS, 0.75, "nimble").reason == "no turns left"


def test_s1_reports_when_the_criteria_are_met_or_the_drives_run_out() -> None:
    options = action_options(VIEW)
    steps = [look(), drive(), look()]
    met = Answer("true", 0.9, {"true": 0.99, "false": 0.01})
    done = judge_turn({"next": answer("go_3", 0.9), "criteria_met": met}, options, steps, LIMITS, 0.75, "nimble")
    assert done.kind == "report"
    assert done.text == (
        "Saw: Painting (1.3 m), Sofa (2.0 m), opening (3.1 m).\n"
        "Did: drove to Sofa (succeeded after 5.7 s).\n"
        "Final nav state: succeeded.\n"
        "Success criteria met (S1 nimble, p=0.99)."
    )
    not_met = Answer("false", 0.9, {"true": 0.02, "false": 0.98})
    two_drives = [look(), drive(), look(), drive("failed"), look()]
    tired = judge_turn({"next": answer("go_2", 0.9), "criteria_met": not_met}, options, two_drives, LIMITS, 0.75, "nimble")
    assert tired.kind == "report" and "Final nav state: failed." in tired.text and "not met yet (S1 nimble, p=0.02)" in tired.text
    assert executor_report([look()], met=False, p_met=0.0, model="nimble").startswith("Saw: Painting (1.3 m)")


def test_s1_calls_carry_the_documented_injected_call_signature() -> None:
    call = types.Part(function_call=types.FunctionCall(name="get_current_view", args={}), thought_signature=INJECTED_CALL_SIGNATURE)
    assert call.model_dump(mode="json", exclude_none=True)["thought_signature"] == "skip_thought_signature_validator"


def test_the_goal_gate_leaves_goal_boundaries_to_gemini() -> None:
    state = {"current_goal": GOAL, "pending_goals": [], "last_report": "Drove toward the sofa; blocked."}
    assert goal_escalation(state, max_goal_attempts=3) is None
    assert goal_escalation({**state, "current_goal": None}, 3) == "no active goal"
    assert goal_escalation({**state, "current_goal": {**GOAL, "status": "limit"}}, 3) == "no active goal"
    assert goal_escalation({**state, "pending_goals": [{"goal": "find the owner"}]}, 3) == "owner goals queued"
    assert goal_escalation({**state, "current_goal": {**GOAL, "attempts": 2}}, 3) == "last attempt"
    assert goal_escalation({**state, "last_report": " "}, 3) == "no report yet"
    continued = GoalDecision(previous_goal_status="continue", reasoning="S1 (nimble): still in progress.")
    assert GoalDecision.model_validate_json(continued.model_dump_json(exclude_none=True)).goal is None
