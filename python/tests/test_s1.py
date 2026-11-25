"""Offline: System 1's decisions as pure functions over real genai objects and nimble's real reply
shapes. The model itself is exercised live (tests/live/test_s1_live.py)."""
from __future__ import annotations

import pytest
from google.genai import types

from gpt_pet.agents.executor import BUDGET_MARKER
from gpt_pet.goals import GoalDecision
from gpt_pet.places import Place, Places
from gpt_pet.s1 import (
    ASK_GEMINI,
    INJECTED_CALL_SIGNATURE,
    MAX_DRIVE_OPTIONS,
    NOTHING_NEW,
    REST_REPLY,
    RULES_ONLY,
    Answer,
    S1Error,
    Step,
    SystemOne,
    Turn,
    action_options,
    certainty,
    executor_questions,
    executor_report,
    executor_state,
    goal_escalation,
    judge_turn,
    met_question_due,
    parse_answer,
    policy_turn,
    rule_turn,
    scan_turn,
    tool_steps,
)
from gpt_pet.settings import load_settings
from gpt_pet.stats import RunStats

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


# --- with the places memory's marks (`[brain] places_memory`) and the scan rule (`[s1] scan_rule`)

FRIDGE_GOAL = {**GOAL, "goal": "find the fridge", "success_criteria": "The fridge is within 1 m"}
MARKED = {
    "agent": {"x": 1.0, "z": 2.0, "yaw": 90.0},
    "destinations": [
        {"marker": 1, "id": "obj:DiningTable|1", "kind": "object", "label": "DiningTable", "description": "DiningTable, 0.9 m ahead", "distance_m": 0.9, "here": True},
        {"marker": 2, "id": "obj:Sofa|2", "kind": "object", "label": "Sofa", "description": "Sofa, 2.0 m ahead", "distance_m": 2.0, "visited": True},
        {"marker": 3, "id": "psg:4", "kind": "passage", "label": "opening", "description": "passage: opening ahead, ~2.0 m wide", "distance_m": 3.1},
        {"marker": 4, "id": "obj:Painting|1", "kind": "object", "label": "Painting", "description": "Painting, 1.9 m ahead-left", "distance_m": 1.9},
        {"marker": 5, "id": "obj:Television|3", "kind": "object", "label": "Television", "description": "Television, 2.4 m right", "distance_m": 2.4, "failed": 2},
        {"marker": 6, "id": "obj:Fridge|4", "kind": "object", "label": "Fridge", "description": "Fridge, 3.5 m ahead-right", "distance_m": 3.5},
    ],
}


def marked_look(view: dict = MARKED) -> Step:
    return Step("get_current_view", {}, {"result": view, "isError": False})


def test_marks_drop_what_is_here_or_keeps_failing_and_rank_the_rest_for_the_goal() -> None:
    options = action_options(MARKED, goal=FRIDGE_GOAL, marks=True)
    drives = [key for key in options if key.startswith("go_")]
    assert drives == ["go_6", "go_3", "go_4", "go_2"]  # named by the goal, passage, unvisited, visited
    assert options["go_2"]["text"] == "Drive to Sofa, 2.0 m ahead (visited before)"
    assert list(options)[len(drives):] == ["turn_left", "turn_right", ASK_GEMINI]
    assert [key for key in action_options(MARKED) if key.startswith("go_")] == [f"go_{n}" for n in range(1, 7)]  # no marks: as before
    crowded = {"destinations": [{"marker": n, "id": f"obj:Box|{n}", "kind": "object", "label": "Box", "distance_m": 2.0 + n} for n in range(1, 13)]}
    assert sum(key.startswith("go_") for key in action_options(crowded, goal=FRIDGE_GOAL, marks=True)) == MAX_DRIVE_OPTIONS


def test_the_criteria_question_is_due_when_what_the_goal_names_is_here() -> None:
    table_goal = {**GOAL, "goal": "get a close look at the dining table", "success_criteria": "The dining table is within 1 m"}
    assert met_question_due(table_goal, [marked_look()], MARKED, marks=True)
    assert not met_question_due(table_goal, [marked_look()], MARKED, marks=False)  # no marks: only after a drive
    assert not met_question_due(FRIDGE_GOAL, [marked_look()], MARKED, marks=True)  # the fridge is not here yet
    assert met_question_due(FRIDGE_GOAL, [marked_look(), drive(), marked_look()], MARKED, marks=True)


def test_with_nothing_worth_a_drive_the_scan_rule_turns_then_reports() -> None:
    first = scan_turn([marked_look()], LIMITS, marks=True)
    assert (first.kind, first.tool, first.args, first.by) == ("call", "do_rotate", {"action": "RotateRight", "degrees": 90}, "rules")
    turned_out = [marked_look(), turn(), marked_look(), turn(), marked_look()]
    done = scan_turn(turned_out, LIMITS, marks=True)
    assert done.kind == "report" and done.text.endswith(f"Success criteria not judged: {NOTHING_NEW} (S1 rules).")


def test_with_the_scan_rule_s1_reports_itself_when_the_looks_run_out() -> None:
    out_of_looks = [look(), turn(), look(), turn(), look(), drive(), look(), drive()]
    report = rule_turn(GOAL, out_of_looks, LIMITS, scan=True)
    assert (report.kind, report.by) == ("report", "rules")
    assert report.text.endswith("Success criteria not judged: no looks left this stretch (S1 rules).")
    assert rule_turn(GOAL, out_of_looks, LIMITS).reason == "no looks left"  # without it, Gemini writes the report


def test_reports_with_marks_name_passages_first_with_their_direction() -> None:
    report = executor_report([marked_look()], met=False, p_met=0.1, model="nimble", marks=True)
    assert report.splitlines()[0] == (
        "Saw: passage: opening ahead, ~2.0 m wide, DiningTable (0.9 m) [here], Sofa (2.0 m) [visited], "
        "Painting (1.9 m), Television (2.4 m), Fridge (3.5 m)."
    )
    assert executor_state(FRIDGE_GOAL, [marked_look()], marks=True)["in_view"][0] == "DiningTable (0.9 m) [here]"


# --- carrying out Gemini's direction by rule (`[s1] explore_policy`)


def test_a_goal_s_target_is_driven_to_first_even_out_of_view() -> None:
    places = Places()
    places.destinations["psg:9"] = Place(id="psg:9", kind="passage", label="doorway")
    goal = {**FRIDGE_GOAL, "kind": "visit", "target_id": "psg:9"}
    first = policy_turn(goal, [marked_look()], MARKED, LIMITS, places)
    assert (first.kind, first.tool, first.args, first.by) == ("call", "set_nav_goal", {"destination_id": "psg:9"}, "rules")
    tried = [marked_look(), Step("set_nav_goal", {"destination_id": "psg:9"}, drive().response), marked_look()]
    assert policy_turn(goal, tried, MARKED, LIMITS, places) is None  # once per stretch; then nimble decides
    places.destinations["psg:9"].reached_tick = 3
    assert policy_turn(goal, [marked_look()], MARKED, LIMITS, places) is None  # reached on an earlier stretch
    here = {"destinations": [*MARKED["destinations"], {"marker": 7, "id": "psg:10", "kind": "passage", "label": "opening", "distance_m": 0.4, "here": True}]}
    places.destinations["psg:10"] = Place(id="psg:10", kind="passage", label="opening")
    assert policy_turn({**goal, "target_id": "psg:10"}, [marked_look(here)], here, LIMITS, places) is None


def test_an_explore_goal_takes_unvisited_passages_first_then_the_farthest_object() -> None:
    explore = {**GOAL, "goal": "explore the flat", "kind": "explore"}
    first = policy_turn(explore, [marked_look()], MARKED, LIMITS, Places())
    assert first.args == {"destination_id": "psg:4"}
    no_passage = {"destinations": [d for d in MARKED["destinations"] if d["kind"] != "passage"]}
    farthest = policy_turn(explore, [marked_look(no_passage)], no_passage, LIMITS, Places())
    assert farthest.args == {"destination_id": "obj:Fridge|4"}  # 3.5 m; the sofa is visited, the TV failed twice, the table here
    assert policy_turn({**explore, "kind": "find"}, [marked_look()], MARKED, LIMITS, Places()) is None  # not explore: nimble
    two_drives = [marked_look(), drive(), marked_look(), drive(), marked_look()]
    assert policy_turn(explore, two_drives, MARKED, LIMITS, Places()) is None  # out of drives


async def test_rules_only_s1_has_no_decision_model_and_hands_its_judgments_on() -> None:
    settings = load_settings("sim")
    settings = settings.model_copy(update={"s1": settings.s1.model_copy(update={"enabled": True, "rules_only": True})})
    stats = RunStats()
    s1 = SystemOne(settings, stats)
    assert s1.client is None
    await s1.warm_up()  # nothing to load
    turn = await s1._decide(GOAL, [], {}, ask_met=True)
    assert (turn.kind, turn.reason) == ("escalate", RULES_ONLY)
    planned = Turn("call", tool="set_nav_goal", args={"destination_id": "psg:1"})
    assert await s1._unless_met(planned, GOAL, []) is planned  # no model to say "met" first
    assert stats.decider.count == 0



def test_a_goal_the_owner_canceled_ends_the_stretch_with_s1_s_own_report() -> None:
    from gpt_pet.agents.executor import cancel_refusal
    from gpt_pet.s1 import rule_turn

    drove = Step("set_nav_goal", {"destination_id": "psg:1"}, {"result": {"goal_id": "g1"}, "isError": False, "drive": {"outcome": "canceled"}})
    refused = Step("get_current_view", {}, {"error": cancel_refusal('the owner gave a new goal ("come here")')})
    turn = rule_turn(GOAL, [marked_look(), drove, refused], LIMITS)
    assert turn.kind == "report" and "Success criteria not judged: the owner canceled the goal (S1 rules)." in turn.text
    assert "Final nav state: canceled." in turn.text


def test_the_owner_s_queued_goal_starts_without_asking_a_model() -> None:
    from gpt_pet.goals import rest_record
    from gpt_pet.s1 import owner_goal_decision

    pending = [{"id": 5, "goal": "find the owner and say hi", "sub_goals": []}]
    decision = owner_goal_decision({"current_goal": None, "pending_goals": pending})
    assert decision.previous_goal_status == "none" and decision.goal == "find the owner and say hi"
    assert owner_goal_decision({"current_goal": rest_record(3), "pending_goals": pending}).goal == "find the owner and say hi"
    assert owner_goal_decision({"current_goal": GOAL, "pending_goals": pending}) is None  # a goal in progress: the model judges it
    assert owner_goal_decision({"current_goal": None, "pending_goals": []}) is None
