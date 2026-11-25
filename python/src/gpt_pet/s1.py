"""System 1: a local decision model takes the turns it can decide; Gemini takes the rest.

nimble (Bespoke Labs, served by Ollama >= 0.35) answers typed questions about structured state in
one pass, through TypeSafe's System One API (`POST /v1/systemone`). Two `before_model_callback`s
put it in front of Gemini: returning an `LlmResponse` answers that turn without calling Gemini;
returning None hands the turn to Gemini with the full context, camera image included.

- Goal setter: nimble judges whether the current goal is still in progress. Only a confident
  "continue" skips Gemini. New goals, finished goals and anything uncertain go to Gemini, which
  stays the only author of goals.
- Executor: rules take the mechanical turns (look first, look again after moving). nimble picks
  the next action from the destinations the MCP server offers, or decides that the success
  criteria are met, and S1 writes the report. Once Gemini takes a turn it keeps the rest of the
  tick.

With `[s1] rules_only` there is no decision model: the rules still take their turns, and every
turn the model would have judged goes to Gemini (or the local model in its place).

An answer counts only at `[s1] min_confidence` or above. For a choice that is the API's
`confidence` (1 - normalized entropy of the probabilities); for a yes/no answer it is the margin
|p(yes) - p(no)|. nimble's probabilities are a softmax over the offered options, not calibrated
correctness, so tune the threshold with the portal's stats.
"""
from __future__ import annotations

import base64
import logging
import math
import time
from collections.abc import Mapping
from dataclasses import dataclass, field
from typing import TYPE_CHECKING, Any, Literal

import httpx
from google.adk.models.llm_response import LlmResponse
from google.genai import types

from gpt_pet.agents.executor import is_budget_refusal, is_cancel_refusal
from gpt_pet.people import ATTENTION_LIMITS, ATTENTION_TOOL, EXECUTOR, People
from gpt_pet.goals import REST_STATUS, STATE_CURRENT_GOAL, STATE_LAST_REPORT, STATE_PENDING_GOALS, GoalDecision
from gpt_pet.navwait import NULL_DRIVE_M
from gpt_pet.places import MAX_FAILED_DRIVES
from gpt_pet.settings import S1Settings, Settings

if TYPE_CHECKING:
    from gpt_pet.places import Places
    from gpt_pet.stats import RunStats

ENDPOINT = "/v1/systemone"
LOOK = "get_current_view"
DRIVE = "set_nav_goal"
TURN = "do_rotate"
MOVE = "do_move"
TURN_DEGREES = 90
TURNS = {"turn_left": "RotateLeft", "turn_right": "RotateRight"}
ALREADY_THERE_M = NULL_DRIVE_M
"""A drive planned shorter than this means the pet was already at the destination."""
ASK_GEMINI = "ask_gemini"
STUCK_DRIVES = 2
"""`[s1] unstuck_rule`: drives failed this many times in a row without the robot moving means stuck."""
BACK_UP_M = 0.5
RULES_ONLY = "rules only"
"""Why a turn went to Gemini under `[s1] rules_only`: it needed the decision model."""
MAX_DRIVE_OPTIONS = 8
"""With the places memory on, at most this many destinations are offered (best first): nimble's
latency grows with the prompt, and it favours the first options."""
REST_REPLY = "Goal limit reached; resting."
"""What executor.md tells the executor to answer for the rest stand-in of a limited run."""
INJECTED_CALL_SIGNATURE = base64.urlsafe_b64decode("skip_thought_signature_validator")
"""Gemini 3 rejects a history whose function calls lack a thought signature, so when Gemini takes
over a tick after S1's calls, those calls carry the sentinel Google documents for injected calls
(the SDK sends these bytes as exactly that base64 string)."""

GOAL_QUESTIONS: dict[str, Any] = {
    "status": {
        "type": "choice",
        "instructions": "Judging by the executor's latest report, what is the status of the goal?",
        "criteria": {
            "continue": "In progress: the success criteria are not met yet, and more work can get there",
            "done": "The report shows the success criteria are met",
            "abandoned": "The report shows the goal is impossible or keeps failing",
        },
    }
}

log = logging.getLogger("gpt_pet.s1")


class S1Error(RuntimeError):
    """The decision model did not answer: unreachable, timed out, or an unexpected reply."""


# --- answers -------------------------------------------------------------------------------


@dataclass(frozen=True)
class Answer:
    choice: str
    certainty: float
    probabilities: dict[str, float]


def certainty(probabilities: Mapping[str, float]) -> float:
    """1 - normalized entropy: 1 for a sure answer, 0 when every option is equally likely."""
    if len(probabilities) < 2:
        return 1.0
    entropy = -sum(p * math.log(p) for p in probabilities.values() if p > 0)
    return max(0.0, 1.0 - entropy / math.log(len(probabilities)))


def parse_answer(raw: Any) -> Answer:
    """One answer of a `/v1/systemone` reply: a choice, or a yes/no (`noul`) probability."""
    if not isinstance(raw, dict):
        raise S1Error(f"unexpected answer: {raw!r:.200}")
    if raw.get("type") == "noul" or "noul" in raw:
        p_true = raw.get("noul")
        if not isinstance(p_true, (int, float)):
            raise S1Error(f"yes/no answer without a probability: {raw!r:.200}")
        probabilities = {"true": float(p_true), "false": 1.0 - float(p_true)}
        # The margin, not entropy: entropy would demand p of about 0.96 for a 0.75 bar, and
        # nimble judged "0.4 m from the armchair: within 1 m?" at p = 0.94.
        return Answer("true" if p_true >= 0.5 else "false", abs(2 * float(p_true) - 1), probabilities)
    probabilities = {
        str(key): float(value) for key, value in (raw.get("probabilities") or {}).items() if isinstance(value, (int, float))
    }
    choice = raw.get("choice")
    if not isinstance(choice, str):
        if not probabilities:
            raise S1Error(f"choice answer without a choice: {raw!r:.200}")
        choice = max(probabilities, key=probabilities.__getitem__)
    reported = raw.get("confidence")
    sure = float(reported) if isinstance(reported, (int, float)) else certainty(probabilities)
    return Answer(choice, sure, probabilities)


class DecisionClient:
    """Asks typed questions at `/v1/systemone` and parses the answers."""

    def __init__(self, settings: S1Settings, stats: "RunStats | None" = None) -> None:
        self.settings = settings
        self.stats = stats
        self.url = str(settings.base_url).rstrip("/") + ENDPOINT

    async def decide(self, state: Any, questions: dict[str, Any], *, record: bool = True) -> dict[str, Answer]:
        body = {"model": self.settings.model, "state": state, "questions": questions}
        started = time.monotonic()
        try:
            async with httpx.AsyncClient(timeout=self.settings.timeout_s) as client:
                response = await client.post(self.url, json=body)
        except httpx.HTTPError as exc:
            raise S1Error(f"{self.settings.model} at {self.url}: {exc!r}") from exc
        if response.status_code != 200:
            raise S1Error(f"{self.settings.model} answered HTTP {response.status_code}: {response.text[:200]}")
        try:
            answers = response.json()["answers"]
        except (ValueError, KeyError, TypeError) as exc:
            raise S1Error(f"unexpected reply: {response.text[:200]}") from exc
        if not isinstance(answers, dict):
            raise S1Error(f"unexpected answers: {answers!r:.200}")
        parsed = {name: parse_answer(answers.get(name)) for name in questions}
        if record and self.stats is not None:
            self.stats.record_decider_call(time.monotonic() - started)
        return parsed


# --- the executor's tick so far ------------------------------------------------------------


@dataclass
class Step:
    """One executor tool call this tick and its response (None until it answers)."""

    name: str
    args: dict[str, Any]
    response: Any = None

    @property
    def result(self) -> Any:
        return self.response.get("result") if isinstance(self.response, dict) else None

    @property
    def failed(self) -> bool:
        response = self.response
        return isinstance(response, dict) and bool(response.get("isError") or response.get("is_error") or "error" in response)


def tool_steps(contents: list[types.Content]) -> list[Step]:
    """The tool calls in a request's contents, paired with their responses, in order."""
    steps: list[Step] = []
    by_id: dict[str, Step] = {}
    for content in contents:
        for part in content.parts or []:
            if part.function_call is not None:
                step = Step(part.function_call.name or "", dict(part.function_call.args or {}))
                steps.append(step)
                if part.function_call.id:
                    by_id[part.function_call.id] = step
            elif part.function_response is not None:
                answer = part.function_response
                step = by_id.get(answer.id or "") or next(
                    (s for s in steps if s.name == answer.name and s.response is None), None
                )
                if step is not None:
                    step.response = answer.response
    return steps


def used(steps: list[Step], name: str) -> int:
    return sum(1 for step in steps if step.name == name)


def at_limit(steps: list[Step], name: str, limits: Mapping[str, int]) -> bool:
    limit = limits.get(name)
    return limit is not None and used(steps, name) >= limit


def latest_view(steps: list[Step]) -> dict[str, Any] | None:
    for step in reversed(steps):
        if step.name == LOOK and not step.failed and isinstance(step.result, dict):
            return step.result
    return None


def destinations(view: Mapping[str, Any] | None) -> list[dict[str, Any]]:
    return [d for d in (view or {}).get("destinations") or [] if isinstance(d, dict) and d.get("id")]


def seen(destination: Mapping[str, Any], *, marks: bool = False) -> str:
    """How a report names a destination: a passage by its description (which says where it is),
    an object by its label and distance; with the places memory, its mark too."""
    distance = destination.get("distance_m")
    if marks and destination.get("kind") == "passage" and destination.get("description"):
        text = str(destination["description"])
    else:
        text = f"{destination.get('label')} ({distance} m)" if distance is not None else str(destination.get("label"))
    if marks:
        text += "".join(f" [{mark}]" for mark in ("here", "visited") if destination.get(mark))
    return text


def sightings(view: Mapping[str, Any] | None, *, marks: bool = False, limit: int = 6) -> list[str]:
    """What a report says is in view: with the places memory, every passage first."""
    found = destinations(view)
    if marks:
        passages = [d for d in found if d.get("kind") == "passage"]
        others = [d for d in found if d.get("kind") != "passage"]
        return [seen(d, marks=True) for d in passages] + [seen(d, marks=True) for d in others[:limit]]
    return [seen(d) for d in found[:limit]]


def _normalized(text: str) -> str:
    return "".join(ch for ch in text.lower() if ch.isalnum())


def goal_words(goal: Mapping[str, Any]) -> str:
    """The goal, its success criteria and steps as one normalized string, for name matching."""
    parts = [str(goal.get("goal") or ""), str(goal.get("success_criteria") or ""), *map(str, goal.get("sub_goals") or [])]
    return _normalized(" ".join(parts))


def named_in_goal(destination: Mapping[str, Any], words: str) -> bool:
    """The goal names this destination ("dining table" names a DiningTable)."""
    label = _normalized(str(destination.get("label") or ""))
    return bool(label) and label in words


def fresh(destination: Mapping[str, Any]) -> bool:
    """Worth a drive by the pet's marks: not within reach already, not failed too often, and not
    failed from this very spot (`failed_here`, with `[brain] failed_guard`)."""
    return not destination.get("here") and not destination.get("failed_here") and int(destination.get("failed") or 0) < MAX_FAILED_DRIVES


def ranked(found: list[dict[str, Any]], words: str) -> list[dict[str, Any]]:
    """Destinations the goal names first, then unvisited passages, unvisited objects, and the
    visited last; the view's order (nearest first) within each group."""

    def key(item: tuple[int, dict[str, Any]]) -> tuple[int, int, int, int]:
        index, destination = item
        return (
            0 if named_in_goal(destination, words) else 1,
            1 if destination.get("visited") else 0,
            0 if destination.get("kind") == "passage" else 1,
            index,
        )

    return [destination for _, destination in sorted(enumerate(found), key=key)]


def describe(step: Step) -> str:
    if step.name == DRIVE:
        result = step.result if isinstance(step.result, dict) else {}
        label = result.get("label") or step.args.get("destination_id")
        drive = step.response.get("drive") if isinstance(step.response, dict) else None
        if not isinstance(drive, dict):
            return f"drove to {label}"
        planned = (result.get("path") or {}).get("length_m")
        if drive.get("outcome") == "succeeded" and isinstance(planned, (int, float)) and planned < ALREADY_THERE_M:
            return f"was already at {label}"
        return f"drove to {label} ({drive.get('outcome')} after {drive.get('waited_s')} s)"
    if step.name == TURN:
        direction = "left" if step.args.get("action") == "RotateLeft" else "right"
        return f"turned {direction} {step.args.get('degrees', TURN_DEGREES)} degrees"
    if step.name == LOOK:
        return "looked"
    if step.name == ATTENTION_TOOL:
        result = step.result if isinstance(step.result, dict) else {}
        return f"turned to {step.args.get('destination_id')} (attention {result.get('state', 'unknown')})"
    return step.name


# --- executor decisions --------------------------------------------------------------------


@dataclass(frozen=True)
class Turn:
    """What S1 does with one executor turn."""

    kind: Literal["call", "report", "escalate"]
    tool: str | None = None
    args: dict[str, Any] = field(default_factory=dict)
    text: str | None = None
    reason: str | None = None
    """Why Gemini takes the turn (escalations only)."""
    by: Literal["rules", "nimble"] = "rules"
    certainty: float | None = None


def driven_to(steps: list[Step]) -> set[str]:
    return {str(step.args.get("destination_id")) for step in steps if step.name == DRIVE}


def action_options(
    view: Mapping[str, Any] | None,
    exclude: set[str] = frozenset(),
    *,
    goal: Mapping[str, Any] | None = None,
    marks: bool = False,
) -> dict[str, dict[str, Any]]:
    """Choice key -> option: one per destination in view (keyed by marker) except `exclude`d ids,
    turns, and Gemini. Excluding this stretch's drives keeps S1 from repeating one. With the
    places memory (`marks`), destinations already here or failed too often are left out, and the
    rest are ranked for the goal (`ranked`) and capped at MAX_DRIVE_OPTIONS."""
    found = [d for d in destinations(view) if d["id"] not in exclude]
    if marks:
        found = ranked([d for d in found if fresh(d)], goal_words(goal or {}))[:MAX_DRIVE_OPTIONS]
    options: dict[str, dict[str, Any]] = {}
    for index, destination in enumerate(found, start=1):
        key = f"go_{destination.get('marker') or index}"
        # ai2thor-mcp descriptions start with the label ("Sofa, 2.0 m ahead-left"); short options
        # keep the prompt small, which is most of nimble's latency.
        text = f"Drive to {destination.get('description') or destination.get('label')}"
        if marks and destination.get("visited"):
            text += " (visited before)"
        options[key] = {"destination": destination, "text": text}
    options["turn_left"] = {"text": "Turn left 90 degrees to look for something that serves the goal"}
    options["turn_right"] = {"text": "Turn right 90 degrees to look for something that serves the goal"}
    options[ASK_GEMINI] = {"text": "None of these serves the goal, or deciding needs the camera image"}
    return options


CRITERIA_MET_QUESTION: dict[str, Any] = {
    "type": "noul",
    "instructions": "Do the destinations in view and what the pet did show that the success criteria are met now?",
    "criteria": {"true": "Clearly met now", "false": "Not met yet, or it cannot be told from this"},
}


def executor_questions(options: Mapping[str, Mapping[str, Any]], *, ask_met: bool) -> dict[str, Any]:
    """The next-action choice, plus whether the success criteria are met once the pet has driven."""
    questions: dict[str, Any] = {
        "next": {
            "type": "choice",
            "instructions": (
                "Which action best serves the goal next? When the goal names something in view, drive to it. "
                "Follow the steps in order when there are any."
            ),
            "criteria": {key: option["text"] for key, option in options.items()},
        }
    }
    if ask_met:
        questions["criteria_met"] = CRITERIA_MET_QUESTION
    return questions


def executor_state(goal: Mapping[str, Any], steps: list[Step], *, marks: bool = False) -> dict[str, Any]:
    in_view = destinations(latest_view(steps))
    return {
        "goal": goal.get("goal"),
        "success_criteria": goal.get("success_criteria"),
        "steps": goal.get("sub_goals") or [],
        "done_this_stretch": [describe(step) for step in steps if step.name != LOOK] or ["looked around"],
        "in_view": [seen(d, marks=True) if marks else d.get("description") or d.get("label") for d in in_view],
    }


NO_LOOKS_LEFT = "no looks left this stretch"
NOTHING_NEW = "nothing new in view and no turns left this stretch"


def executor_report(
    steps: list[Step],
    *,
    met: bool | None,
    p_met: float = 0.0,
    model: str = "",
    marks: bool = False,
    unjudged: str = NO_LOOKS_LEFT,
) -> str:
    """The executor's four-line report, written by S1. `met=None`: S1 ended the stretch by rule
    without judging the criteria, for the reason `unjudged`."""
    in_view = ", ".join(sightings(latest_view(steps), marks=marks)) or "nothing notable"
    did = "; ".join(describe(step) for step in steps if step.name != LOOK) or "looked around"
    drives = [step for step in steps if step.name == DRIVE and isinstance(step.response, dict)]
    nav = (drives[-1].response.get("drive") or {}).get("outcome", "unknown") if drives else "no drive"
    if met is None:
        verdict = f"Success criteria not judged: {unjudged} (S1 rules)."
    else:
        verdict = f"Success criteria {'met' if met else 'not met yet'} (S1 {model}, p={p_met:.2f})."
    return f"Saw: {in_view}.\nDid: {did}.\nFinal nav state: {nav}.\n{verdict}"


def rule_turn(
    goal: Mapping[str, Any], steps: list[Step], limits: Mapping[str, int], *, scan: bool = False, marks: bool = False
) -> Turn | None:
    """The turns rules settle without any model; None when the turn needs a judgment. With the
    scan rule, a stretch out of looks ends with S1's own report instead of a turn for Gemini."""
    if goal.get("status") == REST_STATUS:
        return Turn("report", text=REST_REPLY)
    if not steps:
        return Turn("call", LOOK)
    last = steps[-1]
    if is_cancel_refusal(last.response):  # the owner replaced the goal: report what happened, no model
        return Turn("report", text=executor_report(steps, met=None, unjudged="the owner canceled the goal", marks=marks))
    if is_budget_refusal(last.response):
        return Turn("escalate", reason="tool budget")
    if last.response is None or last.failed:
        return Turn("escalate", reason="tool error")
    if last.name == LOOK:
        return None
    if last.name in (DRIVE, TURN, MOVE):
        if at_limit(steps, LOOK, limits):
            if scan:
                return Turn("report", text=executor_report(steps, met=None, marks=marks))
            return Turn("escalate", reason="no looks left")
        return Turn("call", LOOK)
    return Turn("escalate", reason=f"after {last.name}")


def person_report(steps: list[Step], people: People, person_id: str, *, marks: bool = False) -> str:
    """The stretch ends because a person came into view: what was seen and done, and who it is,
    so the goal setter can decide whether to engage them."""
    sighting = people.last.get(person_id)
    attention = people.attention or {}
    report = executor_report(steps, met=None, unjudged="a person came into view", marks=marks)
    who = sighting.words() if sighting is not None else person_id
    state = attention.get("state") if attention.get("target") == person_id else None
    return report + f"\nPerson: {who}" + (f"; the robot turned to them (attention {state})." if state else ".")


def people_turn(goal: Mapping[str, Any], steps: list[Step], people: People, limits: Mapping[str, int], *, marks: bool = False) -> Turn | None:
    """`[people]` rules, before every other rule. On a goal about people: attend to the person in
    view, look, and let the model say what they are doing. Otherwise a person coming into view
    (not reacted to lately) ends the stretch: turn to them, then report, so the goal setter can
    decide. None: nothing about people this turn."""
    if not steps:
        return None
    last = steps[-1]
    social = people.is_social(goal)
    if last.name == ATTENTION_TOOL:
        if last.response is None or last.failed:
            return None  # refused or failed: the generic rules handle it
        person_id = str(last.args.get("destination_id") or "")
        if social:
            return Turn("call", LOOK) if not at_limit(steps, LOOK, limits) else Turn("escalate", reason="a person to describe")
        engagement = last.response.get("engagement") if isinstance(last.response, dict) else None
        if isinstance(engagement, dict):
            words = ", ".join(engagement.get("evidence") or []) or "showed no sign of wanting to interact"
            if engagement.get("decision") == "interested":
                return Turn("report", text=person_report(steps, people, person_id, marks=marks) + f"\nThey want to interact: {words}.")
            # not interested: they are left be (attention has ended); the goal goes on
            return Turn("call", LOOK) if not at_limit(steps, LOOK, limits) else Turn(
                "report", text=executor_report(steps, met=None, unjudged="a person who is not interested held the stretch up", marks=marks)
            )
        if people.interest_known(person_id) and people.leaving_be(person_id):
            return Turn("call", LOOK) if not at_limit(steps, LOOK, limits) else None
        return Turn("report", text=person_report(steps, people, person_id, marks=marks))
    if last.name != LOOK or last.failed or not isinstance(last.result, dict):
        return None
    in_view = people.in_view(last.result)
    if not in_view:
        if social and not used(steps, ATTENTION_TOOL) and not at_limit(steps, ATTENTION_TOOL, limits):
            # the person is out of view: attention turns toward where they were last seen
            target = str(goal.get("target_id") or "")
            known = target if target in people.last else next((s.id for s in people.recent(within_s=people.react_every_s * 5)), None)
            if known:
                return Turn("call", ATTENTION_TOOL, {"destination_id": known})
        return None
    if social:
        target = str(goal.get("target_id") or "")
        person = next((d for d in in_view if d["id"] == target), None) or min(in_view, key=lambda d: float(d.get("distance_m") or 99.0))
        if not people.attending(person["id"]) and not at_limit(steps, ATTENTION_TOOL, limits):
            return Turn("call", ATTENTION_TOOL, {"destination_id": person["id"]})
        return Turn("escalate", reason="a person to describe")
    sighting = people.to_react(EXECUTOR, goal, [people.last[d["id"]] for d in in_view if d["id"] in people.last])
    if sighting is None:
        return None  # reacted to them lately: carry on with the goal
    people.react(EXECUTOR, sighting.id)
    if not people.attending(sighting.id) and not at_limit(steps, ATTENTION_TOOL, limits):
        return Turn("call", ATTENTION_TOOL, {"destination_id": sighting.id})
    return Turn("report", text=person_report(steps, people, sighting.id, marks=marks))


def met_question_due(goal: Mapping[str, Any], steps: list[Step], view: Mapping[str, Any] | None, *, marks: bool = False) -> bool:
    """Whether to ask if the success criteria are met: once the pet has driven this stretch, on
    a goal's later stretch, or (with the places memory) when something the goal names is here."""
    if used(steps, DRIVE) > 0 or int(goal.get("attempts") or 0) > 0:
        return True
    words = goal_words(goal)
    return marks and any(d.get("here") and named_in_goal(d, words) for d in destinations(view))


def policy_turn(
    goal: Mapping[str, Any], steps: list[Step], view: Mapping[str, Any] | None, limits: Mapping[str, int], places: "Places"
) -> Turn | None:
    """Gemini's direction carried out by rule (`[s1] explore_policy`): drive to the goal's
    `target_id` first, even out of view; on an explore goal, drive to the first unvisited passage
    in view, else the farthest unvisited object. None: nothing for the policy to do."""
    if at_limit(steps, DRIVE, limits):
        return None
    driven = driven_to(steps)
    in_view = {d["id"]: d for d in destinations(view)}
    target = str(goal.get("target_id") or "")
    remembered = places.destinations.get(target)
    if target and target not in driven and not (remembered and remembered.reached_tick is not None):
        if not in_view.get(target, {}).get("here") and not (places.failed_guard and places.failed_here(target)):
            return Turn("call", DRIVE, {"destination_id": target})
    if goal.get("kind") != "explore":
        return None
    candidates = [d for d in in_view.values() if d["id"] not in driven and fresh(d) and not d.get("visited")]
    passages = [d for d in candidates if d.get("kind") == "passage"]
    if passages:
        return Turn("call", DRIVE, {"destination_id": passages[0]["id"]})
    objects = sorted(candidates, key=lambda d: -float(d.get("distance_m") or 0.0))
    return Turn("call", DRIVE, {"destination_id": objects[0]["id"]}) if objects else None


def unstuck_turn(steps: list[Step], limits: Mapping[str, int]) -> Turn | None:
    """`[s1] unstuck_rule`: drives keep failing without the robot moving, so back up, then turn
    around, before driving again. None once both are done this stretch (or out of budget)."""
    if not any(step.name == MOVE for step in steps) and not at_limit(steps, MOVE, limits):
        return Turn("call", MOVE, {"action": "MoveBack", "moveMagnitude": BACK_UP_M})
    if not any(step.name == TURN and float(step.args.get("degrees") or 0) >= 180 for step in steps) and not at_limit(steps, TURN, limits):
        return Turn("call", TURN, {"action": TURNS["turn_right"], "degrees": 180})
    return None


def scan_turn(steps: list[Step], limits: Mapping[str, int], *, marks: bool = False) -> Turn:
    """Nothing in view is worth a drive: turn to see more, or report once no turns are left."""
    if at_limit(steps, TURN, limits):
        return Turn("report", text=executor_report(steps, met=None, marks=marks, unjudged=NOTHING_NEW))
    return Turn("call", TURN, {"action": TURNS["turn_right"], "degrees": TURN_DEGREES})


def judge_turn(
    answers: Mapping[str, Answer],
    options: Mapping[str, Mapping[str, Any]],
    steps: list[Step],
    limits: Mapping[str, int],
    min_certainty: float,
    model: str,
    *,
    marks: bool = False,
) -> Turn:
    """The executor turn nimble's answers call for, or an escalation when they are not good enough."""
    step, met = answers["next"], answers.get("criteria_met")
    p_met = met.probabilities.get("true", 0.0) if met else 0.0
    if met is not None and met.choice == "true" and met.certainty >= min_certainty:
        report = executor_report(steps, met=True, p_met=p_met, model=model, marks=marks)
        return Turn("report", text=report, by="nimble", certainty=met.certainty)
    if step.certainty < min_certainty:
        return Turn("escalate", reason="unsure", by="nimble", certainty=step.certainty)
    if step.choice == ASK_GEMINI or step.choice not in options:
        return Turn("escalate", reason="asked for Gemini", by="nimble", certainty=step.certainty)
    if step.choice in TURNS:
        if at_limit(steps, TURN, limits):
            return Turn("escalate", reason="no turns left", by="nimble", certainty=step.certainty)
        args = {"action": TURNS[step.choice], "degrees": TURN_DEGREES}
        return Turn("call", TURN, args, by="nimble", certainty=step.certainty)
    if at_limit(steps, DRIVE, limits):
        report = executor_report(steps, met=False, p_met=p_met, model=model, marks=marks)
        return Turn("report", text=report, by="nimble", certainty=step.certainty)
    destination_id = options[step.choice]["destination"]["id"]
    return Turn("call", DRIVE, {"destination_id": destination_id}, by="nimble", certainty=step.certainty)


# --- goal setter decisions -----------------------------------------------------------------


def owner_goal_decision(state: Mapping[str, Any]) -> GoalDecision | None:
    """With no goal in progress and the owner's goals queued, the next goal is the first queued one
    (the goal memory adopts it deterministically), so no model needs to decide; None otherwise."""
    pending = state.get(STATE_PENDING_GOALS) or []
    goal = state.get(STATE_CURRENT_GOAL)
    if not pending or (isinstance(goal, dict) and goal.get("status") == "active"):
        return None
    first = pending[0] if isinstance(pending[0], Mapping) else {}
    text = str(first.get("goal") or "").strip() or "the owner's goal"
    return GoalDecision(
        previous_goal_status="none", goal=text, reasoning="S1 rules: no goal in progress, so the owner's queued goal starts."
    )


def goal_escalation(state: Mapping[str, Any], max_goal_attempts: int) -> str | None:
    """Why Gemini must make this goal decision; None when S1 may judge it."""
    goal = state.get(STATE_CURRENT_GOAL)
    if not isinstance(goal, dict) or goal.get("status") != "active":
        return "no active goal"
    if state.get(STATE_PENDING_GOALS):
        return "owner goals queued"
    if int(goal.get("attempts") or 0) + 1 >= max_goal_attempts:
        return "last attempt"  # a continue would be abandoned and replaced by the default goal
    if not str(state.get(STATE_LAST_REPORT) or "").strip():
        return "no report yet"
    return None


def goal_state(state: Mapping[str, Any]) -> dict[str, Any]:
    goal = state[STATE_CURRENT_GOAL]
    return {
        "goal": goal.get("goal"),
        "success_criteria": goal.get("success_criteria"),
        "steps": goal.get("sub_goals") or [],
        "latest_report": state.get(STATE_LAST_REPORT),
    }


# --- the callbacks -------------------------------------------------------------------------


class SystemOne:
    """The decision client and the two `before_model_callback`s that put it in front of Gemini."""

    def __init__(self, settings: Settings, stats: "RunStats", places: "Places | None" = None, people: People | None = None) -> None:
        self.settings = settings.s1
        self.people = people
        """`[people]`: the people rules (`people_turn`) and a goal decision whenever someone new is near."""
        self.limits = {**ATTENTION_LIMITS, **settings.tool_limits} if people is not None else dict(settings.tool_limits)
        self.max_goal_attempts = settings.brain.max_goal_attempts
        self.marks = settings.brain.places_memory
        """Views carry the places memory's marks: skip and rank destinations by them."""
        self.scan = settings.s1.scan_rule
        self.places = places
        self.explore = settings.s1.explore_policy and places is not None
        self.unstuck = settings.s1.unstuck_rule and places is not None
        self.explore_ticks = settings.s1.explore_ticks if places is not None else 0
        self.stats = stats
        self.client = None if settings.s1.rules_only else DecisionClient(settings.s1, stats)
        """None with `[s1] rules_only`: the turns it would decide go to Gemini."""
        self._invocation: str | None = None
        self._escalated = False

    async def warm_up(self) -> None:
        """Load the model before the first decision; a cold start takes seconds."""
        if self.client is None:
            return
        question = {"type": "noul", "instructions": "Is this a warm-up request?", "criteria": {"true": "yes", "false": "no"}}
        try:
            await self.client.decide("warm-up", {"ready": question}, record=False)
        except S1Error as exc:
            log.warning("S1 is enabled but %s is not answering (%s); Gemini takes every turn until it does", self.settings.model, exc)
        else:
            log.info("S1 ready: %s at %s", self.settings.model, self.client.url)

    async def goal_setter_turn(self, callback_context: Any, llm_request: Any) -> LlmResponse | None:
        state = callback_context.state
        owner = owner_goal_decision(state)
        if owner is not None:
            part = types.Part(text=owner.model_dump_json(exclude_none=True))
            return self._respond("goal_setter", part, by="rules", action="owner goal", certainty=None)
        news = self.people.news_for_setter(state.get(STATE_CURRENT_GOAL)) if self.people is not None else None
        nearby = "person wants to interact" if news is not None and self.people.social is not None else "person nearby"
        reason = goal_escalation(state, self.max_goal_attempts) or (nearby if news is not None else None)
        if reason is None and self._keeps_exploring(state.get(STATE_CURRENT_GOAL) or {}):
            decision = GoalDecision(previous_goal_status="continue", reasoning="S1 rules: still exploring; unvisited places remain.")
            return self._respond("goal_setter", types.Part(text=decision.model_dump_json(exclude_none=True)), by="rules", action="continue", certainty=None)
        reason = reason or (RULES_ONLY if self.client is None else None)
        if reason is None:
            try:
                answers = await self.client.decide(goal_state(state), GOAL_QUESTIONS)
            except S1Error as exc:
                log.warning("S1 goal check failed: %s", exc)
                reason = "S1 error"
            else:
                status = answers["status"]
                if status.choice != "continue":
                    reason = f"judged {status.choice}"
                elif status.certainty < self.settings.min_confidence:
                    reason = "unsure"
                else:
                    decision = GoalDecision(
                        previous_goal_status="continue",
                        reasoning=f"S1 ({self.settings.model}): still in progress (certainty {status.certainty:.2f}).",
                    )
                    part = types.Part(text=decision.model_dump_json(exclude_none=True))
                    return self._respond("goal_setter", part, by="nimble", action="continue", certainty=status.certainty)
        self.stats.record_s1_escalation("goal_setter", reason)
        return None

    def _keeps_exploring(self, goal: Mapping[str, Any]) -> bool:
        """`[s1] explore_ticks`: an explore goal goes on by rule while it has had fewer stretches,
        unvisited places remain, and the robot is not stuck."""
        return (
            self.explore_ticks > 0
            and goal.get("kind") == "explore"
            and int(goal.get("attempts") or 0) < self.explore_ticks
            and self.places is not None
            and self.places.stuck_drives == 0
            and bool(self.places.unvisited())
        )

    async def executor_turn(self, callback_context: Any, llm_request: Any) -> LlmResponse | None:
        if callback_context.invocation_id != self._invocation:
            self._invocation, self._escalated = callback_context.invocation_id, False
        if self._escalated:
            return None
        goal = callback_context.state.get(STATE_CURRENT_GOAL) or {}
        steps = tool_steps(llm_request.contents)
        turn = people_turn(goal, steps, self.people, self.limits, marks=self.marks) if self.people is not None else None
        if turn is None:
            turn = rule_turn(goal, steps, self.limits, scan=self.scan, marks=self.marks)
        if turn is None and self.unstuck and self.places.stuck_drives >= STUCK_DRIVES:
            turn = unstuck_turn(steps, self.limits)
            if turn is not None:
                log.info("S1: %d drives failed without moving; %s", self.places.stuck_drives, turn.tool)
        if turn is None:
            view = latest_view(steps)
            if self.people is not None:
                view = self.people.without_people(view)  # people are not places to drive to
            options = action_options(view, exclude=driven_to(steps), goal=goal, marks=self.marks)
            ask_met = met_question_due(goal, steps, view, marks=self.marks)
            planned = policy_turn(goal, steps, view, self.limits, self.places) if self.explore and self.places else None
            if planned is not None:
                turn = await self._unless_met(planned, goal, steps) if ask_met else planned
            elif self.scan and not ask_met and not any(option.get("destination") for option in options.values()):
                turn = scan_turn(steps, self.limits, marks=self.marks)
            else:
                turn = await self._decide(goal, steps, options, ask_met=ask_met)
        if turn.kind == "escalate":
            self._escalated = True
            self.stats.record_s1_escalation("executor", turn.reason or "unknown")
            log.info("S1 hands the rest of the tick to Gemini: %s", turn.reason)
            return None
        if turn.kind == "report":
            return self._respond("executor", types.Part(text=turn.text), by=turn.by, action="report", certainty=turn.certainty)
        call = types.Part(
            function_call=types.FunctionCall(name=turn.tool, args=turn.args), thought_signature=INJECTED_CALL_SIGNATURE
        )
        return self._respond("executor", call, by=turn.by, action=turn.tool or "", certainty=turn.certainty)

    async def _unless_met(self, planned: Turn, goal: Mapping[str, Any], steps: list[Step]) -> Turn:
        """The policy's drive, unless the decision model is sure the criteria are met already."""
        if self.client is None:
            return planned
        try:
            answers = await self.client.decide(executor_state(goal, steps, marks=self.marks), {"criteria_met": CRITERIA_MET_QUESTION})
        except S1Error as exc:
            log.warning("S1 criteria check failed: %s", exc)
            return planned
        met = answers["criteria_met"]
        if met.choice == "true" and met.certainty >= self.settings.min_confidence:
            report = executor_report(steps, met=True, p_met=met.probabilities.get("true", 0.0), model=self.settings.model, marks=self.marks)
            return Turn("report", text=report, by="nimble", certainty=met.certainty)
        return planned

    async def _decide(
        self, goal: Mapping[str, Any], steps: list[Step], options: Mapping[str, Mapping[str, Any]], *, ask_met: bool
    ) -> Turn:
        """Ask the decision model for the next action (and, when due, whether the criteria are met)."""
        if self.client is None:
            return Turn("escalate", reason=RULES_ONLY)
        try:
            state = executor_state(goal, steps, marks=self.marks)
            answers = await self.client.decide(state, executor_questions(options, ask_met=ask_met))
        except S1Error as exc:
            log.warning("S1 executor decision failed: %s", exc)
            return Turn("escalate", reason="S1 error")
        turn = judge_turn(answers, options, steps, self.limits, self.settings.min_confidence, self.settings.model, marks=self.marks)
        step, met = answers["next"], answers.get("criteria_met")
        log.info(
            "S1 %s: next=%s (certainty %.2f)%s -> %s",
            self.settings.model,
            options.get(step.choice, {}).get("text", step.choice),
            step.certainty,
            f", met p={met.probabilities['true']:.2f}" if met else "",
            turn.tool or turn.kind if turn.kind != "escalate" else f"Gemini ({turn.reason})",
        )
        return turn

    def _respond(self, agent: str, part: types.Part, *, by: str, action: str, certainty: float | None) -> LlmResponse:
        self.stats.record_s1_turn(agent, by)
        tag: dict[str, Any] = {"by": by, "action": action}
        if by == "nimble":
            tag["model"] = self.settings.model
        if certainty is not None:
            tag["certainty"] = round(certainty, 2)
        return LlmResponse(content=types.Content(role="model", parts=[part]), custom_metadata={"s1": tag})
