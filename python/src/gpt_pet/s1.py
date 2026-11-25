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

from gpt_pet.agents.executor import is_budget_refusal
from gpt_pet.goals import REST_STATUS, STATE_CURRENT_GOAL, STATE_LAST_REPORT, STATE_PENDING_GOALS, GoalDecision
from gpt_pet.settings import S1Settings, Settings

if TYPE_CHECKING:
    from gpt_pet.stats import RunStats

ENDPOINT = "/v1/systemone"
LOOK = "get_current_view"
DRIVE = "set_nav_goal"
TURN = "do_rotate"
TURN_DEGREES = 90
TURNS = {"turn_left": "RotateLeft", "turn_right": "RotateRight"}
ALREADY_THERE_M = 0.3
"""A drive planned shorter than this means the pet was already at the destination."""
ASK_GEMINI = "ask_gemini"
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


def seen(destination: Mapping[str, Any]) -> str:
    distance = destination.get("distance_m")
    return f"{destination.get('label')} ({distance} m)" if distance is not None else str(destination.get("label"))


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


def action_options(view: Mapping[str, Any] | None, exclude: set[str] = frozenset()) -> dict[str, dict[str, Any]]:
    """Choice key -> option: one per destination in view (keyed by marker) except `exclude`d ids,
    turns, and Gemini. Excluding this stretch's drives keeps S1 from repeating one."""
    options: dict[str, dict[str, Any]] = {}
    for index, destination in enumerate(destinations(view), start=1):
        if destination["id"] in exclude:
            continue
        key = f"go_{destination.get('marker') or index}"
        # ai2thor-mcp descriptions start with the label ("Sofa, 2.0 m ahead-left"); short options
        # keep the prompt small, which is most of nimble's latency.
        options[key] = {"destination": destination, "text": f"Drive to {destination.get('description') or destination.get('label')}"}
    options["turn_left"] = {"text": "Turn left 90 degrees to look for something that serves the goal"}
    options["turn_right"] = {"text": "Turn right 90 degrees to look for something that serves the goal"}
    options[ASK_GEMINI] = {"text": "None of these serves the goal, or deciding needs the camera image"}
    return options


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
        questions["criteria_met"] = {
            "type": "noul",
            "instructions": "Do the destinations in view and what the pet did show that the success criteria are met now?",
            "criteria": {"true": "Clearly met now", "false": "Not met yet, or it cannot be told from this"},
        }
    return questions


def executor_state(goal: Mapping[str, Any], steps: list[Step]) -> dict[str, Any]:
    return {
        "goal": goal.get("goal"),
        "success_criteria": goal.get("success_criteria"),
        "steps": goal.get("sub_goals") or [],
        "done_this_stretch": [describe(step) for step in steps if step.name != LOOK] or ["looked around"],
        "in_view": [d.get("description") or d.get("label") for d in destinations(latest_view(steps))],
    }


def executor_report(steps: list[Step], *, met: bool, p_met: float, model: str) -> str:
    """The executor's four-line report, written by S1."""
    in_view = ", ".join(seen(d) for d in destinations(latest_view(steps))[:6]) or "nothing notable"
    did = "; ".join(describe(step) for step in steps if step.name != LOOK) or "looked around"
    drives = [step for step in steps if step.name == DRIVE and isinstance(step.response, dict)]
    nav = (drives[-1].response.get("drive") or {}).get("outcome", "unknown") if drives else "no drive"
    verdict = "met" if met else "not met yet"
    return f"Saw: {in_view}.\nDid: {did}.\nFinal nav state: {nav}.\nSuccess criteria {verdict} (S1 {model}, p={p_met:.2f})."


def rule_turn(goal: Mapping[str, Any], steps: list[Step], limits: Mapping[str, int]) -> Turn | None:
    """The turns rules settle without any model; None when the turn needs a judgment."""
    if goal.get("status") == REST_STATUS:
        return Turn("report", text=REST_REPLY)
    if not steps:
        return Turn("call", LOOK)
    last = steps[-1]
    if is_budget_refusal(last.response):
        return Turn("escalate", reason="tool budget")
    if last.response is None or last.failed:
        return Turn("escalate", reason="tool error")
    if last.name == LOOK:
        return None
    if last.name in (DRIVE, TURN):
        if at_limit(steps, LOOK, limits):
            return Turn("escalate", reason="no looks left")
        return Turn("call", LOOK)
    return Turn("escalate", reason=f"after {last.name}")


def judge_turn(
    answers: Mapping[str, Answer],
    options: Mapping[str, Mapping[str, Any]],
    steps: list[Step],
    limits: Mapping[str, int],
    min_certainty: float,
    model: str,
) -> Turn:
    """The executor turn nimble's answers call for, or an escalation when they are not good enough."""
    step, met = answers["next"], answers.get("criteria_met")
    p_met = met.probabilities.get("true", 0.0) if met else 0.0
    if met is not None and met.choice == "true" and met.certainty >= min_certainty:
        return Turn("report", text=executor_report(steps, met=True, p_met=p_met, model=model), by="nimble", certainty=met.certainty)
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
        return Turn("report", text=executor_report(steps, met=False, p_met=p_met, model=model), by="nimble", certainty=step.certainty)
    destination_id = options[step.choice]["destination"]["id"]
    return Turn("call", DRIVE, {"destination_id": destination_id}, by="nimble", certainty=step.certainty)


# --- goal setter decisions -----------------------------------------------------------------


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

    def __init__(self, settings: Settings, stats: "RunStats") -> None:
        self.settings = settings.s1
        self.limits = dict(settings.tool_limits)
        self.max_goal_attempts = settings.brain.max_goal_attempts
        self.stats = stats
        self.client = DecisionClient(settings.s1, stats)
        self._invocation: str | None = None
        self._escalated = False

    async def warm_up(self) -> None:
        """Load the model before the first decision; a cold start takes seconds."""
        question = {"type": "noul", "instructions": "Is this a warm-up request?", "criteria": {"true": "yes", "false": "no"}}
        try:
            await self.client.decide("warm-up", {"ready": question}, record=False)
        except S1Error as exc:
            log.warning("S1 is enabled but %s is not answering (%s); Gemini takes every turn until it does", self.settings.model, exc)
        else:
            log.info("S1 ready: %s at %s", self.settings.model, self.client.url)

    async def goal_setter_turn(self, callback_context: Any, llm_request: Any) -> LlmResponse | None:
        state = callback_context.state
        reason = goal_escalation(state, self.max_goal_attempts)
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

    async def executor_turn(self, callback_context: Any, llm_request: Any) -> LlmResponse | None:
        if callback_context.invocation_id != self._invocation:
            self._invocation, self._escalated = callback_context.invocation_id, False
        if self._escalated:
            return None
        goal = callback_context.state.get(STATE_CURRENT_GOAL) or {}
        steps = tool_steps(llm_request.contents)
        turn = rule_turn(goal, steps, self.limits)
        if turn is None:
            options = action_options(latest_view(steps), exclude=driven_to(steps))
            # The criteria can only have changed by moving, or be met already on a goal's later stretch.
            ask_met = used(steps, DRIVE) > 0 or int(goal.get("attempts") or 0) > 0
            try:
                answers = await self.client.decide(executor_state(goal, steps), executor_questions(options, ask_met=ask_met))
            except S1Error as exc:
                log.warning("S1 executor decision failed: %s", exc)
                turn = Turn("escalate", reason="S1 error")
            else:
                turn = judge_turn(answers, options, steps, self.limits, self.settings.min_confidence, self.settings.model)
                step, met = answers["next"], answers.get("criteria_met")
                log.info(
                    "S1 %s: next=%s (certainty %.2f)%s -> %s",
                    self.settings.model,
                    options.get(step.choice, {}).get("text", step.choice),
                    step.certainty,
                    f", met p={met.probabilities['true']:.2f}" if met else "",
                    turn.tool or turn.kind if turn.kind != "escalate" else f"Gemini ({turn.reason})",
                )
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

    def _respond(self, agent: str, part: types.Part, *, by: str, action: str, certainty: float | None) -> LlmResponse:
        self.stats.record_s1_turn(agent, by)
        tag: dict[str, Any] = {"by": by, "action": action}
        if by == "nimble":
            tag["model"] = self.settings.model
        if certainty is not None:
            tag["certainty"] = round(certainty, 2)
        return LlmResponse(content=types.Content(role="model", parts=[part]), custom_metadata={"s1": tag})
