"""Runs the brain tick by tick: one Runner, one session, loop controls, explicit cleanup.

Shared by the CLI, the server, and the live tests so all drive the pet exactly the same way.
"""
from __future__ import annotations

import asyncio
import json
import logging
import time
from collections.abc import Callable, Iterable, Mapping
from dataclasses import dataclass
from datetime import datetime, timezone
from typing import Any

from google.adk.agents.invocation_context import LlmCallsLimitExceededError
from google.adk.agents.run_config import RunConfig
from google.adk.apps.app import App
from google.adk.events.event import Event
from google.adk.events.event_actions import EventActions
from google.adk.runners import Runner
from google.adk.sessions import InMemorySessionService
from google.adk.sessions.base_session_service import BaseSessionService
from google.genai import types

from gpt_pet.agents.executor import EXECUTOR_NAME, is_budget_refusal
from gpt_pet.agents.fallbacks import error_name, is_transient_model_error
from gpt_pet.brain import Brain
from gpt_pet.goals import (
    REST_STATUS,
    STATE_CURRENT_GOAL,
    STATE_GOAL_DECISION,
    STATE_GOAL_LIMIT,
    STATE_GOALS_STARTED,
    STATE_LAST_REPORT,
    STATE_LIMIT_REACHED,
    STATE_NEXT_GOAL_ID,
    STATE_PENDING_GOALS,
    STATE_TICK,
    cancel_current_goal,
    initial_state,
    pending_goal_record,
)
from gpt_pet.settings import BrainSettings
from gpt_pet.stats import RunStats, StatsPlugin

APP_NAME = "gpt_pet"
RUNTIME_AUTHOR = "gpt_pet_runtime"
BUDGET_EXHAUSTED_REPORT = (
    "The executor ran out of its LLM call budget before writing a report; treat the goal as not done yet."
)
FAILED_TICK_BACKOFF_S = 2.0
MAX_FAILED_TICK_BACKOFF_S = 30.0

log = logging.getLogger("gpt_pet.runtime")


def executor_failed_report(error: str) -> str:
    return f"The executor's stretch failed ({error}) before it reported; treat the goal as not done yet."


def tick_message(number: int) -> types.Content:
    return types.Content(role="user", parts=[types.Part(text=f"tick {number}")])


def function_calls(events: Iterable[Event]) -> list[types.FunctionCall]:
    return [fc for event in events for fc in event.get_function_calls()]


def function_responses(events: Iterable[Event]) -> list[types.FunctionResponse]:
    return [fr for event in events for fr in event.get_function_responses()]


def is_s1_event(event: Event) -> bool:
    """A model turn System 1 answered instead of Gemini (`gpt_pet.s1` tags them)."""
    return bool((event.custom_metadata or {}).get("s1"))


def is_fallback_event(event: Event) -> bool:
    """A model turn a fallback answered because Gemini failed (`gpt_pet.agents.fallbacks`)."""
    return bool((event.custom_metadata or {}).get("fallback"))


def model_call_count(events: Iterable[Event]) -> int:
    """Gemini calls: model turns that neither System 1 nor a fallback answered."""
    return sum(
        1
        for e in events
        if e.content is not None
        and e.content.role == "model"
        and not e.partial
        and not is_s1_event(e)
        and not is_fallback_event(e)
    )


def s1_turn_count(events: Iterable[Event]) -> int:
    return sum(1 for e in events if is_s1_event(e) and not e.partial)


def response_is_error(response: types.FunctionResponse) -> bool:
    """A real tool failure. Budget refusals are the executor's caps working, not errors."""
    payload = response.response
    if not isinstance(payload, dict) or is_budget_refusal(payload):
        return False
    return bool(payload.get("isError") or payload.get("is_error") or "error" in payload)


def response_is_refusal(response: types.FunctionResponse) -> bool:
    return is_budget_refusal(response.response)


def _jsonable(value: Any) -> Any:
    return json.loads(json.dumps(value, default=str))


def event_payloads(event: Event, tick: int) -> list[dict[str, Any]]:
    """Plain-JSON records of one ADK event, as the portal's Events Log and the experiment
    harness's decision log show it: one per tool call or response, else one for its text. Turns
    System 1 or a fallback took instead of Gemini carry their tag (`s1`, `fallback`); a local
    model's turns carry its timings (`ollama`)."""
    if event.partial or event.content is None:
        return []
    base: dict[str, Any] = {"tick": tick, "author": event.author, "timestamp": event.timestamp, "invocation_id": event.invocation_id}
    metadata = event.custom_metadata or {}
    for tag in ("s1", "fallback", "ollama"):
        if metadata.get(tag):
            base[tag] = metadata[tag]
    payloads: list[dict[str, Any]] = []
    for call in event.get_function_calls():
        payloads.append({**base, "functionCall": {"id": call.id or "", "name": call.name or "", "args": _jsonable(dict(call.args or {}))}})
    for response in event.get_function_responses():
        payloads.append(
            {**base, "functionResponse": {"id": response.id or "", "name": response.name or "", "response": _jsonable(response.response)}}
        )
    text = "".join(part.text for part in (event.content.parts or []) if part.text and not part.thought).strip()
    if text and not payloads:
        payloads.append({**base, "text": text})
    return payloads


def final_texts(events: Iterable[Event]) -> list[tuple[str, str]]:
    """(author, text) for every non-partial model event that carries visible text."""
    out: list[tuple[str, str]] = []
    for event in events:
        if event.partial or event.content is None or event.content.role != "model":
            continue
        text = "".join(p.text for p in (event.content.parts or []) if p.text and not p.thought)
        if text.strip():
            out.append((event.author, text.strip()))
    return out


@dataclass
class TickResult:
    number: int
    seconds: float
    events: list[Event]
    state: dict[str, Any]
    truncated: bool = False
    """True when the tick hit the LLM call budget before the executor reported."""
    error: str | None = None
    """Why the tick failed (a transient error such as a Gemini timeout); None when it ran through."""

    @property
    def failed(self) -> bool:
        return self.error is not None

    @property
    def fallbacks(self) -> list[str]:
        """The agents whose failed Gemini calls a fallback answered this tick."""
        return [e.author for e in self.events if is_fallback_event(e) and not e.partial]

    @property
    def tool_calls(self) -> list[str]:
        return [fc.name or "" for fc in function_calls(self.events)]

    @property
    def tool_errors(self) -> list[str]:
        return [fr.name or "" for fr in function_responses(self.events) if response_is_error(fr)]

    @property
    def refusals(self) -> list[str]:
        return [fr.name or "" for fr in function_responses(self.events) if response_is_refusal(fr)]

    @property
    def goal(self) -> dict[str, Any] | None:
        return self.state.get(STATE_CURRENT_GOAL)

    @property
    def decision_status(self) -> str:
        decision = self.state.get(STATE_GOAL_DECISION) or {}
        return str(decision.get("previous_goal_status", "?"))

    @property
    def started_new_goal(self) -> bool:
        goal = self.goal or {}
        return bool(goal) and goal.get("status") != REST_STATUS and goal.get("created_tick") == self.state.get(STATE_TICK)

    @property
    def limit_reached(self) -> bool:
        return bool(self.state.get(STATE_LIMIT_REACHED))

    def summary_dict(self) -> dict[str, Any]:
        return {
            "number": self.number,
            "seconds": round(self.seconds, 2),
            "tools": self.tool_calls,
            "errors": len(self.tool_errors),
            "refusals": len(self.refusals),
            "llm_calls": model_call_count(self.events),
            "s1_turns": s1_turn_count(self.events),
            "fallbacks": len(self.fallbacks),
            "truncated": self.truncated,
            "error": self.error,
            "goal_id": (self.goal or {}).get("id"),
            "goal": (self.goal or {}).get("goal"),
            "goal_kind": (self.goal or {}).get("kind"),
            "target_id": (self.goal or {}).get("target_id"),
            "status": self.decision_status,
            "new_goal": self.started_new_goal,
            "limit_reached": self.limit_reached,
        }

    def summary(self) -> str:
        goal = self.goal or {}
        failed = f" failed={self.error}" if self.failed else ""
        return (
            f"tick={self.number} goal_id={goal.get('id')} status={self.decision_status} "
            f"tools={','.join(self.tool_calls) or '-'} errors={len(self.tool_errors)} refusals={len(self.refusals)} "
            f"llm_calls={model_call_count(self.events)} s1_turns={s1_turn_count(self.events)} "
            f"fallbacks={len(self.fallbacks)} truncated={int(self.truncated)}{failed} seconds={self.seconds:.1f}"
        )


EventHook = Callable[[Event, int], None]
TickHook = Callable[[TickResult], None]
StatusHook = Callable[[], None]
NoticeHook = Callable[[dict[str, Any]], None]
"""Receives runtime notices for the portal's Events Log: `{kind, tick, text, timestamp, ...}`."""


def next_delay(result: TickResult, brain: BrainSettings) -> float:
    """Pure: how long the loop rests after `result` (a new goal rests longer, for testing)."""
    if result.limit_reached:
        return 0.0
    return brain.goal_delay_s if result.started_new_goal else brain.tick_delay_s


def failure_backoff(consecutive: int) -> float:
    """Pure: the rest after the `consecutive`-th failed tick in a row: 2 s, doubling, capped at 30 s."""
    return min(MAX_FAILED_TICK_BACKOFF_S, FAILED_TICK_BACKOFF_S * 2 ** max(0, consecutive - 1))


def failure_notice(result: TickResult, seconds: float) -> dict[str, Any]:
    """Pure: the notice announcing a failed tick and the rest before the retry."""
    return {
        "kind": "failure",
        "tick": result.number,
        "seconds": seconds,
        "text": f"Tick {result.number} failed ({result.error}); retrying in {seconds:g} s.",
        "timestamp": time.time(),
    }


def rest_notice(result: TickResult, seconds: float) -> dict[str, Any]:
    """Pure: the notice announcing the rest after `result`, and why it happens."""
    if result.started_new_goal:
        goal_id = (result.goal or {}).get("id")
        why = f"pacing after goal #{goal_id} started (goal_delay_s)"
    else:
        why = "pacing between ticks (tick_delay_s)"
    return {
        "kind": "rest",
        "tick": result.number,
        "seconds": seconds,
        "text": f"Resting {seconds:g} s before the next tick: {why}.",
        "timestamp": time.time(),
    }


def limit_notice(result: TickResult, limit: int | None) -> dict[str, Any]:
    """Pure: the notice announcing that the loop paused itself on the run's goal limit."""
    return {
        "kind": "limit",
        "tick": result.number,
        "text": f"Paused: this run's goal limit ({limit}) is reached. Allow more goals to continue.",
        "timestamp": time.time(),
    }


def person_notice(who: str, tick: int, *, skipped: bool = False) -> dict[str, Any]:
    """Pure: the notice saying a person came into view while the pet rested, so it wakes up
    (`skipped`: a person it saw during the tick, so it does not rest at all)."""
    text = (
        f"A person is near: {who}. The pet does not rest; the goal setter decides about them now."
        if skipped
        else f"A person came into view while resting: {who}. The pet wakes up."
    )
    return {"kind": "person", "tick": tick, "text": text, "timestamp": time.time()}


def cancel_notice(goal: Mapping[str, Any], replacement: Mapping[str, Any], tick: int) -> dict[str, Any]:
    """Pure: the notice saying the owner's new goal cancels the goal in progress."""
    old = str(goal.get("goal") or "").strip()
    new = str(replacement.get("goal") or "").strip()
    return {
        "kind": "cancel",
        "tick": tick,
        "text": f'Canceled goal #{goal.get("id")} ({old[:80]}): the owner\'s goal "{new[:80]}" starts now.',
        "timestamp": time.time(),
    }


class PetRuntime:
    """One pet process: creates the session, runs ticks (optionally in a controllable loop),
    queues goals from the outside, and closes the runner and the toolset."""

    def __init__(
        self,
        brain: Brain,
        *,
        user_id: str = "pet",
        session_id: str | None = None,
        session_service: BaseSessionService | None = None,
        app_name: str = APP_NAME,
        on_event: EventHook | None = None,
        on_tick: TickHook | None = None,
        on_status: StatusHook | None = None,
        on_notice: NoticeHook | None = None,
    ) -> None:
        self.brain = brain
        self.user_id = user_id
        self.app_name = app_name
        self.session_id = session_id or datetime.now(timezone.utc).strftime("run-%Y%m%dT%H%M%SZ")
        self.session_service = session_service or InMemorySessionService()
        app = App(name=app_name, root_agent=brain.workflow, plugins=[StatsPlugin(brain.stats)])
        self.runner = Runner(app=app, session_service=self.session_service)
        self.on_event = on_event
        self.on_tick = on_tick
        self.on_status = on_status
        self.on_notice = on_notice
        if getattr(brain, "people", None) is not None:
            # "Human wants to interact with me!" and "... leaving them be": the pet's social notices
            brain.people.on_event = lambda notice: self._notice({"tick": self.ticks, "timestamp": time.time(), **notice})
        self.ticks = 0
        self.last_tick: TickResult | None = None
        self.running_tick = False
        self._paused = False
        self._stopping = False
        self._tick_requested = False
        self._wake = asyncio.Event()
        self._queued_goals: list[dict[str, Any]] = []
        self._cancel_current = False
        """The owner's new goal cancels the goal in progress as the next tick starts."""
        self._goal_limit: int | None = brain.settings.brain.max_goals_per_run or None
        self.limit_reached = False
        self.waiting_until: float | None = None
        self._warm_up: asyncio.Task[None] | None = None

    # --- lifecycle -------------------------------------------------------------------------

    async def start(self) -> "PetRuntime":
        await self.session_service.create_session(
            app_name=self.app_name,
            user_id=self.user_id,
            session_id=self.session_id,
            state=initial_state(),
        )
        if self.brain.s1 is not None:
            self._warm_up = asyncio.create_task(self.brain.s1.warm_up(), name="s1-warm-up")
        return self

    async def close(self) -> None:
        self._stopping = True
        self._wake.set()
        if self._warm_up is not None and not self._warm_up.done():
            self._warm_up.cancel()
        try:
            await self.runner.close()
        finally:
            try:
                await self.brain.toolset.close()
            finally:
                await self.brain.close()

    async def __aenter__(self) -> "PetRuntime":
        return await self.start()

    async def __aexit__(self, *exc_info: object) -> None:
        await self.close()

    # --- controls --------------------------------------------------------------------------

    @property
    def stats(self) -> RunStats:
        return self.brain.stats

    @property
    def paused(self) -> bool:
        return self._paused

    def pause(self) -> None:
        self._paused = True
        self._notify_status()

    def resume(self) -> None:
        if self.limit_reached:
            log.info("goal limit reached; add more goals before resuming")
            self._notify_status()
            return
        self._paused = False
        self._wake.set()
        self._notify_status()

    @property
    def goal_limit(self) -> int | None:
        return self._goal_limit

    @property
    def goals_used(self) -> int:
        state = self.last_tick.state if self.last_tick else {}
        return int(state.get(STATE_GOALS_STARTED) or 0)

    def extend_goal_limit(self, goals: int) -> int | None:
        """Allow `goals` more goals this run; resumes the loop if it paused on the limit.
        Returns the new limit, or None when the run is unlimited."""
        if goals < 1:
            raise ValueError("goals must be at least 1")
        if self._goal_limit is None:
            return None
        self._goal_limit += goals
        if self.limit_reached:
            self.limit_reached = False
            self._paused = False
            self._wake.set()
        self._notify_status()
        return self._goal_limit

    @property
    def waiting_s(self) -> float | None:
        if self.waiting_until is None:
            return None
        return max(0.0, self.waiting_until - time.monotonic())

    def request_tick(self) -> None:
        """Run one tick even while paused (the loop picks it up)."""
        self._tick_requested = True
        self._wake.set()

    def stop(self) -> None:
        self._stopping = True
        self._wake.set()

    async def run_loop(self, max_ticks: int = 0) -> int:
        """Run ticks until stopped; `max_ticks=0` means forever. Honors pause, the goal limit
        (pauses itself), and the configured rests between ticks. A failed tick (a transient error
        such as a Gemini timeout) is retried after a growing backoff."""
        brain = self.brain.settings.brain
        failures = 0
        while not self._stopping and (max_ticks == 0 or self.ticks < max_ticks):
            if self._paused and not self._tick_requested:
                self._wake.clear()
                await self._wake.wait()
                continue
            self._tick_requested = False
            result = await self.run_tick()
            if self._stopping or (max_ticks and self.ticks >= max_ticks):
                break
            if result.failed:
                failures += 1
                backoff = failure_backoff(failures)
                self._notice(failure_notice(result, backoff))
                await self._rest(backoff)
                continue
            failures = 0
            if result.limit_reached:
                self.limit_reached = True
                log.info("goal limit of %d reached; pausing the loop", brain.max_goals_per_run)
                self._notice(limit_notice(result, self._goal_limit))
                self.pause()
                continue
            delay = next_delay(result, brain)
            if self._queued_goals:
                delay = 0  # the owner's goal is waiting: start it now
            if delay > 0:
                self._notice(rest_notice(result, delay))
            await self._rest(delay, watch=True)
        return self.ticks

    async def _rest(self, seconds: float, *, watch: bool = False) -> None:
        """Sleep between ticks; stop, pause, resume and request_tick all cut it short. With
        `watch` and the person watch (`[people] watch`), the pet looks around every
        `watch_every_s` meanwhile, and a person coming into view wakes it up; so does a person
        the goal setter has not heard about yet (seen during the tick: no rest at all then)."""
        if seconds <= 0:
            return
        person_watch = self.brain.watch if watch else None
        news = self._person_news() if person_watch is not None else None
        if news is not None:
            self._notice(person_notice(news.words(), self.ticks, skipped=True))
            return  # the goal setter decides about them now, not after the rest
        deadline = time.monotonic() + seconds
        self.waiting_until = deadline
        self._notify_status()
        self._wake.clear()
        try:
            while True:
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    break
                step = min(remaining, person_watch.every_s) if person_watch is not None else remaining
                try:
                    await asyncio.wait_for(self._wake.wait(), timeout=step)
                    break  # woken: stop, pause, a tick request or an owner's goal
                except asyncio.TimeoutError:
                    pass
                if person_watch is None or self._paused or time.monotonic() >= deadline:
                    continue
                state = self.last_tick.state if self.last_tick else {}
                sighting = await person_watch.look_between_ticks(self.ticks, state.get(STATE_CURRENT_GOAL))
                if sighting is None:
                    sighting = self._person_news()
                if sighting is not None:
                    self._notice(person_notice(sighting.words(), self.ticks))
                    break
        finally:
            self.waiting_until = None
            self._notify_status()

    def _person_news(self) -> Any:
        """A person the goal setter has not heard about yet (`People.news_pending`), counted as a
        rest the person cut short; None without one or with `[people]` off."""
        people = getattr(self.brain, "people", None)
        if people is None:
            return None
        state = self.last_tick.state if self.last_tick else {}
        sighting = people.news_pending(state.get(STATE_CURRENT_GOAL))
        if sighting is not None:
            people.counts["rest_wakes"] += 1
        return sighting

    # --- goals from outside ----------------------------------------------------------------

    async def submit_goal(self, goal: str, sub_goals: Iterable[str] = (), *, cancel_current: bool = True) -> dict[str, Any]:
        """Queue a goal; it enters session state at the start of the next tick.

        With `cancel_current` (the portal's behaviour) the goal in progress is canceled: a tick
        running now refuses its remaining tool calls and stops its drive (`GoalInterrupt`), and
        the next tick ends the goal as "canceled" and starts the owner's goal at once, without
        the rest between ticks. Goals already waiting in the queue keep their places.

        Owner goals bypass the run's goal limit: a loop that paused itself on the limit resumes,
        and a rest between ticks is cut short so the goal is picked up promptly. A pause the
        owner asked for explicitly is respected.
        """
        raw = await self._raw_state()
        next_id = int(raw.get(STATE_NEXT_GOAL_ID) or 1) + len(self._queued_goals)
        record = pending_goal_record(next_id, goal, sub_goals)
        self._queued_goals.append(record)
        current = raw.get(STATE_CURRENT_GOAL)
        if cancel_current and not self._cancel_current and isinstance(current, dict) and current.get("status") == "active":
            self._cancel_current = True
            if self.running_tick:
                self.brain.interrupt.request(f'the owner gave a new goal ("{record["goal"][:60]}")')
            self._notice(cancel_notice(current, record, self.ticks))
        if self.limit_reached:
            self.limit_reached = False
            self._paused = False
        self._wake.set()
        self._notify_status()
        return record

    def _take_queued_delta(self, raw: dict[str, Any]) -> dict[str, Any] | None:
        """State to apply at the start of a tick: the canceled goal, queued goals and the
        goal-limit window."""
        delta: dict[str, Any] = {}
        if self._cancel_current:
            self._cancel_current = False
            delta.update(cancel_current_goal(raw, self.brain.settings.brain.goal_history_limit) or {})
        if self._queued_goals:
            pending = list(raw.get(STATE_PENDING_GOALS) or []) + self._queued_goals
            next_id = max(int(raw.get(STATE_NEXT_GOAL_ID) or 1), max(int(g["id"]) for g in self._queued_goals) + 1)
            self._queued_goals = []
            delta[STATE_PENDING_GOALS] = pending
            delta[STATE_NEXT_GOAL_ID] = next_id
        if self._goal_limit is not None and raw.get(STATE_GOAL_LIMIT) != self._goal_limit:
            delta[STATE_GOAL_LIMIT] = self._goal_limit
        return delta or None

    # --- state -----------------------------------------------------------------------------

    async def _raw_state(self) -> dict[str, Any]:
        session = await self.session_service.get_session(
            app_name=self.app_name, user_id=self.user_id, session_id=self.session_id
        )
        return dict(session.state) if session is not None else {}

    async def state(self) -> dict[str, Any]:
        """Session state, with goals queued since the last tick merged in."""
        raw = await self._raw_state()
        if self._queued_goals:
            raw[STATE_PENDING_GOALS] = list(raw.get(STATE_PENDING_GOALS) or []) + list(self._queued_goals)
        return raw

    def default_run_config(self) -> RunConfig:
        return RunConfig(max_llm_calls=self.brain.settings.brain.max_llm_calls_per_tick)

    # --- one tick --------------------------------------------------------------------------

    async def run_tick(self, run_config: RunConfig | None = None) -> TickResult:
        """Run one tick. Hitting the LLM call budget truncates it; a transient error (see
        `is_transient_model_error`) fails it. Either way the tick returns a result; any other
        error propagates."""
        self.ticks += 1
        number = self.ticks
        started = time.monotonic()
        events: list[Event] = []
        truncated = False
        error: str | None = None
        raw = await self._raw_state()
        self.brain.interrupt.clear()  # a cancel requested during the last tick has done its job
        state_delta = self._take_queued_delta(raw)
        self.running_tick = True
        self._notify_status()
        try:
            async for event in self.runner.run_async(
                user_id=self.user_id,
                session_id=self.session_id,
                new_message=tick_message(number),
                state_delta=state_delta,
                run_config=run_config or self.default_run_config(),
            ):
                events.append(event)
                self._emit(event, number)
        except LlmCallsLimitExceededError as exc:
            truncated = True
            log.warning("tick %d truncated: %s", number, exc)
            await self._note_report(number, "truncated", BUDGET_EXHAUSTED_REPORT)
        except ExceptionGroup as group:
            if group.subgroup(LlmCallsLimitExceededError) is not None:
                truncated = True
                log.warning("tick %d truncated: %s", number, group)
                await self._note_report(number, "truncated", BUDGET_EXHAUSTED_REPORT)
            elif is_transient_model_error(group):
                error = await self._note_failure(number, group, events)
            else:
                raise
        except Exception as exc:
            if not is_transient_model_error(exc):
                raise
            error = await self._note_failure(number, exc, events)
        finally:
            self.running_tick = False
        result = TickResult(
            number=number,
            seconds=time.monotonic() - started,
            events=events,
            state=await self.state(),
            truncated=truncated,
            error=error,
        )
        self.last_tick = result
        self.stats.record_tick(result)
        if self.on_tick is not None:
            self.on_tick(result)
        self._notify_status()
        return result

    def _emit(self, event: Event, number: int) -> None:
        if self.on_event is None:
            return
        try:
            self.on_event(event, number)
        except Exception:  # noqa: BLE001 - a broken listener must not stop the pet
            log.exception("event hook failed")

    def _notify_status(self) -> None:
        if self.on_status is None:
            return
        try:
            self.on_status()
        except Exception:  # noqa: BLE001
            log.exception("status hook failed")

    def _notice(self, notice: dict[str, Any]) -> None:
        log.info("%s", notice["text"])
        if self.on_notice is None:
            return
        try:
            self.on_notice(notice)
        except Exception:  # noqa: BLE001
            log.exception("notice hook failed")

    async def _note_failure(self, number: int, exc: BaseException, events: list[Event]) -> str:
        """Log a failed tick and return its error name. If the executor had started, its report
        from an earlier tick is stale, so the next goal-setter turn is told it never reported; a
        tick that failed before the executor started leaves the state for the retry as it was."""
        error = error_name(exc)
        log.warning("tick %d failed (%s): %s", number, error, exc)
        if any(event.author == EXECUTOR_NAME for event in events):
            await self._note_report(number, "failed", executor_failed_report(error))
        return error

    async def _note_report(self, number: int, kind: str, report: str) -> None:
        """Stand in for the executor's report when it never wrote one this tick."""
        session = await self.session_service.get_session(
            app_name=self.app_name, user_id=self.user_id, session_id=self.session_id
        )
        if session is None:
            return
        await self.session_service.append_event(
            session,
            Event(
                author=RUNTIME_AUTHOR,
                invocation_id=f"{kind}-tick-{number}",
                actions=EventActions(state_delta={STATE_LAST_REPORT: report}),
            ),
        )
