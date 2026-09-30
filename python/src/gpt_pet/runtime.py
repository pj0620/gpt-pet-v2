"""Runs the brain tick by tick: one Runner, one session, loop controls, explicit cleanup.

Shared by the CLI, the server, and the live tests so all drive the pet exactly the same way.
"""
from __future__ import annotations

import asyncio
import logging
import time
from collections.abc import Callable, Iterable
from dataclasses import dataclass
from datetime import datetime, timezone
from typing import Any

from google.adk.agents.invocation_context import LlmCallsLimitExceededError
from google.adk.agents.run_config import RunConfig
from google.adk.events.event import Event
from google.adk.events.event_actions import EventActions
from google.adk.runners import Runner
from google.adk.sessions import InMemorySessionService
from google.adk.sessions.base_session_service import BaseSessionService
from google.genai import types

from gpt_pet.agents.executor import is_budget_refusal
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
    initial_state,
    pending_goal_record,
)
from gpt_pet.settings import BrainSettings

APP_NAME = "gpt_pet"
RUNTIME_AUTHOR = "gpt_pet_runtime"
BUDGET_EXHAUSTED_REPORT = (
    "The executor ran out of its LLM call budget before writing a report; treat the goal as not done yet."
)

log = logging.getLogger("gpt_pet.runtime")


def tick_message(number: int) -> types.Content:
    return types.Content(role="user", parts=[types.Part(text=f"tick {number}")])


def function_calls(events: Iterable[Event]) -> list[types.FunctionCall]:
    return [fc for event in events for fc in event.get_function_calls()]


def function_responses(events: Iterable[Event]) -> list[types.FunctionResponse]:
    return [fr for event in events for fr in event.get_function_responses()]


def model_call_count(events: Iterable[Event]) -> int:
    return sum(1 for e in events if e.content is not None and e.content.role == "model" and not e.partial)


def response_is_error(response: types.FunctionResponse) -> bool:
    """A real tool failure. Budget refusals are the executor's caps working, not errors."""
    payload = response.response
    if not isinstance(payload, dict) or is_budget_refusal(payload):
        return False
    return bool(payload.get("isError") or payload.get("is_error") or "error" in payload)


def response_is_refusal(response: types.FunctionResponse) -> bool:
    return is_budget_refusal(response.response)


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
            "truncated": self.truncated,
            "goal_id": (self.goal or {}).get("id"),
            "status": self.decision_status,
            "new_goal": self.started_new_goal,
            "limit_reached": self.limit_reached,
        }

    def summary(self) -> str:
        goal = self.goal or {}
        return (
            f"tick={self.number} goal_id={goal.get('id')} status={self.decision_status} "
            f"tools={','.join(self.tool_calls) or '-'} errors={len(self.tool_errors)} refusals={len(self.refusals)} "
            f"llm_calls={model_call_count(self.events)} truncated={int(self.truncated)} "
            f"seconds={self.seconds:.1f}"
        )


EventHook = Callable[[Event, int], None]
TickHook = Callable[[TickResult], None]
StatusHook = Callable[[], None]


def next_delay(result: TickResult, brain: BrainSettings) -> float:
    """Pure: how long the loop rests after `result` (a new goal rests longer, for testing)."""
    if result.limit_reached:
        return 0.0
    return brain.goal_delay_s if result.started_new_goal else brain.tick_delay_s


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
    ) -> None:
        self.brain = brain
        self.user_id = user_id
        self.app_name = app_name
        self.session_id = session_id or datetime.now(timezone.utc).strftime("run-%Y%m%dT%H%M%SZ")
        self.session_service = session_service or InMemorySessionService()
        self.runner = Runner(node=brain.workflow, app_name=app_name, session_service=self.session_service)
        self.on_event = on_event
        self.on_tick = on_tick
        self.on_status = on_status
        self.ticks = 0
        self.last_tick: TickResult | None = None
        self.running_tick = False
        self._paused = False
        self._stopping = False
        self._tick_requested = False
        self._wake = asyncio.Event()
        self._queued_goals: list[dict[str, Any]] = []
        self._goal_limit: int | None = brain.settings.brain.max_goals_per_run or None
        self.limit_reached = False
        self.waiting_until: float | None = None

    # --- lifecycle -------------------------------------------------------------------------

    async def start(self) -> "PetRuntime":
        await self.session_service.create_session(
            app_name=self.app_name,
            user_id=self.user_id,
            session_id=self.session_id,
            state=initial_state(),
        )
        return self

    async def close(self) -> None:
        self._stopping = True
        self._wake.set()
        try:
            await self.runner.close()
        finally:
            await self.brain.toolset.close()

    async def __aenter__(self) -> "PetRuntime":
        return await self.start()

    async def __aexit__(self, *exc_info: object) -> None:
        await self.close()

    # --- controls --------------------------------------------------------------------------

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
        (pauses itself), and the configured rests between ticks."""
        brain = self.brain.settings.brain
        while not self._stopping and (max_ticks == 0 or self.ticks < max_ticks):
            if self._paused and not self._tick_requested:
                self._wake.clear()
                await self._wake.wait()
                continue
            self._tick_requested = False
            result = await self.run_tick()
            if self._stopping or (max_ticks and self.ticks >= max_ticks):
                break
            if result.limit_reached:
                self.limit_reached = True
                log.info("goal limit of %d reached; pausing the loop", brain.max_goals_per_run)
                self.pause()
                continue
            await self._rest(next_delay(result, brain))
        return self.ticks

    async def _rest(self, seconds: float) -> None:
        """Sleep between ticks; stop, pause, resume and request_tick all cut it short."""
        if seconds <= 0:
            return
        self.waiting_until = time.monotonic() + seconds
        self._notify_status()
        self._wake.clear()
        try:
            await asyncio.wait_for(self._wake.wait(), timeout=seconds)
        except asyncio.TimeoutError:
            pass
        finally:
            self.waiting_until = None
            self._notify_status()

    # --- goals from outside ----------------------------------------------------------------

    async def submit_goal(self, goal: str, sub_goals: Iterable[str] = ()) -> dict[str, Any]:
        """Queue a goal; it enters session state at the start of the next tick.

        Owner goals bypass the run's goal limit: a loop that paused itself on the limit resumes,
        and a rest between ticks is cut short so the goal is picked up promptly. A pause the
        owner asked for explicitly is respected.
        """
        raw = await self._raw_state()
        next_id = int(raw.get(STATE_NEXT_GOAL_ID) or 1) + len(self._queued_goals)
        record = pending_goal_record(next_id, goal, sub_goals)
        self._queued_goals.append(record)
        if self.limit_reached:
            self.limit_reached = False
            self._paused = False
        self._wake.set()
        self._notify_status()
        return record

    def _take_queued_delta(self, raw: dict[str, Any]) -> dict[str, Any] | None:
        """State to apply at the start of a tick: queued goals and the goal-limit window."""
        delta: dict[str, Any] = {}
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
        self.ticks += 1
        number = self.ticks
        started = time.monotonic()
        events: list[Event] = []
        truncated = False
        state_delta = self._take_queued_delta(await self._raw_state())
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
            await self._note_truncation(number)
        except ExceptionGroup as group:
            if group.subgroup(LlmCallsLimitExceededError) is None:
                raise
            truncated = True
            log.warning("tick %d truncated: %s", number, group)
            await self._note_truncation(number)
        finally:
            self.running_tick = False
        result = TickResult(
            number=number,
            seconds=time.monotonic() - started,
            events=events,
            state=await self.state(),
            truncated=truncated,
        )
        self.last_tick = result
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

    async def _note_truncation(self, number: int) -> None:
        """Tell the next goal-setter turn that the executor never reported."""
        session = await self.session_service.get_session(
            app_name=self.app_name, user_id=self.user_id, session_id=self.session_id
        )
        if session is None:
            return
        await self.session_service.append_event(
            session,
            Event(
                author=RUNTIME_AUTHOR,
                invocation_id=f"truncated-tick-{number}",
                actions=EventActions(state_delta={STATE_LAST_REPORT: BUDGET_EXHAUSTED_REPORT}),
            ),
        )
