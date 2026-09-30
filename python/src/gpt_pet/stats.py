"""Live run statistics: LLM calls and tokens, latencies, tools, drives, goals.

`RunStats` accumulates one pet run in memory. `StatsPlugin` feeds it every model and tool call
through ADK's plugin hooks, with exact timings and Gemini's usage metadata; drives arrive from
`gpt_pet.navwait` and ticks from `PetRuntime`. `snapshot()` is what the portal shows
(`GET /api/stats` and the `stats` SSE event).

S1 (`gpt_pet.s1`) reports the turns it takes instead of Gemini, the turns it hands to Gemini and
why, and every decision-model call. The drive wait's status checks are S1 work too: rules that
replace the model's polling.
"""
from __future__ import annotations

import logging
import math
import time
from collections import deque
from collections.abc import Callable
from typing import TYPE_CHECKING, Any

from google.adk.plugins.base_plugin import BasePlugin

from gpt_pet.agents.executor import is_budget_refusal

if TYPE_CHECKING:
    from gpt_pet.navwait import DriveWait
    from gpt_pet.runtime import TickResult

LATENCY_WINDOW = 200
"""Percentiles cover the most recent samples; counts and means cover the whole run."""
ACTUATION_TOOLS = frozenset({"set_nav_goal", "do_move", "do_rotate"})
TOKEN_FIELDS = {
    "input": "prompt_token_count",
    "output": "candidates_token_count",
    "thinking": "thoughts_token_count",
    "cached": "cached_content_token_count",
    "total": "total_token_count",
}

log = logging.getLogger("gpt_pet.stats")


def percentile(ordered: list[float], pct: float) -> float:
    """Nearest-rank percentile of an ascending, non-empty list."""
    rank = max(1, math.ceil(pct / 100 * len(ordered)))
    return ordered[rank - 1]


class Latencies:
    def __init__(self) -> None:
        self.count = 0
        self.total_s = 0.0
        self.recent: deque[float] = deque(maxlen=LATENCY_WINDOW)

    def add(self, seconds: float) -> None:
        self.count += 1
        self.total_s += seconds
        self.recent.append(seconds)

    def summary(self) -> dict[str, Any]:
        if not self.count:
            return {"count": 0, "avg_ms": None, "p50_ms": None, "p95_ms": None, "last_ms": None}
        ordered = sorted(self.recent)
        return {
            "count": self.count,
            "avg_ms": round(1000 * self.total_s / self.count),
            "p50_ms": round(1000 * percentile(ordered, 50)),
            "p95_ms": round(1000 * percentile(ordered, 95)),
            "last_ms": round(1000 * self.recent[-1]),
        }


def usage_tokens(usage: Any) -> dict[str, int]:
    """Gemini usage metadata as plain counts; missing fields count as zero."""
    counts = {key: int(getattr(usage, name, None) or 0) for key, name in TOKEN_FIELDS.items()}
    if not counts["total"]:
        counts["total"] = counts["input"] + counts["output"] + counts["thinking"]
    return counts


def tool_result_is_error(result: Any) -> bool:
    return isinstance(result, dict) and bool(result.get("isError") or result.get("is_error") or "error" in result)


class RunStats:
    """One run's counters. Every `record_*` call notifies `on_change` (the server throttles it)."""

    def __init__(self, started: float | None = None, wall_started: float | None = None) -> None:
        self.on_change: Callable[[], None] | None = None
        self.s1_model: str | None = None
        """The S1 decision model when `[s1]` is enabled (configuration: a reset keeps it)."""
        self._goals_seen: set[Any] = set()
        self._finished_seen: set[Any] = set()
        self.reset(started, wall_started)

    def reset(self, started: float | None = None, wall_started: float | None = None) -> None:
        """Zero every counter. Goals already seen stay seen, so none is counted twice."""
        self.started = time.monotonic() if started is None else started
        self.started_at = time.time() if wall_started is None else wall_started
        self.llm = Latencies()
        self.llm_errors = 0
        self.agents: dict[str, dict[str, float]] = {}
        self.tokens = dict.fromkeys(TOKEN_FIELDS, 0)
        self.last_call_tokens = 0
        self.tool_calls = 0
        self.tool_errors = 0
        self.tool_refusals = 0
        self.tools = Latencies()
        self.first_action_s: float | None = None
        self.ticks = Latencies()
        self.truncated = 0
        self.goals_started = 0
        self.goals_done = 0
        self.goals_abandoned = 0
        self.drives = Latencies()
        self.drive_outcomes: dict[str, int] = {}
        self.drive_checks = 0
        self.decider = Latencies()
        self.s1_turns: dict[str, int] = {}
        self.s1_agents: dict[str, int] = {}
        self.s1_escalations: dict[str, int] = {}
        self._changed()

    # --- recording -------------------------------------------------------------------------

    def record_llm_call(self, agent: str, seconds: float, usage: Any = None) -> None:
        tokens = usage_tokens(usage)
        self.llm.add(seconds)
        per_agent = self.agents.setdefault(agent, {"calls": 0, "tokens": 0, "seconds": 0.0})
        per_agent["calls"] += 1
        per_agent["tokens"] += tokens["total"]
        per_agent["seconds"] += seconds
        for key, value in tokens.items():
            self.tokens[key] += value
        self.last_call_tokens = tokens["total"]
        self._changed()

    def record_llm_error(self) -> None:
        self.llm_errors += 1
        self._changed()

    def record_tool_call(self, name: str, seconds: float, *, error: bool = False, refused: bool = False, now: float | None = None) -> None:
        """Every call the model makes; refused ones never ran, so they have no latency."""
        self.tool_calls += 1
        if refused:
            self.tool_refusals += 1
        else:
            self.tools.add(seconds)
            if error:
                self.tool_errors += 1
            elif name in ACTUATION_TOOLS and self.first_action_s is None:
                self.first_action_s = (time.monotonic() if now is None else now) - self.started
        self._changed()

    def record_drive(self, wait: "DriveWait") -> None:
        self.drives.add(wait.waited_s)
        self.drive_outcomes[wait.outcome] = self.drive_outcomes.get(wait.outcome, 0) + 1
        self.drive_checks += wait.checks
        self._changed()

    def record_decider_call(self, seconds: float) -> None:
        """One answered call to the S1 decision model."""
        self.decider.add(seconds)
        self._changed()

    def record_s1_turn(self, agent: str, by: str) -> None:
        """A model turn S1 took instead of Gemini, by `rules` or by the decision model."""
        self.s1_turns[by] = self.s1_turns.get(by, 0) + 1
        self.s1_agents[agent] = self.s1_agents.get(agent, 0) + 1
        self._changed()

    def record_s1_escalation(self, agent: str, reason: str) -> None:
        """A turn S1 handed to Gemini, and why."""
        key = f"{agent}: {reason}"
        self.s1_escalations[key] = self.s1_escalations.get(key, 0) + 1
        self._changed()

    def record_tick(self, result: "TickResult") -> None:
        self.ticks.add(result.seconds)
        if result.truncated:
            self.truncated += 1
        goal = result.state.get("current_goal") or {}
        if goal.get("status") == "active" and goal.get("id") not in self._goals_seen:
            self._goals_seen.add(goal.get("id"))
            self.goals_started += 1
        for finished in result.state.get("goal_history") or []:
            goal_id = finished.get("id")
            if goal_id in self._finished_seen:
                continue
            self._finished_seen.add(goal_id)
            if finished.get("status") == "done":
                self.goals_done += 1
            else:
                self.goals_abandoned += 1
        self._changed()

    def _changed(self) -> None:
        if self.on_change is None:
            return
        try:
            self.on_change()
        except Exception:  # noqa: BLE001 - a broken listener must not stop the pet
            log.exception("stats listener failed")

    # --- reading ---------------------------------------------------------------------------

    def snapshot(self, now: float | None = None) -> dict[str, Any]:
        uptime = max(0.0, (time.monotonic() if now is None else now) - self.started)
        minutes = uptime / 60

        def per_min(count: float) -> float:
            return round(count / minutes, 2) if minutes > 0 else 0.0

        s1_turns = sum(self.s1_turns.values())
        model_turns = s1_turns + self.llm.count
        return {
            "started_at": self.started_at,
            "uptime_s": round(uptime, 1),
            "llm": {
                "calls": self.llm.count,
                "errors": self.llm_errors,
                "per_min": per_min(self.llm.count),
                "latency": self.llm.summary(),
                "by_agent": {
                    name: {"calls": int(a["calls"]), "tokens": int(a["tokens"]), "avg_ms": round(1000 * a["seconds"] / a["calls"])}
                    for name, a in sorted(self.agents.items())
                },
            },
            "tokens": {**self.tokens, "per_min": per_min(self.tokens["total"]), "last_call": self.last_call_tokens},
            "tools": {
                "calls": self.tool_calls,
                "errors": self.tool_errors,
                "refusals": self.tool_refusals,
                "latency": self.tools.summary(),
            },
            "ticks": {"count": self.ticks.count, "truncated": self.truncated, "latency": self.ticks.summary()},
            "goals": {
                "started": self.goals_started,
                "done": self.goals_done,
                "abandoned": self.goals_abandoned,
                "started_per_min": per_min(self.goals_started),
                "done_per_min": per_min(self.goals_done),
            },
            "drives": {
                "count": self.drives.count,
                "outcomes": dict(sorted(self.drive_outcomes.items())),
                "driving_s": round(self.drives.total_s, 1),
                "driving_pct": round(100 * self.drives.total_s / uptime, 1) if uptime > 0 else 0.0,
                "checks": self.drive_checks,
                "latency": self.drives.summary(),
            },
            "s1": {
                "enabled": self.s1_model is not None,
                "model": self.s1_model,
                "turns": s1_turns,
                "share_pct": round(100 * s1_turns / model_turns, 1) if model_turns else 0.0,
                "by": dict(sorted(self.s1_turns.items())),
                "by_agent": dict(sorted(self.s1_agents.items())),
                "escalations": dict(sorted(self.s1_escalations.items(), key=lambda item: (-item[1], item[0]))),
                "decider": {"calls": self.decider.count, "latency": self.decider.summary()},
            },
            "first_action_s": None if self.first_action_s is None else round(self.first_action_s, 1),
        }


class StatsPlugin(BasePlugin):
    """Feeds `RunStats` from ADK's model and tool hooks. It only observes: every hook returns None."""

    def __init__(self, stats: RunStats) -> None:
        super().__init__(name="gpt_pet_stats")
        self.stats = stats
        self._model_started: dict[tuple[str, str], float] = {}
        self._tool_started: dict[str, float] = {}

    async def before_model_callback(self, *, callback_context: Any, llm_request: Any) -> None:
        self._model_started[(callback_context.invocation_id, callback_context.agent_name)] = time.monotonic()

    async def after_model_callback(self, *, callback_context: Any, llm_response: Any) -> None:
        if llm_response.partial:
            return
        started = self._model_started.pop((callback_context.invocation_id, callback_context.agent_name), None)
        if started is not None:
            self.stats.record_llm_call(callback_context.agent_name, time.monotonic() - started, llm_response.usage_metadata)

    async def on_model_error_callback(self, *, callback_context: Any, llm_request: Any, error: Exception) -> None:
        self._model_started.pop((callback_context.invocation_id, callback_context.agent_name), None)
        self.stats.record_llm_error()

    async def before_tool_callback(self, *, tool: Any, tool_args: dict[str, Any], tool_context: Any) -> None:
        self._tool_started[_call_key(tool_context)] = time.monotonic()

    async def after_run_callback(self, *, invocation_context: Any) -> None:
        # A turn S1 answered never reaches after_model_callback; drop the timer it started.
        self._model_started.clear()

    async def after_tool_callback(self, *, tool: Any, tool_args: dict[str, Any], tool_context: Any, result: Any) -> None:
        # Runs before the agent's own after-tool callbacks, so a drive's wait is not included.
        started = self._tool_started.pop(_call_key(tool_context), None)
        seconds = time.monotonic() - started if started is not None else 0.0
        refused = is_budget_refusal(result)
        self.stats.record_tool_call(tool.name, seconds, error=not refused and tool_result_is_error(result), refused=refused)


def _call_key(tool_context: Any) -> str:
    return getattr(tool_context, "function_call_id", None) or str(id(tool_context))
