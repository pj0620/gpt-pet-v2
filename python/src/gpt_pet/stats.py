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
from collections.abc import Callable, Mapping
from typing import TYPE_CHECKING, Any

from google.adk.plugins.base_plugin import BasePlugin

from gpt_pet.agents.executor import is_budget_refusal
from gpt_pet.agents.ollama import RELOAD_S

if TYPE_CHECKING:
    from gpt_pet.navwait import DriveWait
    from gpt_pet.runtime import TickResult

LATENCY_WINDOW = 200
"""Percentiles cover the most recent samples; counts and means cover the whole run."""
ACTUATION_TOOLS = frozenset({"set_nav_goal", "do_move", "do_rotate", "do_look"})
DRIVE_TOOL = "set_nav_goal"
PASSAGE_PREFIX = "psg:"
"""Destination ids of passages (doorways, openings) in the robot contract; objects are `obj:`."""
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
        self.max_s = 0.0
        self.recent: deque[float] = deque(maxlen=LATENCY_WINDOW)

    def add(self, seconds: float) -> None:
        self.count += 1
        self.total_s += seconds
        self.max_s = max(self.max_s, seconds)
        self.recent.append(seconds)

    def summary(self) -> dict[str, Any]:
        if not self.count:
            return {"count": 0, "avg_ms": None, "p50_ms": None, "p95_ms": None, "max_ms": None, "last_ms": None}
        ordered = sorted(self.recent)
        return {
            "count": self.count,
            "avg_ms": round(1000 * self.total_s / self.count),
            "p50_ms": round(1000 * percentile(ordered, 50)),
            "p95_ms": round(1000 * percentile(ordered, 95)),
            "max_ms": round(1000 * self.max_s),
            "last_ms": round(1000 * self.recent[-1]),
        }


class LocalModelTimes:
    """Where a local model's calls (`gpt_pet.agents.ollama`) spent their time, by Ollama's own
    account: loading the model (a reload: it had been unloaded, e.g. to make room for another
    model), reading the prompt, and writing the answer."""

    def __init__(self) -> None:
        self.calls = 0
        self.reloads = 0
        self.load_s = 0.0
        self.prompt_tokens = 0
        self.prompt_s = 0.0
        self.output_tokens = 0
        self.output_s = 0.0
        self.max_prompt_tokens = 0

    def add(self, timed: Mapping[str, Any]) -> None:
        self.calls += 1
        load_s = float(timed.get("load_s") or 0)
        self.load_s += load_s
        if load_s > RELOAD_S:
            self.reloads += 1
        prompt_tokens = int(timed.get("prompt_tokens") or 0)
        self.prompt_tokens += prompt_tokens
        self.prompt_s += float(timed.get("prompt_s") or 0)
        self.output_tokens += int(timed.get("output_tokens") or 0)
        self.output_s += float(timed.get("output_s") or 0)
        self.max_prompt_tokens = max(self.max_prompt_tokens, prompt_tokens)

    def summary(self) -> dict[str, Any]:
        if not self.calls:
            return {"calls": 0}

        def per_s(tokens: int, seconds: float) -> float | None:
            return round(tokens / seconds, 1) if seconds > 0 else None

        return {
            "calls": self.calls,
            "reloads": self.reloads,
            "load_s": round(self.load_s, 1),
            "prompt_s": round(self.prompt_s, 1),
            "output_s": round(self.output_s, 1),
            "prompt_tok_s": per_s(self.prompt_tokens, self.prompt_s),
            "output_tok_s": per_s(self.output_tokens, self.output_s),
            "max_prompt_tokens": self.max_prompt_tokens,
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
        self.people_counts: Callable[[], dict[str, int]] | None = None
        """With `[people]`: the people memory's counters (sightings, watch looks, drives the watch
        stopped, rests it woke, attentions, goal decisions a person caused)."""
        self._goals_seen: set[Any] = set()
        self._finished_seen: set[Any] = set()
        self.reset(started, wall_started)

    def reset(self, started: float | None = None, wall_started: float | None = None) -> None:
        """Zero every counter. Goals already seen stay seen, so none is counted twice."""
        self.started = time.monotonic() if started is None else started
        self.started_at = time.time() if wall_started is None else wall_started
        self.llm = Latencies()
        self.llm_errors = 0
        self.llm_failed = Latencies()
        """How long the failed Gemini calls took to fail (a timeout shows up here, not in `llm`)."""
        self.llm_fallbacks = 0
        self.by_model: dict[str, dict[str, float]] = {}
        """Calls and tokens per model, for pricing a run (`gpt_pet.experiment.run_cost`)."""
        self.local = LocalModelTimes()
        self.maps_shown = 0
        """Goal decisions Gemini made with the robot's map in view (`[brain] map_for_goals`)."""
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
        self.ticks_failed = 0
        self.goals_started = 0
        self.goals_done = 0
        self.goals_abandoned = 0
        self.goals_canceled = 0
        self.drives = Latencies()
        self.drive_outcomes: dict[str, int] = {}
        self.drive_checks = 0
        self.null_drives = 0
        self.passage_drives = 0
        self.drive_m = 0.0
        self.destinations_reached: set[str] = set()
        self.drive_calls: dict[str, int] = {}
        """set_nav_goal calls by who decided them: "s1" or "gemini"."""
        self.decider = Latencies()
        self.s1_turns: dict[str, int] = {}
        self.s1_agents: dict[str, int] = {}
        self.s1_escalations: dict[str, int] = {}
        self._changed()

    # --- recording -------------------------------------------------------------------------

    def record_llm_call(
        self,
        agent: str,
        seconds: float,
        usage: Any = None,
        local: Mapping[str, Any] | None = None,
        *,
        model: str | None = None,
        billing: Mapping[str, Any] | None = None,
    ) -> None:
        """One answered call; `local` is a local model's own timings (`gpt_pet.agents.ollama`),
        `billing` what an adapter knows beyond the token counts: Anthropic's cache writes, the
        cost OpenRouter reports."""
        tokens = usage_tokens(usage)
        self.llm.add(seconds)
        if local:
            self.local.add(local)
        if model:
            counts = self.by_model.setdefault(model, dict.fromkeys(("calls", "input", "cached", "cache_write", "output", "thinking", "billed_usd"), 0))
            counts["calls"] += 1
            for key in ("input", "cached", "output", "thinking"):
                counts[key] += tokens[key]
            counts["cache_write"] += int((billing or {}).get("cache_write_tokens") or 0)
            counts["billed_usd"] += float((billing or {}).get("cost_usd") or 0)
        per_agent = self.agents.setdefault(agent, {"calls": 0, "tokens": 0, "seconds": 0.0})
        per_agent["calls"] += 1
        per_agent["tokens"] += tokens["total"]
        per_agent["seconds"] += seconds
        for key, value in tokens.items():
            self.tokens[key] += value
        self.last_call_tokens = tokens["total"]
        self._changed()

    def record_llm_error(self, seconds: float | None = None) -> None:
        """A Gemini call that raised; `seconds` is how long it took to fail, when known."""
        self.llm_errors += 1
        if seconds is not None:
            self.llm_failed.add(seconds)
        self._changed()

    def record_llm_fallback(self) -> None:
        """A failed Gemini call whose agent fallback answered instead (`gpt_pet.agents.fallbacks`)."""
        self.llm_fallbacks += 1
        self._changed()

    def record_map_shown(self) -> None:
        """The goal setter's request carries the robot's map."""
        self.maps_shown += 1
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
        if wait.null:
            self.null_drives += 1
        if (wait.destination_id or "").startswith(PASSAGE_PREFIX):
            self.passage_drives += 1
        self.drive_m += wait.traveled_m or 0.0
        if wait.outcome == "succeeded" and wait.destination_id:
            self.destinations_reached.add(wait.destination_id)
        self._changed()

    def record_drive_call(self, by: str) -> None:
        """A set_nav_goal call, by who decided it ("s1" or "gemini")."""
        self.drive_calls[by] = self.drive_calls.get(by, 0) + 1
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
        if result.failed:
            self.ticks_failed += 1
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
            elif finished.get("status") == "canceled":
                self.goals_canceled += 1  # the owner replaced it: not the pet giving up
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
                "fallbacks": self.llm_fallbacks,
                "maps_shown": self.maps_shown,
                "per_min": per_min(self.llm.count),
                "latency": self.llm.summary(),
                "failed_latency": self.llm_failed.summary(),
                "local": self.local.summary(),
                "by_model": {name: {key: round(value, 6) if key == "billed_usd" else int(value) for key, value in counts.items()} for name, counts in sorted(self.by_model.items())},
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
            "ticks": {
                "count": self.ticks.count,
                "truncated": self.truncated,
                "failed": self.ticks_failed,
                "latency": self.ticks.summary(),
            },
            "goals": {
                "started": self.goals_started,
                "done": self.goals_done,
                "abandoned": self.goals_abandoned,
                "canceled": self.goals_canceled,
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
                "null": self.null_drives,
                "to_passages": self.passage_drives,
                "distance_m": round(self.drive_m, 2),
                "destinations_reached": len(self.destinations_reached),
                "calls_by": dict(sorted(self.drive_calls.items())),
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
            "people": self.people_counts() if self.people_counts is not None else None,
        }


class StatsPlugin(BasePlugin):
    """Feeds `RunStats` from ADK's model and tool hooks. It only observes: every hook returns None."""

    def __init__(self, stats: RunStats) -> None:
        super().__init__(name="gpt_pet_stats")
        self.stats = stats
        self._model_started: dict[tuple[str, str], tuple[float, str]] = {}
        """(invocation, agent) -> (when its model call started, the model it asked)."""
        self._tool_started: dict[str, float] = {}

    async def before_model_callback(self, *, callback_context: Any, llm_request: Any) -> None:
        key = (callback_context.invocation_id, callback_context.agent_name)
        self._model_started[key] = (time.monotonic(), str(getattr(llm_request, "model", None) or ""))

    async def after_model_callback(self, *, callback_context: Any, llm_response: Any) -> None:
        if llm_response.partial:
            return
        metadata = llm_response.custom_metadata or {}
        if metadata.get("fallback"):
            # The failed call was timed by on_model_error_callback; this is its stand-in.
            self.stats.record_llm_fallback()
            return
        started = self._model_started.pop((callback_context.invocation_id, callback_context.agent_name), None)
        if started is not None:
            began, model = started
            self.stats.record_llm_call(
                callback_context.agent_name,
                time.monotonic() - began,
                llm_response.usage_metadata,
                metadata.get("ollama"),
                model=model or None,
                billing=metadata.get("billing"),
            )

    async def on_model_error_callback(self, *, callback_context: Any, llm_request: Any, error: Exception) -> None:
        started = self._model_started.pop((callback_context.invocation_id, callback_context.agent_name), None)
        self.stats.record_llm_error(None if started is None else time.monotonic() - started[0])

    async def before_tool_callback(self, *, tool: Any, tool_args: dict[str, Any], tool_context: Any) -> None:
        self._tool_started[_call_key(tool_context)] = time.monotonic()

    async def on_event_callback(self, *, invocation_context: Any, event: Any) -> None:
        if event.partial or event.content is None:
            return
        by = "s1" if (event.custom_metadata or {}).get("s1") else "gemini"
        for call in event.get_function_calls():
            if call.name == DRIVE_TOOL:
                self.stats.record_drive_call(by)

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
