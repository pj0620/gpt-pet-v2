"""Experiments: run the pet for a fixed time under a test config and a bot config, and record its
totals as one CSV row per run.

A test config says what to run (`test_time`, `room_id`, `repeats`); a bot config says which brain
(a settings profile plus overrides of any ini section, e.g. `{"s1": {"enabled": false}}`). A suite
crosses its tests with its bots and runs each pair `repeats` times, interleaving the bots (A B,
B A, ...) unless told otherwise. Every run gets a fresh simulator with the requested room on a
free port, so runs start from the same state and never touch another simulator; the clock starts
once the simulator and S1's model have booted.

Results append to `experiments/results/results.csv` (new metrics add columns), each run's full
stats go to `runs/<run_id>.json`, and `summary.csv` holds the mean and standard deviation per
experiment and bot.

    gpt-pet experiment experiments/suites/s1_on_vs_off.json
    gpt-pet experiment --test '{"test_time": "2min"}' --bot experiments/bots/s1_on.json
"""
from __future__ import annotations

import asyncio
import csv
import json
import logging
import re
import socket
import statistics
import time
from collections.abc import Callable, Iterator, Mapping
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Literal

from google.adk.events.event import Event
from pydantic import BaseModel, ConfigDict, Field, field_validator

import gpt_pet
from gpt_pet.agents.models import missing_keys
from gpt_pet.agents.ollama import OllamaLlm, local_models
from gpt_pet.brain import build_brain
from gpt_pet.mcp_proxy import McpProxy
from gpt_pet.runtime import PetRuntime, TickResult, event_payloads
from gpt_pet.settings import Settings, read_ini, resolve_config_path
from gpt_pet.simlaunch import DEFAULT_SCENE, launch_mcp_server, stop_mcp_server
from gpt_pet.trace import TraceRecorder

log = logging.getLogger("gpt_pet.experiment")

# src/gpt_pet/__init__.py -> gpt_pet -> src -> python -> gpt-pet-v2
DEFAULT_RESULTS_DIR = Path(gpt_pet.__file__).resolve().parents[3] / "experiments" / "results"
RESULTS_CSV = "results.csv"
SUMMARY_CSV = "summary.csv"
PRICING_JSON = DEFAULT_RESULTS_DIR.parent / "pricing.json"
"""USD per million tokens by model id, with each price's source and date (`run_cost`)."""
BILLING_ERRORS = ("402 RESOURCE_EXHAUSTED", "prepayment credits are depleted", "insufficient_quota", "credit balance is too low")
"""How Gemini, OpenAI and Anthropic say the account has no money left (`out_of_credit`)."""
CACHE_WRITE_RATE = 1.25
"""A cache write costs this times the input price when the table gives no price of its own
(Anthropic's 5-minute cache)."""
SIM_BOOT_TIMEOUT_S = 600  # the first simulator call boots Unity (and may download the build once)

_DURATION = re.compile(r"(\d+(?:\.\d+)?)(hours?|hrs?|h|minutes?|mins?|m|seconds?|secs?|s)?")
_UNIT_SECONDS = {"h": 3600.0, "m": 60.0, "s": 1.0}

SUMMARY_METRICS = (
    "duration_s",
    "ticks",
    "ticks_failed",
    "tick_avg_s",
    "llm_calls",
    "llm_avg_ms",
    "llm_p95_ms",
    "llm_max_ms",
    "llm_reloads",
    "llm_prompt_tok_s",
    "llm_output_tok_s",
    "tokens_total",
    "tokens_input",
    "tokens_output",
    "cost_usd",
    "tool_calls",
    "drives",
    "drives_s1",
    "drives_gemini",
    "null_drive_pct",
    "passage_drives",
    "drive_m",
    "driving_pct",
    "goals_started",
    "goals_done",
    "s1_turns",
    "s1_share_pct",
    "s1_handoffs",
    "s1_decider_calls",
    "destinations_seen",
    "objects_seen",
    "passages_seen",
    "coverage_pct",
    "visited_pct",
    "known_m2",
    "trail_m",
)
EXPLORATION_TOOL = "sim_exploration_stats"
COVERAGE_EVERY_S = 30.0
"""How often a run samples the floor seen, for the coverage timeline (`coverage_row`)."""
COVERAGE_MARKS_S = (60, 120, 180, 240)
EXPLORATION_METRICS = ("coverage_pct", "visited_pct", "known_m2", "free_m2", "trail_m")


def _seconds(value: str | float | int) -> float:
    if isinstance(value, (int, float)):
        return float(value)
    text = value.strip().lower().replace(" ", "")
    parts = list(_DURATION.finditer(text))
    if not text or "".join(part.group(0) for part in parts) != text:
        raise ValueError(f"not a duration: {value!r} (try '5min', '90s' or '2m30s')")
    return sum(float(part.group(1)) * _UNIT_SECONDS[(part.group(2) or "s")[0]] for part in parts)


def parse_duration(value: str | float | int) -> float:
    """Seconds from `300`, `"300s"`, `"5min"`, `"2m30s"` or `"1h"`; a bare number is seconds."""
    seconds = _seconds(value)
    if seconds <= 0:
        raise ValueError(f"a test must last longer than 0 s: {value!r}")
    return seconds


def parse_offset(value: str | float | int) -> float:
    """Seconds into a run, as `parse_duration` reads them, where 0 (the clock's start) is allowed."""
    seconds = _seconds(value)
    if seconds < 0:
        raise ValueError(f"an event cannot happen before the run starts: {value!r}")
    return seconds


# --- configs -------------------------------------------------------------------------------


class _Config(BaseModel):
    model_config = ConfigDict(extra="forbid")


class TimedEvent(_Config):
    """A simulator tool the harness calls at a set time into a run — something happening in the
    world, like a person walking past (`sim_person_*`). Not the pet's doing: the pet never sees
    these tools, only their effect."""

    at: str | float
    """When, from the clock's start: `"60s"`, `"1m30s"` or a number of seconds (0 allowed)."""
    tool: str
    args: dict[str, Any] = Field(default_factory=dict)

    @field_validator("at")
    @classmethod
    def _is_an_offset(cls, value: str | float) -> str | float:
        parse_offset(value)
        return value

    @property
    def seconds(self) -> float:
        return parse_offset(self.at)


class TestConfig(_Config):
    """What to run: for how long, in which room, how many times, and what happens in the world."""

    __test__ = False  # not a pytest test class

    name: str = ""
    test_time: str | float = "5min"
    room_id: str = DEFAULT_SCENE
    """The AI2-THOR scene, e.g. FloorPlan_Train1_3 (a RoboTHOR apartment) or FloorPlan2 (a kitchen)."""
    repeats: int = Field(1, ge=1)
    warm_up: bool = True
    """Boot the simulator and S1's model before the clock starts, so boot time is not measured."""
    events: list[TimedEvent] = Field(default_factory=list)
    """Simulator tools called at set times during each run (`fire_events`), e.g. a person
    spawning at 10 s and walking past the pet at 60 s."""
    sim_env: dict[str, str] = Field(default_factory=dict)
    """Environment the test's world needs in its simulator, e.g. `{"AI2THOR_MCP_PERSON_PRELOAD":
    "1"}` for a person; merged under the bot's own `sim_env`, which wins on a clash."""
    trace: bool = False
    """Record each run's trace (`gpt_pet.trace`: every frame the pet saw, its actions, the robot's
    and the person's positions each second) in `runs/<run_id>.trace/`, drawn when the run ends."""

    @field_validator("test_time")
    @classmethod
    def _is_a_duration(cls, value: str | float) -> str | float:
        parse_duration(value)
        return value

    @property
    def seconds(self) -> float:
        return parse_duration(self.test_time)

    @property
    def label(self) -> str:
        return self.name or f"{self.test_time} in {self.room_id}"


class BotConfig(_Config):
    """Which brain: a settings profile plus overrides, `{section: {key: value}}` as in the ini."""

    name: str
    profile: Literal["sim"] = "sim"
    overrides: dict[str, dict[str, Any]] = Field(default_factory=dict)
    sim_env: dict[str, str] = Field(default_factory=dict)
    """Environment for this bot's simulator: ai2thor-mcp's own switches, e.g.
    `{"AI2THOR_MCP_FIXES": "replan,recovery"}` (simulator only; the real robot has none)."""


class Suite(_Config):
    """Every test crossed with every bot, each pair run `repeats` times."""

    name: str
    tests: list[TestConfig] = Field(min_length=1)
    bots: list[BotConfig] = Field(min_length=1)
    repeats: int | None = Field(None, ge=1)
    """Runs per test and bot, overriding every test's own `repeats`."""
    order: Literal["interleaved", "blocked"] = "interleaved"
    """`interleaved`: each repeat runs every bot once, the order reversing each time (A B, B A,
    A B, ...), so drift during the session (API latency, load) falls on every bot alike.
    `blocked`: all of one bot's runs, then the next bot's."""

    def runs_per_bot(self, test: TestConfig) -> int:
        return self.repeats or test.repeats

    def plan(self) -> Iterator[tuple[TestConfig, BotConfig, int]]:
        for test in self.tests:
            repeats = range(1, self.runs_per_bot(test) + 1)
            if self.order == "blocked":
                for bot in self.bots:
                    for repeat in repeats:
                        yield test, bot, repeat
                continue
            for repeat in repeats:
                for bot in self.bots if repeat % 2 else reversed(self.bots):
                    yield test, bot, repeat


def load_json(value: str | Path | Mapping[str, Any], base: Path | None = None) -> dict[str, Any]:
    """A config given as a dict, as inline JSON (`'{"test_time": "5min"}'`), or as a JSON file
    path (relative to `base` when given)."""
    if isinstance(value, Mapping):
        return dict(value)
    text = str(value).strip()
    if text.startswith("{"):
        return json.loads(text)
    path = Path(text).expanduser()
    if base is not None and not path.is_absolute():
        path = base / path
    return json.loads(path.read_text(encoding="utf-8"))


def load_suite(path: str | Path) -> Suite:
    """A suite file. Tests and bots may be inline objects or paths to their own JSON files
    (relative to the suite), and a single `"test"` may stand in for `"tests"`."""
    path = Path(path).expanduser().resolve()
    data = load_json(path)
    if "test" in data and "tests" not in data:
        data["tests"] = [data.pop("test")]
    data["tests"] = [load_json(item, path.parent) for item in data.get("tests", [])]
    data["bots"] = [load_json(item, path.parent) for item in data.get("bots", [])]
    return Suite.model_validate(data)


def build_suite(
    suite: str | Path | None = None,
    *,
    test: str | Mapping[str, Any] | None = None,
    bots: list[str | Mapping[str, Any]] = (),
    name: str = "adhoc",
    repeats: int | None = None,
    order: Literal["interleaved", "blocked"] | None = None,
) -> Suite:
    """A suite from a file, or from one test config and some bot configs; `repeats` and `order`
    override the suite's."""
    if suite is not None:
        built = load_suite(suite)
    else:
        if test is None or not bots:
            raise ValueError("give a suite file, or a test config and at least one bot config")
        built = Suite(
            name=name,
            tests=[TestConfig.model_validate(load_json(test))],
            bots=[BotConfig.model_validate(load_json(bot)) for bot in bots],
        )
    if repeats is not None:
        built = built.model_copy(update={"repeats": repeats})
    if order is not None:
        built = built.model_copy(update={"order": order})
    return built


def bot_settings(bot: BotConfig, mcp_url: str, config_dir: Path | None = None) -> Settings:
    """The bot's profile with its overrides applied, pointed at the run's own simulator."""
    raw = read_ini(resolve_config_path(bot.profile, config_dir))
    for section, values in bot.overrides.items():
        raw.setdefault(section, {}).update(values)
    raw["mcp"] = {**raw.get("mcp", {}), "transport": "streamable_http", "url": mcp_url}
    return Settings.model_validate(raw)


# --- one run -------------------------------------------------------------------------------


LAUNCH_ATTEMPTS = 3
"""A run's simulator launch is retried on another port when its port was taken meanwhile."""


def port_taken(log_path: Path) -> bool:
    """The simulator's log says its port was already in use (the free port went to someone else)."""
    try:
        return "address already in use" in log_path.read_text(encoding="utf-8", errors="replace").lower()
    except OSError:
        return False


def free_port() -> int:
    with socket.socket() as sock:
        sock.bind(("127.0.0.1", 0))
        return int(sock.getsockname()[1])


def _slug(text: str) -> str:
    return re.sub(r"[^a-z0-9]+", "-", text.lower()).strip("-") or "run"


def _pick(data: Mapping[str, Any], *keys: str) -> Any:
    for key in keys:
        if not isinstance(data, Mapping) or key not in data:
            return ""
        data = data[key]
    return "" if data is None else data


def metrics_row(snapshot: Mapping[str, Any]) -> dict[str, Any]:
    """The CSV columns for one run's stats snapshot (blank where the run produced none)."""
    ms_to_s = lambda ms: "" if ms in ("", None) else round(ms / 1000, 2)  # noqa: E731
    outcomes = _pick(snapshot, "drives", "outcomes") or {}
    escalations = _pick(snapshot, "s1", "escalations") or {}
    s1_by = _pick(snapshot, "s1", "by") or {}
    drives = _pick(snapshot, "drives", "count")
    null_drives = _pick(snapshot, "drives", "null")
    drive_calls = _pick(snapshot, "drives", "calls_by") or {}
    waits = [_pick(snapshot, "llm", key, "max_ms") for key in ("latency", "failed_latency")]
    waits = [ms for ms in waits if ms != ""]
    return {
        "ticks": _pick(snapshot, "ticks", "count"),
        "tick_avg_s": ms_to_s(_pick(snapshot, "ticks", "latency", "avg_ms")),
        "ticks_truncated": _pick(snapshot, "ticks", "truncated"),
        "ticks_failed": _pick(snapshot, "ticks", "failed"),
        "llm_calls": _pick(snapshot, "llm", "calls"),
        "llm_calls_goal_setter": _pick(snapshot, "llm", "by_agent", "goal_setter", "calls") or (0 if snapshot else ""),
        "llm_calls_executor": _pick(snapshot, "llm", "by_agent", "executor", "calls") or (0 if snapshot else ""),
        "llm_errors": _pick(snapshot, "llm", "errors"),
        "llm_fallbacks": _pick(snapshot, "llm", "fallbacks"),
        "llm_maps_shown": _pick(snapshot, "llm", "maps_shown"),
        "llm_calls_per_min": _pick(snapshot, "llm", "per_min"),
        "llm_avg_ms": _pick(snapshot, "llm", "latency", "avg_ms"),
        "llm_p50_ms": _pick(snapshot, "llm", "latency", "p50_ms"),
        "llm_p95_ms": _pick(snapshot, "llm", "latency", "p95_ms"),
        "llm_max_ms": max(waits) if waits else "",  # the longest wait for Gemini, answered or not
        "llm_reloads": _pick(snapshot, "llm", "local", "reloads"),  # local models only, from here
        "llm_load_s": _pick(snapshot, "llm", "local", "load_s"),
        "llm_prompt_s": _pick(snapshot, "llm", "local", "prompt_s"),
        "llm_output_s": _pick(snapshot, "llm", "local", "output_s"),
        "llm_prompt_tok_s": _pick(snapshot, "llm", "local", "prompt_tok_s"),
        "llm_output_tok_s": _pick(snapshot, "llm", "local", "output_tok_s"),
        "llm_max_prompt_tokens": _pick(snapshot, "llm", "local", "max_prompt_tokens"),
        "tokens_total": _pick(snapshot, "tokens", "total"),
        "tokens_input": _pick(snapshot, "tokens", "input"),
        "tokens_output": _pick(snapshot, "tokens", "output"),
        "tokens_thinking": _pick(snapshot, "tokens", "thinking"),
        "tokens_cached": _pick(snapshot, "tokens", "cached"),
        "tokens_per_min": _pick(snapshot, "tokens", "per_min"),
        "tool_calls": _pick(snapshot, "tools", "calls"),
        "tool_errors": _pick(snapshot, "tools", "errors"),
        "tool_refusals": _pick(snapshot, "tools", "refusals"),
        "tool_avg_ms": _pick(snapshot, "tools", "latency", "avg_ms"),
        "drives": drives,
        "drives_succeeded": outcomes.get("succeeded", 0) if snapshot else "",
        "drives_failed": outcomes.get("failed", 0) if snapshot else "",
        "drives_s1": drive_calls.get("s1", 0) if snapshot else "",
        "drives_gemini": drive_calls.get("gemini", 0) if snapshot else "",
        "null_drives": null_drives,
        "null_drive_pct": round(100 * null_drives / drives, 1) if drives and null_drives != "" else "",
        "passage_drives": _pick(snapshot, "drives", "to_passages"),
        "drive_m": _pick(snapshot, "drives", "distance_m"),
        "destinations_reached": _pick(snapshot, "drives", "destinations_reached"),
        "driving_s": _pick(snapshot, "drives", "driving_s"),
        "driving_pct": _pick(snapshot, "drives", "driving_pct"),
        "drive_checks": _pick(snapshot, "drives", "checks"),
        "goals_started": _pick(snapshot, "goals", "started"),
        "goals_done": _pick(snapshot, "goals", "done"),
        "goals_abandoned": _pick(snapshot, "goals", "abandoned"),
        "people_sightings": _pick(snapshot, "people", "sightings"),
        "people_watch_looks": _pick(snapshot, "people", "watch_looks"),
        "people_drive_interrupts": _pick(snapshot, "people", "drive_interrupts"),
        "people_rest_wakes": _pick(snapshot, "people", "rest_wakes"),
        "people_attended": _pick(snapshot, "people", "attended"),
        "people_escalations": _pick(snapshot, "people", "escalations"),
        "goals_done_per_min": _pick(snapshot, "goals", "done_per_min"),
        "s1_turns": _pick(snapshot, "s1", "turns"),
        "s1_share_pct": _pick(snapshot, "s1", "share_pct"),
        "s1_rules": s1_by.get("rules", 0) if snapshot else "",
        "s1_nimble": s1_by.get("nimble", 0) if snapshot else "",
        "s1_handoffs": sum(escalations.values()) if snapshot else "",
        "s1_decider_calls": _pick(snapshot, "s1", "decider", "calls"),
        "s1_decider_avg_ms": _pick(snapshot, "s1", "decider", "latency", "avg_ms"),
        "s1_decider_max_ms": _pick(snapshot, "s1", "decider", "latency", "max_ms"),
        "first_action_s": _pick(snapshot, "first_action_s"),
    }


def load_prices(path: Path = PRICING_JSON) -> dict[str, dict[str, float]]:
    """`{model id: {"input", "cached_input", "cache_write", "output"}}` in USD per million tokens."""
    if not path.is_file():
        return {}
    return dict(json.loads(path.read_text(encoding="utf-8")).get("models", {}))


def price_of(model: str, prices: Mapping[str, Mapping[str, float]]) -> Mapping[str, float] | None:
    """The model's price: its exact id, else the longest listed id it starts with (a dated
    snapshot such as `gpt-5-mini-2025-08-07` takes `gpt-5-mini`'s)."""
    if model in prices:
        return prices[model]
    listed = [name for name in prices if model.startswith(name)]
    return prices[max(listed, key=len)] if listed else None


def run_cost(by_model: Mapping[str, Mapping[str, float]], prices: Mapping[str, Mapping[str, float]]) -> tuple[float | None, list[str]]:
    """What a run's model calls cost in USD, and the models with no price (left out). Cached input
    and cache writes have their own prices; thinking is billed as output. A cost the API reported
    itself (OpenRouter) is taken as it is."""
    total, unpriced = 0.0, []
    for model, counts in by_model.items():
        if counts.get("billed_usd"):
            total += float(counts["billed_usd"])
            continue
        price = price_of(model, prices)
        if price is None:
            unpriced.append(model)
            continue
        cached, written = float(counts.get("cached", 0)), float(counts.get("cache_write", 0))
        fresh = max(0.0, float(counts.get("input", 0)) - cached - written)
        written_price = price.get("cache_write", CACHE_WRITE_RATE * price["input"])
        output = float(counts.get("output", 0)) + float(counts.get("thinking", 0))
        total += (fresh * price["input"] + cached * price.get("cached_input", price["input"]) + written * written_price + output * price["output"]) / 1e6
    return (round(total, 5) if total or not unpriced else None), unpriced


def cost_row(snapshot: Mapping[str, Any], duration_s: float, prices: Mapping[str, Mapping[str, float]]) -> dict[str, Any]:
    """The CSV columns on cost: the run's, the same per hour, and any model with no price."""
    cost, unpriced = run_cost(_pick(snapshot, "llm", "by_model") or {}, prices)
    return {
        "cost_usd": "" if cost is None else cost,
        "cost_per_hour_usd": "" if cost is None or duration_s <= 0 else round(cost * 3600 / duration_s, 3),
        "cost_unpriced": " ".join(unpriced),
    }


def world_row(destinations: list[Mapping[str, Any]] | None, exploration: Mapping[str, Any] | None) -> dict[str, Any]:
    """The CSV columns read from the simulator when the clock stops: the destinations the robot
    discovered (`list_destinations`) and its exploration score (`sim_exploration_stats`)."""
    kinds = [str(d.get("kind", "")) for d in destinations or []]
    found = destinations is not None
    score = dict(exploration or {})
    return {
        "destinations_seen": len(kinds) if found else "",
        "objects_seen": kinds.count("object") if found else "",
        "passages_seen": kinds.count("passage") if found else "",
        **{key: "" if score.get(key) is None else score[key] for key in EXPLORATION_METRICS},
    }


class DecisionLog:
    """One run's decisions as JSON lines (`runs/<run_id>.log.jsonl`): every tool call, tool
    response and text turn with who took it (Gemini, or the turn's `s1` / `fallback` tag), each
    tick's summary, and the runtime's notices. `t` counts seconds from the run's clock start."""

    def __init__(self, path: Path) -> None:
        self.path = path
        self._handle = path.open("w", encoding="utf-8")
        self._clock = time.monotonic()

    def start(self) -> None:
        self._clock = time.monotonic()

    def write(self, kind: str, record: Mapping[str, Any]) -> None:
        if self._handle.closed:  # a cancelled tick can still emit while the run shuts down
            return
        line = {"t": round(time.monotonic() - self._clock, 2), "type": kind, **record}
        self._handle.write(json.dumps(line, default=str) + "\n")
        self._handle.flush()

    def event(self, event: Event, tick: int) -> None:
        for payload in event_payloads(event, tick):
            self.write("event", payload)

    def tick(self, result: TickResult) -> None:
        self.write("tick", result.summary_dict())

    def notice(self, notice: Mapping[str, Any]) -> None:
        self.write("notice", notice)

    def close(self) -> None:
        self._handle.close()


async def sample_coverage(url: str, clock: float, timeline: list[tuple[float, float]]) -> None:
    """Every COVERAGE_EVERY_S, the floor seen so far (`sim_exploration_stats`), as (seconds, %)."""
    while True:
        await asyncio.sleep(COVERAGE_EVERY_S)
        try:
            result = await call_tool(url, EXPLORATION_TOOL)
        except Exception as exc:  # noqa: BLE001 - a missed sample must not disturb the run
            log.warning("coverage sample failed: %s", exc)
            continue
        if not result.is_error and isinstance(result.data, dict) and result.data.get("coverage_pct") is not None:
            timeline.append((round(time.monotonic() - clock, 1), float(result.data["coverage_pct"])))


def coverage_row(timeline: list[tuple[float, float]], final: Any, duration_s: float) -> dict[str, Any]:
    """The CSV columns on how fast the floor was seen: the share seen by each mark (the latest sample
    at or before it), the mean over the run (each sample standing for the time up to the next,
    starting from 0 at the clock's start), and the minutes until 80% was first seen."""
    points = sorted(timeline) + ([(duration_s, float(final))] if _is_number_like(final) else [])
    row: dict[str, Any] = {}
    for mark in COVERAGE_MARKS_S:
        before = [pct for t, pct in points if t <= mark + 1]
        row[f"coverage_{mark}s"] = before[-1] if before else ""
    if points and duration_s > 0:
        area, last_t, last_pct = 0.0, 0.0, 0.0
        for t, pct in points:
            area += (min(t, duration_s) - last_t) * (last_pct + pct) / 2
            last_t, last_pct = min(t, duration_s), pct
        row["coverage_mean_pct"] = round(area / duration_s, 1)
    else:
        row["coverage_mean_pct"] = ""
    reached = next((t for t, pct in points if pct >= 80.0), None)
    row["minutes_to_80pct"] = "" if reached is None else round(reached / 60, 2)
    return row


def _is_number_like(value: Any) -> bool:
    try:
        float(value)
    except (TypeError, ValueError):
        return False
    return True


async def call_tool(url: str, tool: str, args: Mapping[str, Any] | None = None) -> Any:
    proxy = McpProxy(url, timeout_s=SIM_BOOT_TIMEOUT_S)
    try:
        return await proxy.call(tool, dict(args or {}))
    finally:
        await proxy.close()


async def fire_events(url: str, events: list[TimedEvent], clock: float, decisions: DecisionLog) -> None:
    """Call each event's tool at its time into the run, in time order, and log what came back
    (`timed_event` lines, on the run's clock). A failing event is logged and the run goes on."""
    for event in sorted(events, key=lambda item: item.seconds):
        delay = event.seconds - (time.monotonic() - clock)
        if delay > 0:
            await asyncio.sleep(delay)
        record: dict[str, Any] = {"tool": event.tool, "args": event.args}
        try:
            result = await call_tool(url, event.tool, event.args)
        except Exception as exc:  # noqa: BLE001 - a missed event must not disturb the run
            log.warning("timed event %s failed: %s", event.tool, exc)
            decisions.write("timed_event", {**record, "ok": False, "error": repr(exc)[:300]})
            continue
        if result.is_error:
            log.warning("timed event %s failed: %s", event.tool, result.text[:200])
        record["ok"] = not result.is_error
        if isinstance(result.data, dict):
            record["result"] = result.data
        else:
            record["text"] = (result.text or "")[:300]
        decisions.write("timed_event", record)


async def world_metrics(url: str) -> dict[str, Any]:
    """`world_row` from the run's simulator; a tool that fails leaves its columns blank."""
    found: dict[str, Any] = {}
    for tool in ("list_destinations", EXPLORATION_TOOL):
        try:
            result = await call_tool(url, tool)
        except Exception:  # noqa: BLE001 - a missing metric must not lose the run
            log.exception("%s failed", tool)
            continue
        if result.is_error or result.data is None:
            log.warning("%s failed: %s", tool, result.text[:200])
            continue
        found[tool] = result.data
    listed = found.get("list_destinations")
    return world_row(None if listed is None else listed.get("destinations", []), found.get(EXPLORATION_TOOL))


async def ollama_memory(local: list[OllamaLlm]) -> list[dict[str, Any]]:
    """What Ollama holds in memory now (`ollama ps`), when the run has a local model."""
    if not local:
        return []
    try:
        return await local[0].loaded()
    except Exception as exc:  # noqa: BLE001 - a missing reading must not lose the run
        log.warning("could not read Ollama's loaded models: %s", exc)
        return []


def memory_row(local: list[OllamaLlm], start: list[dict[str, Any]], end: list[dict[str, Any]]) -> dict[str, Any]:
    """The CSV columns on memory: the run's local model as loaded (its size at the run's context,
    and the share of it on the GPU) and everything Ollama held when the clock started and stopped,
    which shows whether the models fit side by side."""
    names = {model.model for model in local}
    mine = next((item for item in start if item["name"] in names), None)

    def listing(items: list[dict[str, Any]]) -> str:
        return "; ".join(f"{item['name']} {item['size_gb']} GB ({item['gpu_gb']} GB on GPU)" for item in items)

    return {
        "ollama_size_gb": mine["size_gb"] if mine else "",
        "ollama_gpu_pct": round(100 * mine["gpu_gb"] / mine["size_gb"]) if mine and mine["size_gb"] else "",
        "ollama_loaded_start": listing(start),
        "ollama_loaded_end": listing(end),
    }


async def run_once(
    test: TestConfig, bot: BotConfig, *, experiment: str, repeat: int, results_dir: Path = DEFAULT_RESULTS_DIR
) -> dict[str, Any]:
    """One run: a fresh simulator, the bot's brain for `test_time`, a CSV row with its totals."""
    started = datetime.now(timezone.utc)
    run_id = f"{started:%Y%m%dT%H%M%SZ}-{_slug(experiment)}-{_slug(bot.name)}-{repeat}"
    url = f"http://127.0.0.1:{free_port()}/mcp"
    settings = bot_settings(bot, url)
    local = local_models(settings.model)
    runs_dir = results_dir / "runs"
    runs_dir.mkdir(parents=True, exist_ok=True)
    sim_env = {**test.sim_env, **bot.sim_env}
    meta = {
        "run_id": run_id,
        "experiment": experiment,
        "test": test.label,
        "bot": bot.name,
        "repeat": repeat,
        "started_at": started.isoformat(timespec="seconds"),
        "room_id": test.room_id,
        "test_time_s": test.seconds,
        "s1_enabled": settings.s1.enabled,
        "s1_model": ("" if settings.s1.rules_only else settings.s1.model) if settings.s1.enabled else "",
        "s1_rules_only": settings.s1.rules_only if settings.s1.enabled else "",
        "s1_min_confidence": settings.s1.min_confidence if settings.s1.enabled else "",
        "goal_setter_model": settings.model.goal_setter,
        "executor_model": settings.model.executor,
        "goal_delay_s": settings.brain.goal_delay_s,
        "request_timeout_s": settings.model.request_timeout_s,
        "reasoning_effort": settings.model.reasoning_effort,
        "image_detail": settings.model.image_detail,
        "temperature": settings.model.temperature,
        "ollama_num_ctx": settings.model.ollama_num_ctx if local else "",
        "ollama_think": settings.model.ollama_think if local else "",
        "bot_overrides": json.dumps(bot.overrides, sort_keys=True),
        "sim_env": json.dumps(sim_env, sort_keys=True) if sim_env else "",
        "events": json.dumps([event.model_dump() for event in test.events]) if test.events else "",
    }
    snapshot: dict[str, Any] = {}
    world = world_row(None, None)
    timeline: list[tuple[float, float]] = []
    memory_start: list[dict[str, Any]] = []
    memory_end: list[dict[str, Any]] = []
    elapsed, error = 0.0, ""
    process = None
    decisions = DecisionLog(runs_dir / f"{run_id}.log.jsonl")
    trace = (
        TraceRecorder(runs_dir / f"{run_id}.trace", url=url, meta={"run_id": run_id, "bot": bot.name, "test": test.label})
        if test.trace
        else None
    )

    def on_event(event: Event, tick: int) -> None:
        decisions.event(event, tick)
        if trace is not None:
            trace.event(event, tick)

    def on_tick(result: TickResult) -> None:
        decisions.tick(result)
        if trace is not None:
            trace.tick(result)

    def on_notice(notice: Mapping[str, Any]) -> None:
        decisions.notice(notice)
        if trace is not None:
            trace.notice(notice)

    try:
        for attempt in range(LAUNCH_ATTEMPTS):
            try:
                process = await asyncio.to_thread(
                    launch_mcp_server, url, scene=test.room_id, log_path=runs_dir / f"{run_id}.sim.log", env=sim_env
                )
                break
            except RuntimeError as exc:
                # another process took the free port before the simulator bound it: another port
                if attempt + 1 >= LAUNCH_ATTEMPTS or not port_taken(runs_dir / f"{run_id}.sim.log"):
                    raise
                log.warning("run %s: the simulator's port was taken (%s); trying another", run_id, exc)
                url = f"http://127.0.0.1:{free_port()}/mcp"
                settings = bot_settings(bot, url)
                if trace is not None:
                    trace.url = url
        brain = build_brain(settings)
        if trace is not None:
            brain.listen_to_views(trace.view)
        async with PetRuntime(
            brain,
            user_id="experiment",
            session_id=run_id,
            on_event=on_event,
            on_tick=on_tick,
            on_notice=on_notice,
        ) as pet:
            if test.warm_up:
                await call_tool(url, "get_current_view")  # boots Unity
                if brain.s1 is not None:
                    await brain.s1.warm_up()
                for model in local:
                    log.info("%s loaded in %.1f s", model.model, await model.warm_up())
            memory_start = await ollama_memory(local)
            pet.stats.reset()
            decisions.start()
            clock = time.monotonic()
            loop = asyncio.create_task(pet.run_loop(), name=f"experiment-{run_id}")
            sampler = asyncio.create_task(sample_coverage(url, clock, timeline), name=f"coverage-{run_id}")
            helpers = [sampler]
            if trace is not None:
                trace.start()
                helpers.append(asyncio.create_task(trace.run_sampler(), name=f"trace-{run_id}"))
            if test.events:
                helpers.append(
                    asyncio.create_task(fire_events(url, test.events, clock, decisions), name=f"events-{run_id}")
                )
            done, _ = await asyncio.wait({loop}, timeout=test.seconds)
            for helper in helpers:
                helper.cancel()
            await asyncio.gather(*helpers, return_exceptions=True)
            elapsed = time.monotonic() - clock
            snapshot = pet.stats.snapshot()
            memory_end = await ollama_memory(local)
            if loop in done and loop.exception() is not None:
                error = repr(loop.exception())
            pet.stop()
            loop.cancel()
            await asyncio.gather(loop, return_exceptions=True)
            world = await world_metrics(url)
    except Exception as exc:  # noqa: BLE001 - a failed run is recorded, and the suite goes on
        log.exception("run %s failed", run_id)
        error = error or repr(exc)
    finally:
        decisions.close()
        if trace is not None:
            trace.close()
            try:
                from gpt_pet.trace_report import render

                render(trace.directory)
            except Exception:  # noqa: BLE001 - the drawing is a nicety; the trace itself is written
                log.exception("could not draw the trace of run %s", run_id)
        try:
            await asyncio.to_thread(stop_mcp_server, process)
        except Exception:  # noqa: BLE001 - the run's row matters more than a clean stop
            log.exception("could not stop the simulator of run %s", run_id)
        for model in local:  # free the memory for the next run's model
            try:
                await model.unload()
            except Exception:  # noqa: BLE001
                log.exception("could not unload %s after run %s", model.model, run_id)
    row = {
        **meta,
        "duration_s": round(elapsed, 1),
        **metrics_row(snapshot),
        **cost_row(snapshot, elapsed, load_prices()),
        **world,
        **coverage_row(timeline, world.get("coverage_pct"), elapsed),
        **memory_row(local, memory_start, memory_end),
        "error": error,
    }
    append_row(results_dir / RESULTS_CSV, row)
    record = {"row": row, "test": test.model_dump(), "bot": bot.model_dump(), "stats": snapshot, "coverage_timeline": timeline}
    (runs_dir / f"{run_id}.json").write_text(json.dumps(record, indent=2, default=str), encoding="utf-8")
    return row


async def run_suite(
    suite: Suite, *, results_dir: Path = DEFAULT_RESULTS_DIR, on_row: Callable[[dict[str, Any]], None] | None = None
) -> list[dict[str, Any]]:
    """Every run of the suite in order; the summary is rewritten at the end. A bot whose models
    need an API key that is not set stops the suite before any simulator boots."""
    missing = {key: bot.name for bot in suite.bots for key in missing_keys(bot_settings(bot, "http://127.0.0.1:1/mcp").model)}
    if missing:
        raise RuntimeError(f"API keys not set (python/src/gpt_pet/.env): {', '.join(f'{key} for {bot}' for key, bot in missing.items())}")
    plan = list(suite.plan())
    rows: list[dict[str, Any]] = []
    for number, (test, bot, repeat) in enumerate(plan, start=1):
        log.info("run %d/%d: %s, bot %s, repeat %d", number, len(plan), test.label, bot.name, repeat)
        row = await run_once(test, bot, experiment=suite.name, repeat=repeat, results_dir=results_dir)
        rows.append(row)
        if on_row is not None:
            on_row(row)
        if out_of_credit(row.get("error", "")):
            log.error("stopping the suite after run %d/%d: a provider's balance is exhausted (%s)", number, len(plan), row["error"][:200])
            break
    write_summary(results_dir)
    return rows


def out_of_credit(error: str) -> bool:
    """A run that failed because a provider's prepaid balance or quota is used up: every run after it
    would fail the same way until someone tops it up."""
    return any(sign in error for sign in BILLING_ERRORS)


# --- results -------------------------------------------------------------------------------


def read_rows(path: Path) -> list[dict[str, str]]:
    if not path.is_file():
        return []
    with path.open(newline="", encoding="utf-8") as handle:
        return list(csv.DictReader(handle))


def append_row(path: Path, row: Mapping[str, Any]) -> None:
    """Append one row; when the columns changed, rewrite the file with the union of columns."""
    path.parent.mkdir(parents=True, exist_ok=True)
    fields = list(row)
    if path.is_file():
        with path.open(newline="", encoding="utf-8") as handle:
            existing = next(csv.reader(handle), [])
        if existing == fields:
            with path.open("a", newline="", encoding="utf-8") as handle:
                csv.DictWriter(handle, fieldnames=fields).writerow(row)
            return
        fields = existing + [field for field in fields if field not in existing]
    rows = [*read_rows(path), dict(row)]
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields, restval="")
        writer.writeheader()
        writer.writerows(rows)


def summarize(path: Path) -> list[dict[str, Any]]:
    """Mean and standard deviation of the headline metrics per experiment and bot, over the runs
    that finished without an error."""
    groups: dict[tuple[str, str], list[dict[str, str]]] = {}
    for row in read_rows(path):
        if not row.get("error"):
            groups.setdefault((row["experiment"], row["bot"]), []).append(row)
    summary = []
    for (experiment, bot), runs in groups.items():
        entry: dict[str, Any] = {"experiment": experiment, "bot": bot, "runs": len(runs)}
        for metric in SUMMARY_METRICS:
            values = [float(run[metric]) for run in runs if run.get(metric) not in (None, "")]
            entry[f"{metric}_mean"] = round(statistics.mean(values), 2) if values else ""
            entry[f"{metric}_std"] = round(statistics.stdev(values), 2) if len(values) > 1 else ""
        summary.append(entry)
    return summary


def write_summary(results_dir: Path) -> list[dict[str, Any]]:
    summary = summarize(results_dir / RESULTS_CSV)
    if summary:
        with (results_dir / SUMMARY_CSV).open("w", newline="", encoding="utf-8") as handle:
            writer = csv.DictWriter(handle, fieldnames=list(summary[0]))
            writer.writeheader()
            writer.writerows(summary)
    return summary


def format_summary(summary: list[dict[str, Any]], experiment: str | None = None) -> str:
    """A metrics-by-bot table ("mean ± std") for one experiment, or for all of them."""
    entries = [e for e in summary if experiment is None or e["experiment"] == experiment]
    if not entries:
        return "no finished runs yet"
    columns = [f"{e['bot']} (n={e['runs']})" if experiment else f"{e['experiment']}/{e['bot']} (n={e['runs']})" for e in entries]
    width = max(len(metric) for metric in SUMMARY_METRICS)
    cells = []
    for metric in SUMMARY_METRICS:
        row = []
        for entry in entries:
            mean, std = entry[f"{metric}_mean"], entry[f"{metric}_std"]
            row.append("-" if mean == "" else f"{mean:g}" + ("" if std == "" else f" ± {std:g}"))
        cells.append(row)
    col_width = [max(len(columns[i]), *(len(row[i]) for row in cells)) for i in range(len(columns))]
    lines = [" " * width + "  " + "  ".join(c.rjust(w) for c, w in zip(columns, col_width))]
    for metric, row in zip(SUMMARY_METRICS, cells):
        lines.append(metric.ljust(width) + "  " + "  ".join(c.rjust(w) for c, w in zip(row, col_width)))
    return "\n".join(lines)
