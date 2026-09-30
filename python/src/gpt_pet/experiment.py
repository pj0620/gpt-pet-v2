"""Experiments: run the pet for a fixed time under a test config and a bot config, and record its
totals as one CSV row per run.

A test config says what to run (`test_time`, `room_id`, `repeats`); a bot config says which brain
(a settings profile plus overrides of any ini section, e.g. `{"s1": {"enabled": false}}`). A suite
crosses its tests with its bots and runs each pair `repeats` times, bots in order. Every run gets
a fresh simulator with the requested room on a free port, so runs start from the same state and
never touch another simulator; the clock starts once the simulator and S1's model have booted.

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

from pydantic import BaseModel, ConfigDict, Field, field_validator

import gpt_pet
from gpt_pet.brain import build_brain
from gpt_pet.mcp_proxy import McpProxy
from gpt_pet.runtime import PetRuntime
from gpt_pet.settings import Settings, read_ini, resolve_config_path
from gpt_pet.simlaunch import DEFAULT_SCENE, launch_mcp_server, stop_mcp_server

log = logging.getLogger("gpt_pet.experiment")

# src/gpt_pet/__init__.py -> gpt_pet -> src -> python -> gpt-pet-v2
DEFAULT_RESULTS_DIR = Path(gpt_pet.__file__).resolve().parents[3] / "experiments" / "results"
RESULTS_CSV = "results.csv"
SUMMARY_CSV = "summary.csv"
SIM_BOOT_TIMEOUT_S = 600  # the first simulator call boots Unity (and may download the build once)

_DURATION = re.compile(r"(\d+(?:\.\d+)?)(hours?|hrs?|h|minutes?|mins?|m|seconds?|secs?|s)?")
_UNIT_SECONDS = {"h": 3600.0, "m": 60.0, "s": 1.0}

SUMMARY_METRICS = (
    "duration_s",
    "ticks",
    "tick_avg_s",
    "llm_calls",
    "llm_avg_ms",
    "tokens_total",
    "tokens_input",
    "tokens_output",
    "tool_calls",
    "drives",
    "driving_pct",
    "goals_started",
    "goals_done",
    "s1_turns",
    "s1_share_pct",
    "s1_handoffs",
    "s1_decider_calls",
    "destinations_seen",
)


def parse_duration(value: str | float | int) -> float:
    """Seconds from `300`, `"300s"`, `"5min"`, `"2m30s"` or `"1h"`; a bare number is seconds."""
    if isinstance(value, (int, float)):
        seconds = float(value)
    else:
        text = value.strip().lower().replace(" ", "")
        parts = list(_DURATION.finditer(text))
        if not text or "".join(part.group(0) for part in parts) != text:
            raise ValueError(f"not a duration: {value!r} (try '5min', '90s' or '2m30s')")
        seconds = sum(float(part.group(1)) * _UNIT_SECONDS[(part.group(2) or "s")[0]] for part in parts)
    if seconds <= 0:
        raise ValueError(f"a test must last longer than 0 s: {value!r}")
    return seconds


# --- configs -------------------------------------------------------------------------------


class _Config(BaseModel):
    model_config = ConfigDict(extra="forbid")


class TestConfig(_Config):
    """What to run: for how long, in which room, how many times."""

    __test__ = False  # not a pytest test class

    name: str = ""
    test_time: str | float = "5min"
    room_id: str = DEFAULT_SCENE
    """The AI2-THOR scene, e.g. FloorPlan_Train1_3 (a RoboTHOR apartment) or FloorPlan2 (a kitchen)."""
    repeats: int = Field(1, ge=1)
    warm_up: bool = True
    """Boot the simulator and S1's model before the clock starts, so boot time is not measured."""

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


class Suite(_Config):
    """Every test crossed with every bot, each pair run `repeats` times, bots in order."""

    name: str
    tests: list[TestConfig] = Field(min_length=1)
    bots: list[BotConfig] = Field(min_length=1)

    def plan(self) -> Iterator[tuple[TestConfig, BotConfig, int]]:
        for test in self.tests:
            for bot in self.bots:
                for repeat in range(1, test.repeats + 1):
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
) -> Suite:
    """A suite from a file, or from one test config and some bot configs; `repeats` overrides."""
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
        built = built.model_copy(update={"tests": [t.model_copy(update={"repeats": repeats}) for t in built.tests]})
    return built


def bot_settings(bot: BotConfig, mcp_url: str, config_dir: Path | None = None) -> Settings:
    """The bot's profile with its overrides applied, pointed at the run's own simulator."""
    raw = read_ini(resolve_config_path(bot.profile, config_dir))
    for section, values in bot.overrides.items():
        raw.setdefault(section, {}).update(values)
    raw["mcp"] = {**raw.get("mcp", {}), "transport": "streamable_http", "url": mcp_url}
    return Settings.model_validate(raw)


# --- one run -------------------------------------------------------------------------------


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
    return {
        "ticks": _pick(snapshot, "ticks", "count"),
        "tick_avg_s": ms_to_s(_pick(snapshot, "ticks", "latency", "avg_ms")),
        "ticks_truncated": _pick(snapshot, "ticks", "truncated"),
        "llm_calls": _pick(snapshot, "llm", "calls"),
        "llm_calls_goal_setter": _pick(snapshot, "llm", "by_agent", "goal_setter", "calls") or (0 if snapshot else ""),
        "llm_calls_executor": _pick(snapshot, "llm", "by_agent", "executor", "calls") or (0 if snapshot else ""),
        "llm_errors": _pick(snapshot, "llm", "errors"),
        "llm_calls_per_min": _pick(snapshot, "llm", "per_min"),
        "llm_avg_ms": _pick(snapshot, "llm", "latency", "avg_ms"),
        "llm_p50_ms": _pick(snapshot, "llm", "latency", "p50_ms"),
        "llm_p95_ms": _pick(snapshot, "llm", "latency", "p95_ms"),
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
        "drives": _pick(snapshot, "drives", "count"),
        "drives_succeeded": outcomes.get("succeeded", 0) if snapshot else "",
        "drives_failed": outcomes.get("failed", 0) if snapshot else "",
        "driving_s": _pick(snapshot, "drives", "driving_s"),
        "driving_pct": _pick(snapshot, "drives", "driving_pct"),
        "drive_checks": _pick(snapshot, "drives", "checks"),
        "goals_started": _pick(snapshot, "goals", "started"),
        "goals_done": _pick(snapshot, "goals", "done"),
        "goals_abandoned": _pick(snapshot, "goals", "abandoned"),
        "goals_done_per_min": _pick(snapshot, "goals", "done_per_min"),
        "s1_turns": _pick(snapshot, "s1", "turns"),
        "s1_share_pct": _pick(snapshot, "s1", "share_pct"),
        "s1_rules": s1_by.get("rules", 0) if snapshot else "",
        "s1_nimble": s1_by.get("nimble", 0) if snapshot else "",
        "s1_handoffs": sum(escalations.values()) if snapshot else "",
        "s1_decider_calls": _pick(snapshot, "s1", "decider", "calls"),
        "s1_decider_avg_ms": _pick(snapshot, "s1", "decider", "latency", "avg_ms"),
        "first_action_s": _pick(snapshot, "first_action_s"),
    }


async def call_tool(url: str, tool: str) -> Any:
    proxy = McpProxy(url, timeout_s=SIM_BOOT_TIMEOUT_S)
    try:
        return await proxy.call(tool, {})
    finally:
        await proxy.close()


async def run_once(
    test: TestConfig, bot: BotConfig, *, experiment: str, repeat: int, results_dir: Path = DEFAULT_RESULTS_DIR
) -> dict[str, Any]:
    """One run: a fresh simulator, the bot's brain for `test_time`, a CSV row with its totals."""
    started = datetime.now(timezone.utc)
    run_id = f"{started:%Y%m%dT%H%M%SZ}-{_slug(experiment)}-{_slug(bot.name)}-{repeat}"
    url = f"http://127.0.0.1:{free_port()}/mcp"
    settings = bot_settings(bot, url)
    runs_dir = results_dir / "runs"
    runs_dir.mkdir(parents=True, exist_ok=True)
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
        "s1_model": settings.s1.model if settings.s1.enabled else "",
        "s1_min_confidence": settings.s1.min_confidence if settings.s1.enabled else "",
        "goal_setter_model": settings.model.goal_setter,
        "executor_model": settings.model.executor,
        "goal_delay_s": settings.brain.goal_delay_s,
        "bot_overrides": json.dumps(bot.overrides, sort_keys=True),
    }
    snapshot: dict[str, Any] = {}
    elapsed, destinations, error = 0.0, "", ""
    process = None
    try:
        process = await asyncio.to_thread(launch_mcp_server, url, scene=test.room_id, log_path=runs_dir / f"{run_id}.sim.log")
        brain = build_brain(settings)
        async with PetRuntime(brain, user_id="experiment", session_id=run_id) as pet:
            if test.warm_up:
                await call_tool(url, "get_current_view")  # boots Unity
                if brain.s1 is not None:
                    await brain.s1.warm_up()
            pet.stats.reset()
            clock = time.monotonic()
            loop = asyncio.create_task(pet.run_loop(), name=f"experiment-{run_id}")
            done, _ = await asyncio.wait({loop}, timeout=test.seconds)
            elapsed = time.monotonic() - clock
            snapshot = pet.stats.snapshot()
            if loop in done and loop.exception() is not None:
                error = repr(loop.exception())
            pet.stop()
            loop.cancel()
            await asyncio.gather(loop, return_exceptions=True)
        destinations = len((await call_tool(url, "list_destinations")).data.get("destinations", []))
    except Exception as exc:  # noqa: BLE001 - a failed run is recorded, and the suite goes on
        log.exception("run %s failed", run_id)
        error = error or repr(exc)
    finally:
        await asyncio.to_thread(stop_mcp_server, process)
    row = {**meta, "duration_s": round(elapsed, 1), **metrics_row(snapshot), "destinations_seen": destinations, "error": error}
    append_row(results_dir / RESULTS_CSV, row)
    record = {"row": row, "test": test.model_dump(), "bot": bot.model_dump(), "stats": snapshot}
    (runs_dir / f"{run_id}.json").write_text(json.dumps(record, indent=2, default=str), encoding="utf-8")
    return row


async def run_suite(
    suite: Suite, *, results_dir: Path = DEFAULT_RESULTS_DIR, on_row: Callable[[dict[str, Any]], None] | None = None
) -> list[dict[str, Any]]:
    """Every run of the suite in order; the summary is rewritten at the end."""
    plan = list(suite.plan())
    rows: list[dict[str, Any]] = []
    for number, (test, bot, repeat) in enumerate(plan, start=1):
        log.info("run %d/%d: %s, bot %s, repeat %d", number, len(plan), test.label, bot.name, repeat)
        row = await run_once(test, bot, experiment=suite.name, repeat=repeat, results_dir=results_dir)
        rows.append(row)
        if on_row is not None:
            on_row(row)
    write_summary(results_dir)
    return rows


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
