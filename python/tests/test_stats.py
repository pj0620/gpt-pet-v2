"""Offline: the run statistics as plain accounting on explicit timestamps."""
from __future__ import annotations

from google.genai import types

from gpt_pet.navwait import DriveWait
from gpt_pet.runtime import TickResult
from gpt_pet.stats import RunStats, percentile, usage_tokens


def usage(prompt: int, output: int, thinking: int = 0) -> types.GenerateContentResponseUsageMetadata:
    return types.GenerateContentResponseUsageMetadata(
        prompt_token_count=prompt,
        candidates_token_count=output,
        thoughts_token_count=thinking,
        total_token_count=prompt + output + thinking,
    )


def tick(number: int, current: dict | None, history: list[dict]) -> TickResult:
    state = {"tick": number, "current_goal": current, "goal_history": history}
    return TickResult(number=number, seconds=10.0, events=[], state=state)


def test_percentiles_use_the_nearest_rank() -> None:
    assert percentile([1.0, 2.0, 3.0, 4.0], 50) == 2.0
    assert percentile([1.0, 2.0, 3.0, 4.0], 95) == 4.0
    assert percentile([7.0], 50) == 7.0


def test_missing_usage_fields_count_as_zero() -> None:
    assert usage_tokens(None) == {"input": 0, "output": 0, "thinking": 0, "cached": 0, "total": 0}
    partial = types.GenerateContentResponseUsageMetadata(prompt_token_count=10, candidates_token_count=5)
    assert usage_tokens(partial)["total"] == 15


def test_llm_calls_tokens_and_latency_accumulate() -> None:
    stats = RunStats(started=100.0, wall_started=1_000.0)
    stats.record_llm_call("goal_setter", 1.0, usage(1000, 50, thinking=200))
    stats.record_llm_call("executor", 3.0, usage(2000, 20))
    snapshot = stats.snapshot(now=160.0)  # one minute in
    assert snapshot["started_at"] == 1_000.0 and snapshot["uptime_s"] == 60.0
    llm = snapshot["llm"]
    assert llm["calls"] == 2 and llm["per_min"] == 2.0 and llm["errors"] == 0
    assert llm["latency"] == {"count": 2, "avg_ms": 2000, "p50_ms": 1000, "p95_ms": 3000, "last_ms": 3000}
    assert llm["by_agent"] == {
        "executor": {"calls": 1, "tokens": 2020, "avg_ms": 3000},
        "goal_setter": {"calls": 1, "tokens": 1250, "avg_ms": 1000},
    }
    assert snapshot["tokens"] == {
        "input": 3000,
        "output": 70,
        "thinking": 200,
        "cached": 0,
        "total": 3270,
        "per_min": 3270.0,
        "last_call": 2020,
    }


def test_tools_split_errors_and_refusals_and_time_the_first_action() -> None:
    stats = RunStats(started=100.0)
    stats.record_tool_call("get_current_view", 0.5, now=101.0)
    stats.record_tool_call("do_rotate", 0.0, refused=True, now=102.0)
    stats.record_tool_call("set_nav_goal", 0.2, error=True, now=103.0)
    stats.record_tool_call("set_nav_goal", 0.3, now=104.5)
    snapshot = stats.snapshot(now=110.0)
    tools = snapshot["tools"]
    assert (tools["calls"], tools["errors"], tools["refusals"]) == (4, 1, 1)
    assert tools["latency"]["count"] == 3  # a refused call never ran
    assert snapshot["first_action_s"] == 4.5  # the first actuation that actually ran


def test_drives_count_outcomes_checks_and_share_of_the_time() -> None:
    stats = RunStats(started=0.0)
    stats.record_drive(DriveWait("succeeded", 12.0, None, checks=24))
    stats.record_drive(DriveWait("failed", 3.0, None, checks=6))
    drives = stats.snapshot(now=60.0)["drives"]
    assert drives["count"] == 2 and drives["outcomes"] == {"failed": 1, "succeeded": 1}
    assert (drives["driving_s"], drives["driving_pct"], drives["checks"]) == (15.0, 25.0, 30)


def test_goals_are_counted_once_from_the_tick_state() -> None:
    stats = RunStats(started=0.0)
    first = {"id": 1, "status": "active", "created_tick": 1}
    second = {"id": 2, "status": "active", "created_tick": 3}
    stats.record_tick(tick(1, first, []))
    stats.record_tick(tick(2, first, []))  # a continuation starts nothing
    stats.record_tick(tick(3, second, [{**first, "status": "done"}]))
    rest = {"id": 0, "status": "limit"}
    stats.record_tick(tick(4, rest, [{**second, "status": "abandoned"}, {**first, "status": "done"}]))
    snapshot = stats.snapshot(now=120.0)
    assert snapshot["goals"] == {"started": 2, "done": 1, "abandoned": 1, "started_per_min": 1.0, "done_per_min": 0.5}
    assert snapshot["ticks"]["count"] == 4 and snapshot["ticks"]["latency"]["avg_ms"] == 10_000


def test_reset_zeroes_the_counters_without_recounting_old_goals() -> None:
    stats = RunStats(started=0.0)
    finished = tick(2, None, [{"id": 1, "status": "done"}])
    stats.record_tick(finished)
    stats.record_llm_call("executor", 1.0, usage(10, 1))
    stats.reset(started=50.0)
    stats.record_tick(finished)
    snapshot = stats.snapshot(now=110.0)
    assert snapshot["uptime_s"] == 60.0
    assert snapshot["goals"]["done"] == 0 and snapshot["llm"]["calls"] == 0 and snapshot["tokens"]["total"] == 0


def test_s1_reads_off_until_enabled_then_counts_its_turns_and_hand_offs() -> None:
    stats = RunStats(started=0.0)
    s1 = stats.snapshot(now=1.0)["s1"]
    assert (s1["enabled"], s1["model"], s1["turns"], s1["share_pct"]) == (False, None, 0, 0.0)

    stats.s1_model = "nimble"
    stats.record_s1_turn("executor", "rules")
    stats.record_s1_turn("executor", "nimble")
    stats.record_s1_turn("goal_setter", "nimble")
    stats.record_s1_escalation("executor", "unsure")
    stats.record_decider_call(0.25)
    stats.record_llm_call("executor", 2.0, usage(100, 10))
    s1 = stats.snapshot(now=2.0)["s1"]
    assert s1["enabled"] is True and s1["model"] == "nimble"
    assert s1["turns"] == 3 and s1["by"] == {"nimble": 2, "rules": 1}
    assert s1["by_agent"] == {"executor": 2, "goal_setter": 1}
    assert s1["share_pct"] == 75.0  # three of the four model turns skipped Gemini
    assert s1["escalations"] == {"executor: unsure": 1}
    assert s1["decider"]["calls"] == 1 and s1["decider"]["latency"]["last_ms"] == 250

    stats.reset()
    after = stats.snapshot()["s1"]
    assert after["enabled"] is True and after["turns"] == 0  # configuration survives a reset


def test_every_change_notifies_and_a_broken_listener_is_contained() -> None:
    stats = RunStats(started=0.0)
    changes: list[str] = []
    stats.on_change = lambda: changes.append("changed")
    stats.record_llm_error()
    stats.record_tool_call("get_map", 0.1)
    stats.reset()
    assert changes == ["changed"] * 3

    def broken() -> None:
        raise RuntimeError("listener bug")

    stats.on_change = broken
    stats.record_llm_error()
    assert stats.llm_errors == 1
