"""Offline: the experiment harness's configs, settings overrides, CSV rows and summaries."""
from __future__ import annotations

import json
from pathlib import Path

import pytest
from google.genai import types
from pydantic import ValidationError

from gpt_pet.experiment import (
    BotConfig,
    TestConfig,
    append_row,
    bot_settings,
    build_suite,
    format_summary,
    load_suite,
    metrics_row,
    parse_duration,
    read_rows,
    summarize,
)
from gpt_pet.navwait import DriveWait
from gpt_pet.stats import RunStats

EXPERIMENTS = Path(__file__).resolve().parents[2] / "experiments"


@pytest.mark.parametrize(
    ("text", "seconds"),
    [("5min", 300), ("300", 300), ("90s", 90), ("2m30s", 150), ("1h", 3600), ("1.5 min", 90), (45, 45), ("10 minutes", 600)],
)
def test_durations_read_the_way_people_write_them(text: str | int, seconds: float) -> None:
    assert parse_duration(text) == seconds


@pytest.mark.parametrize("bad", ["", "soon", "5 parsecs", "0s", -3, "5min!"])
def test_other_durations_are_rejected(bad: str | int) -> None:
    with pytest.raises(ValueError):
        parse_duration(bad)


def test_the_shipped_suite_runs_s1_off_three_times_then_s1_on_three_times() -> None:
    suite = load_suite(EXPERIMENTS / "suites" / "s1_on_vs_off.json")
    plan = [(test.seconds, bot.name, repeat) for test, bot, repeat in suite.plan()]
    assert plan == [(300, "s1_off", 1), (300, "s1_off", 2), (300, "s1_off", 3), (300, "s1_on", 1), (300, "s1_on", 2), (300, "s1_on", 3)]
    assert suite.tests[0].room_id == "FloorPlan_Train1_3"
    assert [bot.overrides["s1"]["enabled"] for bot in suite.bots] == [False, True]


def test_bot_overrides_change_the_profile_and_every_run_has_its_own_simulator() -> None:
    bot = BotConfig(name="s1_off", overrides={"s1": {"enabled": False}, "brain": {"goal_delay_s": 0}, "tool_limits": {"do_rotate": 4}})
    settings = bot_settings(bot, "http://127.0.0.1:9123/mcp")
    assert settings.s1.enabled is False and settings.brain.goal_delay_s == 0 and settings.tool_limits["do_rotate"] == 4
    assert str(settings.mcp.url) == "http://127.0.0.1:9123/mcp"
    assert settings.tool_limits["get_current_view"] == 4  # keys the bot leaves alone keep the profile's value
    with pytest.raises(ValidationError, match="bogus"):
        bot_settings(BotConfig(name="typo", overrides={"s1": {"bogus": 1}}), "http://127.0.0.1:9123/mcp")


def test_configs_come_inline_from_files_or_from_suites_that_reference_them(tmp_path: Path) -> None:
    (tmp_path / "bot.json").write_text(json.dumps({"name": "file-bot", "overrides": {"s1": {"enabled": True}}}))
    suite = build_suite(test='{"test_time": "90s"}', bots=[str(tmp_path / "bot.json"), {"name": "inline-bot"}], name="t", repeats=2)
    assert [bot.name for bot in suite.bots] == ["file-bot", "inline-bot"] and suite.tests[0].repeats == 2
    (tmp_path / "suite.json").write_text(json.dumps({"name": "s", "test": {"test_time": "1min"}, "bots": ["bot.json"]}))
    assert load_suite(tmp_path / "suite.json").bots[0].overrides == {"s1": {"enabled": True}}
    with pytest.raises(ValidationError):
        TestConfig(test_time="soon")
    with pytest.raises(ValidationError):
        TestConfig(test_time="5min", room="FloorPlan1")  # a typo for room_id fails loudly
    with pytest.raises(ValueError):
        build_suite(test='{"test_time": "90s"}')  # no bot


def test_a_stats_snapshot_becomes_one_flat_row() -> None:
    stats = RunStats(started=0.0)
    stats.s1_model = "nimble"
    usage = types.GenerateContentResponseUsageMetadata(prompt_token_count=900, candidates_token_count=40, total_token_count=940)
    stats.record_llm_call("executor", 2.0, usage)
    stats.record_s1_turn("executor", "rules")
    stats.record_s1_escalation("executor", "unsure")
    stats.record_drive(DriveWait("succeeded", 12.0, None, checks=24))
    row = metrics_row(stats.snapshot(now=300.0))
    assert (row["llm_calls"], row["llm_calls_executor"], row["llm_calls_goal_setter"]) == (1, 1, 0)
    assert (row["tokens_total"], row["tokens_input"], row["llm_avg_ms"]) == (940, 900, 2000)
    assert (row["drives"], row["drives_succeeded"], row["driving_pct"], row["drive_checks"]) == (1, 1, 4.0, 24)
    assert (row["s1_turns"], row["s1_rules"], row["s1_handoffs"], row["s1_share_pct"]) == (1, 1, 1, 50.0)
    blank = metrics_row({})
    assert set(blank) == set(row) and all(value == "" for value in blank.values())


def test_rows_append_and_new_columns_widen_the_csv(tmp_path: Path) -> None:
    path = tmp_path / "results.csv"
    append_row(path, {"run_id": "a", "bot": "x", "llm_calls": 3})
    append_row(path, {"run_id": "b", "bot": "x", "llm_calls": 5})
    append_row(path, {"run_id": "c", "bot": "y", "llm_calls": 1, "new_metric": 7})
    rows = read_rows(path)
    assert [row["run_id"] for row in rows] == ["a", "b", "c"]
    assert rows[0]["new_metric"] == "" and rows[2]["new_metric"] == "7"


def test_the_summary_is_mean_and_std_per_bot_over_runs_without_errors(tmp_path: Path) -> None:
    path = tmp_path / "results.csv"
    for run_id, bot, calls, error in [("1", "off", 10, ""), ("2", "off", 14, ""), ("3", "on", 4, ""), ("4", "on", 99, "boom")]:
        append_row(path, {"run_id": run_id, "experiment": "e", "bot": bot, "llm_calls": calls, "error": error})
    summary = {entry["bot"]: entry for entry in summarize(path)}
    assert (summary["off"]["runs"], summary["off"]["llm_calls_mean"], summary["off"]["llm_calls_std"]) == (2, 12, 2.83)
    assert (summary["on"]["runs"], summary["on"]["llm_calls_mean"], summary["on"]["llm_calls_std"]) == (1, 4, "")
    table = format_summary(summarize(path), experiment="e")
    assert "off (n=2)" in table and "12 ± 2.83" in table
