"""Offline: the experiment harness's configs, settings overrides, CSV rows and summaries."""
from __future__ import annotations

import json
from pathlib import Path

import pytest
from google.adk.events.event import Event
from google.genai import types
from pydantic import ValidationError

from gpt_pet.experiment import (
    BotConfig,
    DecisionLog,
    Suite,
    TestConfig,
    TimedEvent,
    append_row,
    bot_settings,
    build_suite,
    format_summary,
    load_suite,
    metrics_row,
    parse_duration,
    read_rows,
    summarize,
    world_row,
)
from gpt_pet.navwait import DriveWait
from gpt_pet.runtime import TickResult
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


def test_the_shipped_suite_alternates_s1_off_and_on_three_times_each() -> None:
    suite = load_suite(EXPERIMENTS / "suites" / "s1_on_vs_off.json")
    plan = [(test.seconds, bot.name, repeat) for test, bot, repeat in suite.plan()]
    assert plan == [(300, "s1_off", 1), (300, "s1_on", 1), (300, "s1_on", 2), (300, "s1_off", 2), (300, "s1_off", 3), (300, "s1_on", 3)]
    assert suite.tests[0].room_id == "FloorPlan_Train1_3"
    assert [bot.overrides["s1"]["enabled"] for bot in suite.bots] == [False, True]


def test_the_baseline_suite_runs_each_bot_four_times_interleaved() -> None:
    suite = load_suite(EXPERIMENTS / "suites" / "baseline.json")
    plan = [(bot.name, repeat) for _, bot, repeat in suite.plan()]
    assert [name for name, _ in plan] == ["s1_off", "s1_on", "s1_on", "s1_off"] * 2
    assert sorted(plan) == [(name, n) for name in ("s1_off", "s1_on") for n in range(1, 5)]
    assert suite.tests[0].repeats == 3 and suite.runs_per_bot(suite.tests[0]) == 4  # the suite overrides the test


def test_every_shipped_bot_is_a_valid_profile_override() -> None:
    for path in sorted((EXPERIMENTS / "bots").glob("*.json")):
        bot = BotConfig.model_validate(json.loads(path.read_text()))
        settings = bot_settings(bot, "http://127.0.0.1:9123/mcp")  # strict settings: a typo fails here
        assert settings.brain.goal_delay_s == 0, path.name
    for path in sorted((EXPERIMENTS / "suites").glob("*.json")):
        assert list(load_suite(path).plan()), path.name
    exploration = load_suite(EXPERIMENTS / "suites" / "exploration.json")
    assert len(list(exploration.plan())) == 24  # 6 bots x 4 runs
    p2 = {bot.name: bot_settings(bot, "http://127.0.0.1:9123/mcp") for bot in exploration.bots if bot.name.endswith("_p2")}
    assert [(s.brain.map_for_goals, s.s1.explore_policy) for s in p2.values()] == [(True, False), (True, True)]
    p1 = {bot.name: bot_settings(bot, "http://127.0.0.1:9123/mcp") for bot in load_suite(EXPERIMENTS / "suites" / "p1.json").bots}
    assert list(p1) == ["s1_off", "s1_off_p1", "s1_on", "s1_on_p1"]
    assert [(s.s1.enabled, s.brain.places_memory, s.s1.scan_rule) for s in p1.values()] == [
        (False, False, False),
        (False, True, False),
        (True, False, False),
        (True, True, True),
    ]


def test_blocked_order_runs_one_bot_at_a_time_and_interleaving_reverses_each_repeat() -> None:
    bots = [BotConfig(name=name) for name in "abc"]
    blocked = Suite(name="s", tests=[TestConfig(repeats=2)], bots=bots, order="blocked")
    assert [(bot.name, repeat) for _, bot, repeat in blocked.plan()] == [("a", 1), ("a", 2), ("b", 1), ("b", 2), ("c", 1), ("c", 2)]
    interleaved = Suite(name="s", tests=[TestConfig(repeats=3)], bots=bots)
    assert "".join(bot.name for _, bot, _ in interleaved.plan()) == "abccbaabc"
    with pytest.raises(ValidationError):
        Suite(name="s", tests=[TestConfig()], bots=bots, order="random")
    shipped = build_suite(EXPERIMENTS / "suites" / "s1_on_vs_off.json", order="blocked", repeats=2)
    assert [(bot.name, repeat) for _, bot, repeat in shipped.plan()] == [("s1_off", 1), ("s1_off", 2), ("s1_on", 1), ("s1_on", 2)]


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
    assert [bot.name for bot in suite.bots] == ["file-bot", "inline-bot"] and suite.runs_per_bot(suite.tests[0]) == 2
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
    stats.record_llm_error(20.1)
    stats.record_llm_fallback()
    stats.record_s1_turn("executor", "rules")
    stats.record_s1_escalation("executor", "unsure")
    stats.record_drive(DriveWait("succeeded", 12.0, None, checks=24, destination_id="psg:1", planned_m=3.0))
    stats.record_drive(DriveWait("succeeded", 1.0, None, checks=2, destination_id="obj:Chair|1", planned_m=0.1))
    stats.record_drive_call("s1")
    stats.record_drive_call("gemini")
    stats.record_tick(TickResult(number=1, seconds=20.3, events=[], state={}, error="TimeoutError"))
    row = metrics_row(stats.snapshot(now=300.0))
    assert (row["llm_calls"], row["llm_calls_executor"], row["llm_calls_goal_setter"]) == (1, 1, 0)
    assert (row["tokens_total"], row["tokens_input"], row["llm_avg_ms"]) == (940, 900, 2000)
    assert (row["llm_errors"], row["llm_fallbacks"], row["llm_max_ms"]) == (1, 1, 20_100)  # the failed wait counts
    assert (row["ticks"], row["ticks_failed"]) == (1, 1)
    assert (row["drives"], row["drives_succeeded"], row["driving_pct"], row["drive_checks"]) == (2, 2, 4.3, 26)
    assert (row["null_drives"], row["null_drive_pct"], row["passage_drives"]) == (1, 50.0, 1)
    assert (row["drives_s1"], row["drives_gemini"]) == (1, 1)
    assert (row["s1_turns"], row["s1_rules"], row["s1_handoffs"], row["s1_share_pct"]) == (1, 1, 1, 50.0)
    blank = metrics_row({})
    assert set(blank) == set(row) and all(value == "" for value in blank.values())


def test_what_the_robot_discovered_and_covered_becomes_columns() -> None:
    destinations = [{"id": "obj:Sofa|1", "kind": "object"}, {"id": "obj:TV|2", "kind": "object"}, {"id": "psg:1", "kind": "passage"}]
    score = {"coverage_pct": 41.5, "visited_pct": 12.0, "known_m2": 9.4, "free_m2": 6.1, "trail_m": 7.25, "reachable_points": 230}
    row = world_row(destinations, score)
    assert row == {
        "destinations_seen": 3,
        "objects_seen": 2,
        "passages_seen": 1,
        "coverage_pct": 41.5,
        "visited_pct": 12.0,
        "known_m2": 9.4,
        "free_m2": 6.1,
        "trail_m": 7.25,
    }
    assert world_row([], None)["destinations_seen"] == 0 and world_row([], None)["coverage_pct"] == ""
    assert set(world_row(None, None).values()) == {""}  # the simulator could not be asked


def test_the_decision_log_has_one_json_line_per_call_tick_and_notice(tmp_path: Path) -> None:
    log = DecisionLog(tmp_path / "run.log.jsonl")
    log.start()
    call = types.Part(function_call=types.FunctionCall(id="fc-1", name="set_nav_goal", args={"destination_id": "psg:1"}))
    log.event(
        Event(author="executor", invocation_id="i", content=types.Content(role="model", parts=[call]), custom_metadata={"s1": {"by": "nimble", "certainty": 0.91}}),
        tick=1,
    )
    log.tick(TickResult(number=1, seconds=12.5, events=[], state={}))
    log.notice({"kind": "failure", "tick": 2, "text": "Tick 2 failed (TimeoutError); retrying in 2 s."})
    log.close()
    lines = [json.loads(line) for line in (tmp_path / "run.log.jsonl").read_text().splitlines()]
    assert [line["type"] for line in lines] == ["event", "tick", "notice"]
    assert lines[0]["functionCall"]["args"] == {"destination_id": "psg:1"} and lines[0]["s1"]["certainty"] == 0.91
    assert lines[1]["number"] == 1 and lines[1]["seconds"] == 12.5
    assert all(isinstance(line["t"], float) for line in lines)


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


def test_a_suite_stops_when_a_providers_balance_runs_out() -> None:
    from gpt_pet.experiment import out_of_credit

    assert out_of_credit("ClientError(\"402 RESOURCE_EXHAUSTED. {'error': {'code': 402, 'message': 'Your prepayment credits are depleted.\")")
    assert out_of_credit("OpenAIError: OpenAI-compatible API 429: You exceeded your current quota (insufficient_quota)")
    assert out_of_credit("AnthropicError: Anthropic 400: Your credit balance is too low to access the Anthropic API.")
    assert not out_of_credit("ClientError(\"429 RESOURCE_EXHAUSTED. Too many requests\")")  # a rate limit passes
    assert not out_of_credit("")


def test_a_bot_can_switch_on_its_simulators_fixes() -> None:
    from gpt_pet.experiment import BotConfig

    bot = BotConfig.model_validate({"name": "luna_fixed", "sim_env": {"AI2THOR_MCP_FIXES": "replan,recovery"}})
    assert bot.sim_env == {"AI2THOR_MCP_FIXES": "replan,recovery"}
    assert BotConfig(name="plain").sim_env == {}


def test_the_coverage_timeline_says_how_fast_the_floor_was_seen() -> None:
    from gpt_pet.experiment import coverage_row

    timeline = [(30.0, 40.0), (60.2, 60.0), (90.0, 70.0), (120.5, 82.0), (150.0, 85.0), (180.0, 88.0), (240.0, 90.0)]
    row = coverage_row(timeline, 92.0, 300.0)
    assert (row["coverage_60s"], row["coverage_120s"], row["coverage_180s"], row["coverage_240s"]) == (60.0, 82.0, 88.0, 90.0)
    assert row["minutes_to_80pct"] == 2.01
    # trapezoids from (0, 0): a run that saw the floor sooner has a higher mean
    assert 70 < row["coverage_mean_pct"] < 85
    slow = coverage_row([(t, pct / 2) for t, pct in timeline], 92.0, 300.0)
    assert slow["coverage_mean_pct"] < row["coverage_mean_pct"] and slow["minutes_to_80pct"] == 5.0
    empty = coverage_row([], "", 300.0)
    assert empty == {"coverage_60s": "", "coverage_120s": "", "coverage_180s": "", "coverage_240s": "", "coverage_mean_pct": "", "minutes_to_80pct": ""}


def test_timed_events_are_offsets_from_the_clock_start_and_zero_is_allowed() -> None:
    assert TimedEvent(at="1m30s", tool="sim_person_walk_past_robot").seconds == 90
    assert TimedEvent(at=0, tool="sim_person_spawn").seconds == 0
    assert TimedEvent(at="0s", tool="sim_person_spawn").args == {}
    for bad in ("soon", -5):
        with pytest.raises(ValidationError):
            TimedEvent(at=bad, tool="sim_person_spawn")
    with pytest.raises(ValidationError):  # configs stay strict: a typo is an error, not a no-op
        TestConfig(events=[{"at": "5s", "tool": "sim_person_spawn", "when": "now"}])
    assert TestConfig().events == [] and TestConfig().sim_env == {}


def test_the_person_suite_crosses_the_three_candidates_with_a_scripted_walk_by() -> None:
    suite = load_suite(EXPERIMENTS / "suites" / "person_walkby.json")
    [test] = suite.tests
    assert test.sim_env == {"AI2THOR_MCP_PERSON_PRELOAD": "1"}
    timeline = [(event.seconds, event.tool) for event in sorted(test.events, key=lambda event: event.seconds)]
    assert timeline[0] == (10, "sim_person_spawn")
    assert (60, "sim_person_walk_past_robot") in timeline and (120, "sim_person_goto_robot") in timeline
    assert all(at < test.seconds for at, _ in timeline)
    assert [bot.name for bot in suite.bots] == ["gpt_41_mini", "gemini_31_flash_lite", "gpt_6_luna"]
    assert suite.runs_per_bot(test) == 3 and len(list(suite.plan())) == 9
    for bot in suite.bots:  # the bots' own simulator switches survive the test's
        assert {**test.sim_env, **bot.sim_env} == {"AI2THOR_MCP_PERSON_PRELOAD": "1", "AI2THOR_MCP_FIXES": "all"}


def test_the_person_analysis_reads_sightings_goals_and_drives_from_a_run_log() -> None:
    import importlib.util

    spec = importlib.util.spec_from_file_location("analyze_person", EXPERIMENTS / "person" / "analyze_person.py")
    analyze = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(analyze)

    def view(t: float, *labels: str, attention: str | None = None) -> dict:
        dests = [{"id": "obj:person_0" if label == "Person" else f"obj:{label}", "label": label, "distance_m": 2.5} for label in labels]
        result: dict = {"destinations": dests}
        if attention:
            result["attention"] = {"state": attention, "target": "obj:person_0"}
        return {"t": t, "type": "event", "functionResponse": {"name": "get_current_view", "response": {"result": result}}}

    lines = [
        view(5.0, "Chair"),
        {"t": 10.0, "type": "timed_event", "tool": "sim_person_spawn", "ok": True, "result": {"object_id": "person_0"}},
        view(20.0, "Chair"),
        view(30.0, "Person", "Chair", attention="aiming"),
        {"t": 31.0, "type": "event", "author": "executor", "text": "I see a person 2.5 m ahead next to the chair."},
        {"t": 33.0, "type": "tick", "new_goal": True, "goal": "go say hello to the person", "goal_kind": "social", "target_id": "obj:person_0"},
        {"t": 34.0, "type": "event", "s1": {"by": "rules"}, "functionCall": {"name": "set_nav_goal", "args": {"destination_id": "obj:person_0"}}},
        {"t": 50.0, "type": "event", "functionResponse": {"name": "set_nav_goal", "response": {"result": {}, "drive": {"outcome": "failed", "final_status": {"state": "failed", "reason": "blocked by person_0 near (6.93, -3.24)"}}}}},
        {"t": 55.0, "type": "event", "functionResponse": {"name": "set_nav_goal", "response": {"result": {}, "drive": {"outcome": "failed", "final_status": {"state": "failed", "reason": "blocked by Chair near (1.0, 2.0)"}}}}},
        {"t": 57.0, "type": "event", "functionResponse": {"name": "set_nav_goal", "response": {"result": {}, "drive": {"outcome": "failed", "final_status": {"state": "failed", "reason": "blocked by Person near (2.0, 2.0)"}}}}},
        {"t": 60.0, "type": "timed_event", "tool": "sim_person_walk_past_robot", "ok": True, "result": {}},
        view(70.0, "Person", attention="framed"),
        {"t": 120.0, "type": "timed_event", "tool": "sim_person_goto_robot", "ok": True, "result": {}},
        view(170.0, "Sofa", attention="off"),
    ]
    record = {"row": {"run_id": "r1", "bot": "b", "repeat": 1, "error": ""}, "test": {"events": [{}]}}
    row = analyze.analyze_run(record, lines)
    assert row["spawn_s"] == 10.0 and row["first_seen_s"] == 30.0 and row["seen_after_spawn_s"] == 20.0
    assert row["views_after_spawn"] == 4 and row["views_with_person"] == 2 and row["view_share"] == 0.5
    assert (row["encounters"], row["encounters_noticed"]) == (2, 1)  # the 120 s visit went unseen
    assert row["reports_mentioning"] == 1 and row["person_goals"] == 1
    assert (row["drives_to_person"], row["drives_to_person_s1"], row["drives_to_person_model"]) == (1, 1, 0)
    assert row["reacted"] and row["first_reaction_s"] == 3.0
    assert row["drives_blocked_by_person"] == 2  # by objectId or by label; the chair's doesn't count
    assert (row["attention_views"], row["framed_share"], row["first_framed_s"]) == (2, 0.5, 70.0)  # "off" views don't count
    summary = analyze.summarize([row])
    assert summary["b"]["runs_reacted"] == 1 and summary["b"]["encounters_noticed"] == "1/2"
    assert summary["b"]["framed_share"] == 0.5


def test_a_taken_port_is_recognised_in_the_simulator_log(tmp_path):
    from gpt_pet.experiment import port_taken

    log_path = tmp_path / "run.sim.log"
    log_path.write_text("ERROR:    [Errno 48] error while attempting to bind on address ('0.0.0.0', 51869): [errno 48] address already in use\\n")
    assert port_taken(log_path)
    log_path.write_text("Traceback ... ModuleNotFoundError\\n")
    assert not port_taken(log_path) and not port_taken(tmp_path / "missing.log")
