"""Live: the experiment harness end to end. Its own simulator, real Gemini and nimble, one short run."""
from __future__ import annotations

import json
from pathlib import Path

import pytest

from gpt_pet.experiment import RESULTS_CSV, SUMMARY_CSV, BotConfig, Suite, TestConfig, read_rows, run_suite
from gpt_pet.settings import Settings

pytestmark = [pytest.mark.live, pytest.mark.timeout(900)]


async def test_a_short_run_records_a_row_its_stats_and_a_summary(gemini_key: str, nimble: str, tmp_path: Path) -> None:
    bot = BotConfig(name="s1_on", overrides={"s1": {"enabled": True}, "brain": {"goal_delay_s": 0, "max_goals_per_run": 0}})
    suite = Suite(name="live-smoke", tests=[TestConfig(test_time="45s")], bots=[bot])
    [row] = await run_suite(suite, results_dir=tmp_path)
    assert row["error"] == "", row["error"]
    assert 44 <= row["duration_s"] <= 47
    assert row["llm_calls"] + row["s1_turns"] > 0 and row["tokens_total"] > 0
    assert row["s1_enabled"] is True and row["destinations_seen"] > 0
    assert row["objects_seen"] + row["passages_seen"] == row["destinations_seen"]
    # the simulator's exploration score, read when the clock stopped
    assert 0 < row["coverage_pct"] <= 100 and row["known_m2"] > 0 and row["visited_pct"] > 0
    assert row["trail_m"] >= row["drive_m"] * 0.8  # the trail also has the turns' camera swing
    assert row["drives_s1"] + row["drives_gemini"] >= row["drives"]  # every drive was someone's call
    assert row["ticks_failed"] == 0 and row["llm_max_ms"] <= 20_500  # the request timeout bounds every wait
    [saved] = read_rows(tmp_path / RESULTS_CSV)
    assert saved["run_id"] == row["run_id"] and saved["bot"] == "s1_on"
    assert (tmp_path / "runs" / f"{row['run_id']}.json").is_file()
    assert (tmp_path / SUMMARY_CSV).is_file()

    lines = [json.loads(line) for line in (tmp_path / "runs" / f"{row['run_id']}.log.jsonl").read_text().splitlines()]
    calls = [line for line in lines if line["type"] == "event" and "functionCall" in line]
    assert calls and calls[0]["functionCall"]["name"] == "get_current_view" and calls[0]["s1"]["by"] == "rules"
    # the stats snapshot is taken when the clock stops, the log closes a moment later
    drives = [line for line in calls if line["functionCall"]["name"] == "set_nav_goal"]
    assert 0 <= len(drives) - (row["drives_s1"] + row["drives_gemini"]) <= 1
    assert 0 <= sum(1 for line in drives if "s1" in line) - row["drives_s1"] <= 1
    ticks = [line["number"] for line in lines if line["type"] == "tick"]
    assert ticks == list(range(1, len(ticks) + 1)) and len(ticks) >= row["ticks"]


async def test_a_tests_timed_events_reach_its_own_simulator_on_time(sim_settings: Settings, tmp_path: Path) -> None:
    """A person walks into a run: the test's `sim_env` reaches the run's simulator, its events fire
    on the run's clock in time order (one after the clock stops never fires), and the log keeps
    what each returned."""
    model = sim_settings.model
    bot = BotConfig(
        name="person_smoke",
        overrides={
            "model": {"goal_setter": model.goal_setter, "executor": model.executor, "reasoning_effort": model.reasoning_effort},
            "s1": {"enabled": True, "rules_only": True},
            "brain": {"goal_delay_s": 0, "max_goals_per_run": 0},
        },
    )
    test = TestConfig(
        test_time="40s",
        sim_env={"AI2THOR_MCP_PERSON_PRELOAD": "1"},
        events=[
            {"at": "5s", "tool": "sim_person_goto_robot", "args": {"stop_m": 1.2}},
            {"at": 0, "tool": "sim_person_spawn"},
            {"at": "50s", "tool": "sim_person_stop"},
        ],
    )
    [row] = await run_suite(Suite(name="person-smoke", tests=[test], bots=[bot]), results_dir=tmp_path)
    assert row["error"] == "", row["error"]
    assert json.loads(row["sim_env"])["AI2THOR_MCP_PERSON_PRELOAD"] == "1"
    assert [event["tool"] for event in json.loads(row["events"])][0] == "sim_person_goto_robot"  # as written

    lines = [json.loads(line) for line in (tmp_path / "runs" / f"{row['run_id']}.log.jsonl").read_text().splitlines()]
    fired = [line for line in lines if line["type"] == "timed_event"]
    assert [line["tool"] for line in fired] == ["sim_person_spawn", "sim_person_goto_robot"]
    assert all(line["ok"] for line in fired), fired
    spawn, visit = fired
    assert spawn["result"]["object_id"] and spawn["result"]["mode"] == "idle"
    assert 0 <= spawn["t"] < 3 and 5 <= visit["t"] < 9
