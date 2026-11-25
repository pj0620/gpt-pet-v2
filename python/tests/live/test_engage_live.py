"""Live: the interest check (`[people] engage`) on a real simulator with a real model, traced.
Someone who comes over and waves is found interested ("Human wants to interact with me!") and gets
an interaction goal; someone busy at a piece of furniture is found not interested and left be.
Asserts what the check guarantees, never which goal the model words."""
from __future__ import annotations

import json
import logging
from pathlib import Path

import pytest

from gpt_pet.experiment import BotConfig, Suite, TestConfig, run_suite
from gpt_pet.settings import Settings

pytestmark = [pytest.mark.live, pytest.mark.timeout(900)]

log = logging.getLogger("gpt_pet.tests")


def bot(sim_settings: Settings) -> BotConfig:
    model = sim_settings.model
    return BotConfig(
        name="engage_live",
        overrides={
            "model": {"goal_setter": model.goal_setter, "executor": model.executor, "reasoning_effort": model.reasoning_effort},
            "s1": {"enabled": True, "rules_only": True, "scan_rule": True, "explore_policy": True, "explore_ticks": 3},
            "brain": {"goal_delay_s": 0, "max_goals_per_run": 0, "places_memory": True, "map_for_goals": True, "max_goal_attempts": 4},
            "people": {"enabled": True, "watch_every_s": 1.0, "engage": True},
        },
    )


def records_of(tmp_path: Path, row: dict) -> list[dict]:
    trace_dir = tmp_path / "runs" / f"{row['run_id']}.trace"
    return [json.loads(line) for line in (trace_dir / "trace.jsonl").read_text().splitlines()]


async def test_someone_who_comes_and_waves_is_found_interested_and_engaged(sim_settings: Settings, tmp_path: Path) -> None:
    test = TestConfig(
        test_time="90s",
        trace=True,
        sim_env={"AI2THOR_MCP_PERSON_PRELOAD": "1", "AI2THOR_MCP_PERSON_DESCRIBE_MOTION": "0"},
        events=[
            {"at": 0, "tool": "sim_person_spawn"},
            {"at": "2s", "tool": "sim_person_set_speed", "args": {"speed_mps": 0.5}},
            {"at": "3s", "tool": "sim_person_activity", "args": {"kind": "interact", "style": "wave", "patience_s": 25}},
        ],
    )
    [row] = await run_suite(Suite(name="engage-live", tests=[test], bots=[bot(sim_settings)]), results_dir=tmp_path)
    assert row["error"] == "", row["error"]
    records = records_of(tmp_path, row)
    notices = [r for r in records if r["type"] == "notice" and r.get("kind") in ("social", "ignore")]
    log.info("interest notices: %s", [n["text"] for n in notices])
    social = [n for n in notices if n["kind"] == "social"]
    assert social, [n["text"] for n in notices]
    assert social[0]["text"].startswith("Human wants to interact with me!")
    assert social[0]["engagement"]["decision"] == "interested" and social[0]["engagement"]["readings"] >= 1
    decisions = [json.loads(r["text"]) for r in records if r["type"] == "text" and r["author"] == "goal_setter" and r["text"].startswith("{")]
    after = [d for d in decisions if d.get("kind") == "social" or d.get("target_id") == "obj:person_0"]
    assert after, decisions
    stats = json.loads((tmp_path / "runs" / f"{row['run_id']}.json").read_text())["stats"]
    assert stats["people"]["interested"] >= 1 and stats["people"]["cue_reads"] >= 1


async def test_someone_busy_with_furniture_is_left_be(sim_settings: Settings, tmp_path: Path) -> None:
    test = TestConfig(
        test_time="75s",
        trace=True,
        sim_env={"AI2THOR_MCP_PERSON_PRELOAD": "1", "AI2THOR_MCP_PERSON_DESCRIBE_MOTION": "0"},
        events=[
            {"at": 0, "tool": "sim_person_spawn"},
            # the dining table beside where the robot starts in FloorPlan_Train1_3
            {"at": "2s", "tool": "sim_person_activity", "args": {"kind": "object", "seconds": 70, "object_id": "DiningTable|+01.41|+00.00|-01.60"}},
        ],
    )
    [row] = await run_suite(Suite(name="engage-live", tests=[test], bots=[bot(sim_settings)]), results_dir=tmp_path)
    assert row["error"] == "", row["error"]
    records = records_of(tmp_path, row)
    notices = [r for r in records if r["type"] == "notice" and r.get("kind") in ("social", "ignore")]
    log.info("interest notices: %s", [n["text"] for n in notices])
    samples = [r for r in records if r["type"] == "sample" and (r.get("person") or {}).get("activity")]
    assert samples and samples[-1]["person"]["activity"]["kind"] == "object"
    if not notices:
        pytest.skip("the pet never came across the person in 75 s (a run of the scene's luck, not a failure of the check)")
    assert notices[0]["kind"] == "ignore", [n["text"] for n in notices]
    assert not any(n["kind"] == "social" for n in notices)
