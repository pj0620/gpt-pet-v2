"""Live: the pet notices a person and engages them (`[people]`), on a real simulator with a real
model, traced. Asserts what the rules guarantee (the person is seen, attention is set on them, the
goal decision goes to the model or a social goal starts), never which goal the model picks.
"""
from __future__ import annotations

import json
import logging
from pathlib import Path

import pytest

from gpt_pet.experiment import BotConfig, Suite, TestConfig, run_suite
from gpt_pet.settings import Settings

pytestmark = [pytest.mark.live, pytest.mark.timeout(900)]

log = logging.getLogger("gpt_pet.tests")


async def test_a_person_who_comes_over_is_seen_attended_and_raised_with_the_model(sim_settings: Settings, tmp_path: Path) -> None:
    model = sim_settings.model
    bot = BotConfig(
        name="people_live",
        overrides={
            "model": {"goal_setter": model.goal_setter, "executor": model.executor, "reasoning_effort": model.reasoning_effort},
            "s1": {"enabled": True, "rules_only": True, "scan_rule": True, "explore_policy": True, "explore_ticks": 3},
            "brain": {"goal_delay_s": 0, "max_goals_per_run": 0, "places_memory": True, "map_for_goals": True, "max_goal_attempts": 4},
            "people": {"enabled": True, "watch_every_s": 1.0},
        },
    )
    test = TestConfig(
        test_time="75s",
        trace=True,
        sim_env={"AI2THOR_MCP_PERSON_PRELOAD": "1"},
        events=[
            {"at": 0, "tool": "sim_person_spawn"},
            {"at": "2s", "tool": "sim_person_set_speed", "args": {"speed_mps": 0.5}},
            {"at": "3s", "tool": "sim_person_goto_robot", "args": {"stop_m": 1.6}},
        ],
    )
    [row] = await run_suite(Suite(name="people-live", tests=[test], bots=[bot]), results_dir=tmp_path)
    assert row["error"] == "", row["error"]
    assert int(row["people_sightings"] or 0) > 0, "the pet never saw the person"

    trace_dir = tmp_path / "runs" / f"{row['run_id']}.trace"
    records = [json.loads(line) for line in (trace_dir / "trace.jsonl").read_text().splitlines()]
    views = [r for r in records if r["type"] == "view"]
    assert any(r["people"] for r in views), "no view showed the person"
    attention_calls = [r for r in records if r["type"] == "call" and r["tool"] == "set_attention"]
    assert attention_calls and attention_calls[0]["args"]["destination_id"] == "obj:person_0", attention_calls

    decisions = [json.loads(r["text"]) for r in records if r["type"] == "text" and r["author"] == "goal_setter" and r["text"].startswith("{")]
    social = [d for d in decisions if d.get("goal") and ("person" in d["goal"].lower() or d.get("kind") == "social")]
    stats = json.loads((tmp_path / "runs" / f"{row['run_id']}.json").read_text())["stats"]
    escalations = stats["s1"]["escalations"]
    log.info("social goals: %s; escalations: %s", [d["goal"] for d in social], escalations)
    assert social or "goal_setter: person nearby" in escalations, (decisions, escalations)
    assert stats["people"]["sightings"] >= len([v for v in views if v["people"] and v["by"] != "watch"]) > 0

    # the trace was drawn when the run ended
    assert (trace_dir / "trajectory.png").is_file() and (trace_dir / "timeline.txt").is_file()
    assert list(trace_dir.glob("storyboard-*.png"))
