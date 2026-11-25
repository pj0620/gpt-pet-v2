"""Live: the experiment harness end to end. Its own simulator, real Gemini and nimble, one short run."""
from __future__ import annotations

from pathlib import Path

import pytest

from gpt_pet.experiment import RESULTS_CSV, SUMMARY_CSV, BotConfig, Suite, TestConfig, read_rows, run_suite

pytestmark = [pytest.mark.live, pytest.mark.timeout(900)]


async def test_a_short_run_records_a_row_its_stats_and_a_summary(gemini_key: str, nimble: str, tmp_path: Path) -> None:
    bot = BotConfig(name="s1_on", overrides={"s1": {"enabled": True}, "brain": {"goal_delay_s": 0, "max_goals_per_run": 0}})
    suite = Suite(name="live-smoke", tests=[TestConfig(test_time="45s")], bots=[bot])
    [row] = await run_suite(suite, results_dir=tmp_path)
    assert row["error"] == "", row["error"]
    assert 44 <= row["duration_s"] <= 47
    assert row["llm_calls"] + row["s1_turns"] > 0 and row["tokens_total"] > 0
    assert row["s1_enabled"] is True and row["destinations_seen"] > 0
    [saved] = read_rows(tmp_path / RESULTS_CSV)
    assert saved["run_id"] == row["run_id"] and saved["bot"] == "s1_on"
    assert (tmp_path / "runs" / f"{row['run_id']}.json").is_file()
    assert (tmp_path / SUMMARY_CSV).is_file()
