"""Offline: how long the loop rests after a tick, as a pure function."""
from __future__ import annotations

from gpt_pet.goals import REST_STATUS, STATE_CURRENT_GOAL, STATE_LIMIT_REACHED, STATE_TICK
from gpt_pet.runtime import TickResult, failure_backoff, failure_notice, limit_notice, next_delay, rest_notice
from gpt_pet.settings import BrainSettings

BRAIN = BrainSettings(goal_delay_s=60, tick_delay_s=2, max_goals_per_run=10)


def result(state: dict) -> TickResult:
    return TickResult(number=state.get(STATE_TICK, 1), seconds=1.0, events=[], state=state)


def test_new_goal_rests_longer_than_a_continuation() -> None:
    new_goal = result({STATE_TICK: 3, STATE_CURRENT_GOAL: {"id": 2, "status": "active", "created_tick": 3}})
    continuation = result({STATE_TICK: 4, STATE_CURRENT_GOAL: {"id": 2, "status": "active", "created_tick": 3}})
    assert new_goal.started_new_goal and next_delay(new_goal, BRAIN) == 60
    assert not continuation.started_new_goal and next_delay(continuation, BRAIN) == 2


def test_limit_and_rest_records_do_not_rest() -> None:
    limited = result({STATE_TICK: 9, STATE_CURRENT_GOAL: {"id": 0, "status": REST_STATUS, "created_tick": 9}, STATE_LIMIT_REACHED: True})
    assert limited.limit_reached and not limited.started_new_goal
    assert next_delay(limited, BRAIN) == 0


def test_rests_and_the_goal_limit_explain_themselves_in_the_events_log() -> None:
    new_goal = result({STATE_TICK: 3, STATE_CURRENT_GOAL: {"id": 2, "status": "active", "created_tick": 3}})
    notice = rest_notice(new_goal, 60)
    assert (notice["kind"], notice["tick"], notice["seconds"]) == ("rest", 3, 60)
    assert notice["text"] == "Resting 60 s before the next tick: pacing after goal #2 started (goal_delay_s)."
    continuation = result({STATE_TICK: 4, STATE_CURRENT_GOAL: {"id": 2, "status": "active", "created_tick": 3}})
    assert rest_notice(continuation, 2)["text"] == "Resting 2 s before the next tick: pacing between ticks (tick_delay_s)."
    limited = result({STATE_TICK: 9, STATE_CURRENT_GOAL: {"id": 0, "status": REST_STATUS, "created_tick": 9}, STATE_LIMIT_REACHED: True})
    paused = limit_notice(limited, 10)
    assert paused["kind"] == "limit" and "goal limit (10)" in paused["text"]
    assert all(isinstance(n["timestamp"], float) for n in (notice, paused))


def test_failed_ticks_back_off_from_two_seconds_doubling_up_to_thirty() -> None:
    assert [failure_backoff(n) for n in range(1, 7)] == [2, 4, 8, 16, 30, 30]


def test_a_failed_tick_explains_itself_in_the_events_log() -> None:
    failed = TickResult(number=7, seconds=20.1, events=[], state={STATE_TICK: 6}, error="TimeoutError")
    assert failed.failed and failed.summary_dict()["error"] == "TimeoutError"
    assert "failed=TimeoutError" in failed.summary()
    notice = failure_notice(failed, 4)
    assert (notice["kind"], notice["tick"], notice["seconds"]) == ("failure", 7, 4)
    assert notice["text"] == "Tick 7 failed (TimeoutError); retrying in 4 s."
    assert "failed=" not in result({STATE_TICK: 1}).summary()
