"""Offline: how long the loop rests after a tick, as a pure function."""
from __future__ import annotations

from gpt_pet.goals import REST_STATUS, STATE_CURRENT_GOAL, STATE_LIMIT_REACHED, STATE_TICK
from gpt_pet.runtime import TickResult, next_delay
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
