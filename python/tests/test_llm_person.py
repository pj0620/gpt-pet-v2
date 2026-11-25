"""Offline: when the LLM person stands still and for how long (experiments/person/llm_person.py)."""
from __future__ import annotations

import importlib.util
import math
import random
from pathlib import Path

EXPERIMENTS = Path(__file__).resolve().parents[2] / "experiments"


def load():
    spec = importlib.util.spec_from_file_location("llm_person", EXPERIMENTS / "person" / "llm_person.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_stand_stills_come_at_random_but_never_back_to_back() -> None:
    person = load()
    rng = random.Random(7)
    starts = [person.next_pause_at(rng, 100.0, 60.0) for _ in range(500)]
    assert all(130.0 <= t <= 190.0 for t in starts)  # 30-90 s after now for a 60 s average
    assert max(starts) - min(starts) > 50  # spread out, not a fixed beat
    assert person.next_pause_at(rng, 100.0, 0.0) == math.inf  # --pause-every 0: never


def test_a_stand_still_lasts_about_thirty_seconds() -> None:
    person = load()
    rng = random.Random(3)
    lengths = [person.pause_length(rng, 30.0) for _ in range(500)]
    assert all(24.0 <= s <= 36.0 for s in lengths)
    assert 28.0 <= sum(lengths) / len(lengths) <= 32.0


def test_the_defaults_are_a_slow_stroll_with_half_minute_stops() -> None:
    person = load()
    assert person.DEFAULT_SPEED_MPS == 0.3  # half the simulator's 0.6 m/s
    assert (person.DEFAULT_PAUSE_S, person.DEFAULT_PAUSE_EVERY_S) == (30.0, 60.0)
