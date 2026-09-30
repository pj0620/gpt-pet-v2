# Experiments

Run the pet for a fixed time under a **test config** and a **bot config**, and record its totals
(LLM calls, tokens, latency, drives, goals, System 1 activity, places discovered) as one CSV row
per run. Code: `python/src/gpt_pet/experiment.py`.

```bash
cd python
uv run gpt-pet experiment ../experiments/suites/s1_on_vs_off.json          # a whole suite
uv run gpt-pet experiment --test ../experiments/tests/5min_train1_3.json \
    --bot ../experiments/bots/s1_on.json --repeats 1                        # one test, some bots
uv run gpt-pet experiment --name quick --test '{"test_time": "90s", "room_id": "FloorPlan_Train1_1"}' \
    --bot '{"name": "s1_on_strict", "overrides": {"s1": {"enabled": true, "min_confidence": 0.9}}}'
uv run gpt-pet experiment --summary                                         # print the summary so far
```

The simulator checkout (`../ai2thor-mcp`), `GOOGLE_API_KEY` (in `python/src/gpt_pet/.env`) and,
for bots with System 1, Ollama >= 0.35 with `nimble` pulled are needed; a run takes about
`test_time` plus half a minute of simulator boot.

## Configs

**Test config** (`tests/*.json`): what to run.

| key | default | meaning |
|---|---|---|
| `test_time` | `"5min"` | how long the pet runs: `300`, `"90s"`, `"5min"`, `"2m30s"`, `"1h"` |
| `room_id` | `"FloorPlan_Train1_3"` | the AI2-THOR scene (RoboTHOR apartments `FloorPlan_Train1_1`..., iTHOR rooms `FloorPlan1`...) |
| `repeats` | `1` | runs per bot |
| `name` | | a label for the CSV (defaults to "5min in FloorPlan_Train1_3") |
| `warm_up` | `true` | boot the simulator and S1's model before the clock starts |

**Bot config** (`bots/*.json`): which brain. `profile` names the settings profile (`sim`), and
`overrides` changes any key of any section of that profile's ini file, e.g. `{"s1": {"enabled":
false}}`, `{"model": {"executor": "gemini-3.5-pro"}}`, `{"tool_limits": {"do_rotate": 4}}`.

The shipped bots turn the testing throttles off (`goal_delay_s = 0`, `max_goals_per_run = 0`) so
the pet works for the whole run instead of resting a minute after every new goal; set them back
in a bot file to measure the default pacing.

**Suite** (`suites/*.json`): `{"name", "tests", "bots"}`. Entries are inline objects or paths to
config files, relative to the suite. Every test is crossed with every bot, each pair run `repeats`
times, bots in order.

## Results

Written to `results/` (or `--results DIR`):

- `results.csv`: one row per run, appended. New metrics add columns; old rows keep blanks.
- `summary.csv`: mean and standard deviation per experiment and bot, over runs without an error.
- `runs/<run_id>.json`: the configs and the full stats snapshot of each run (hand-off reasons, per-agent calls).
- `runs/<run_id>.sim.log`: the simulator's log (git-ignored).

Every run gets a fresh simulator with the requested room on a free port, so runs start from the
same place and never touch any other simulator. Runs are not deterministic (Gemini samples at
`temperature = 0.2`), which is what `repeats` is for.

From the research notebook:

```python
from gpt_pet.experiment import BotConfig, Suite, TestConfig, run_suite
suite = Suite(name="try", tests=[TestConfig(test_time="2min")], bots=[BotConfig(name="s1_on", overrides={"s1": {"enabled": True}})])
rows = await run_suite(suite)
```
