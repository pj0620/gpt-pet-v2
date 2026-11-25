# Experiments

Run the pet for a fixed time under a **test config** and a **bot config**, and record its totals
(LLM calls, tokens, latency, drives, goals, System 1 activity, places discovered, floor explored)
as one CSV row per run. Code: `python/src/gpt_pet/experiment.py`.

```bash
cd python
uv run gpt-pet experiment ../experiments/suites/baseline.json              # S1 off vs on, 4 runs each, interleaved
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
| `events` | `[]` | simulator tools called at set times into each run: `[{"at": "60s", "tool": "sim_person_walk_past_robot", "args": {...}}]` (`at` may be 0). Each call and what it returned is a `timed_event` line in the run's log; a failing one is logged and the run goes on. For things happening in the world (a person walking by), never the pet's doing. |
| `sim_env` | `{}` | environment the test's world needs in its simulator, e.g. `{"AI2THOR_MCP_PERSON_PRELOAD": "1"}` for a person; merged under the bot's `sim_env`, which wins on a clash |

**Bot config** (`bots/*.json`): which brain. `profile` names the settings profile (`sim`), and
`overrides` changes any key of any section of that profile's ini file, e.g. `{"s1": {"enabled":
false}}`, `{"model": {"executor": "gemini-3.5-pro"}}`, `{"tool_limits": {"do_rotate": 4}}`.
`sim_env` sets the bot's simulator environment, for ai2thor-mcp's own switches, e.g.
`{"AI2THOR_MCP_FIXES": "replan,recovery"}`. A bot runs the profile's defaults for every key it does
not set, so when the defaults changed (2026-10-03: the simulator and brain fixes on) the bots in
`bots/`, `bots/cloud/` and `bots/fixes/` were pinned to the switches they ran with
(`AI2THOR_MCP_FIXES: none` where they had none); `bots/cloud_fixed/` runs the fixes.

The shipped bots turn the testing throttles off (`goal_delay_s = 0`, `max_goals_per_run = 0`) so
the pet works for the whole run instead of resting a minute after every new goal; set them back
in a bot file to measure the default pacing.

| bot | System 1 | places memory | scan rule | map for goals | explore policy |
|---|---|---|---|---|---|
| `s1_off` | off | off | off | off | off |
| `s1_on` | on | off | off | off | off |
| `s1_off_p1` | off | on | off | off | off |
| `s1_on_p1` | on | on | on | off | off |
| `s1_off_p2` | off | on | off | on | off |
| `s1_on_p2` | on | on | on | on | on |

The `_p1` and `_p2` bots switch on the exploration fixes in two steps (see the main README):
`_p1` stops drives that go nowhere, `_p2` also lets Gemini plan with the map and a summary of
what the pet has done. Every bot sets all four switches itself, so it measures the same thing
whatever the profile's defaults are (the `sim` profile has had all four on since 2026-10-01,
which is the `s1_on_p2` brain). `suites/exploration.json` runs all six bots, 4 runs each,
interleaved (about 2 h 15 min); `suites/p1.json` and `suites/p2.json` compare one step at a time.

**Other models** (`bots/cloud/`, 2026-10-02): the `s1_on_p2` brain with another model as goal
setter and executor, each at its low-latency thinking setting: Gemini 3.5 Flash as the reference
(`gemini_35_flash`) and at minimal thinking, 3.5 / 3.1 Flash-Lite, 3.7 / 3.8 Flash and Robotics-ER
2 at low thinking (`suites/cloud_google.json`); GPT-6 Luna, GPT-4.1 mini and Claude Haiku 4.5
(`suites/cloud_openai_anthropic.json`, needs `OPENAI_API_KEY` and `ANTHROPIC_API_KEY`); DeepSeek
V4.1 Flash and Qwen3.8 Flash through OpenRouter (`suites/cloud_openrouter.json`). A suite whose
models need a key that is not set stops before any simulator boots. Run them into their own
folder and analyse them with `cloud_models/analyze.py`:

```bash
uv run gpt-pet experiment ../experiments/suites/cloud_google.json --results ../experiments/results/cloud_models
uv run --with matplotlib python ../experiments/cloud_models/analyze.py   # scorecard.csv, analysis.json, figures/
```

**Suite** (`suites/*.json`): `{"name", "tests", "bots"}`, optionally `"repeats"` (overrides every
test's) and `"order"`. Entries are inline objects or paths to config files, relative to the suite.
Every test is crossed with every bot, each pair run `repeats` times. `"order": "interleaved"` (the
default) runs every bot once per repeat, reversing the order each time (A B, B A, A B, ...), so
drift during a long session (Gemini latency, machine load) falls on all bots alike; `"blocked"`
runs all of one bot's runs before the next bot's (how the first S1 experiment, before 2026-10-01,
was run). `--repeats` and `--order` override a suite on the command line.

## Results

Written to `results/` (or `--results DIR`):

- `results.csv`: one row per run, appended. New metrics add columns; old rows keep blanks.
- `summary.csv`: mean and standard deviation per experiment and bot, over runs without an error.
- `runs/<run_id>.json`: the configs and the full stats snapshot of each run (hand-off reasons, per-agent calls).
- `runs/<run_id>.log.jsonl`: the run's decisions, one JSON line each: every tool call and response
  and every text turn (with `s1` when System 1 took the turn, and its certainty; `fallback` when
  Gemini failed and a fallback answered), each tick's summary, and the runtime's notices. `t` is
  seconds since the clock started.
- `runs/<run_id>.sim.log`: the simulator's log (git-ignored).

Exploration columns, read from the simulator when the clock stops:

| column | meaning |
|---|---|
| `coverage_pct` | share of the room's reachable floor (AI2-THOR's 0.25 m navigation grid) the occupancy map has seen as free |
| `visited_pct` | share of the reachable floor within 0.5 m of the path the robot travelled |
| `known_m2`, `free_m2` | area the map has observed, and seen free |
| `trail_m` | distance the robot's camera travelled (drives and the swing of turns) |
| `destinations_seen` = `objects_seen` + `passages_seen` | destinations the robot discovered (`list_destinations`) |

These come from the simulator-only tool `sim_exploration_stats` (ai2thor-mcp); the pet never sees it.
The harness also samples it every 30 s during the run (`coverage_timeline` in the run's JSON):
`coverage_60s` .. `coverage_240s` (floor seen by then), `coverage_mean_pct` (mean over the run,
from 0 at the start: higher for a pet that sees the floor sooner) and `minutes_to_80pct`. They
tell runs apart once most of them see nearly all of a small room by the end.

Cost columns: `cost_usd` (the run's model calls at the prices in `pricing.json`, read 2026-10-02:
uncached input, cache reads, cache writes and output with thinking each at their own price, or
what OpenRouter reported), `cost_per_hour_usd`, and `cost_unpriced` (models the table lacks; their
calls are left out). Local models (`ollama/...`) add `llm_reloads`, `llm_load_s`,
`llm_prompt_tok_s`, `llm_output_tok_s` and `llm_max_prompt_tokens` (Ollama's own timings) and
`ollama_size_gb` / `ollama_gpu_pct` / `ollama_loaded_start|end` (memory, from `ollama ps`).

Drive and stall columns: `drives_s1` / `drives_gemini` (who decided each `set_nav_goal`),
`null_drives` and `null_drive_pct` (drives planned under 0.3 m: the robot was already there),
`passage_drives` (drives to doorways and openings), `drive_m` (distance driven),
`destinations_reached`; `llm_max_ms` (the longest wait for Gemini, answered or not: bounded by
`[model] request_timeout_s`), `llm_errors`, `llm_fallbacks` and `ticks_failed`.

Every run gets a fresh simulator with the requested room on a free port, so runs start from the
same place and never touch any other simulator. Runs are not deterministic (Gemini samples at
`temperature = 0.2`), which is what `repeats` is for.

From the research notebook:

```python
from gpt_pet.experiment import BotConfig, Suite, TestConfig, run_suite
suite = Suite(name="try", tests=[TestConfig(test_time="2min")], bots=[BotConfig(name="s1_on", overrides={"s1": {"enabled": True}})])
rows = await run_suite(suite)
```

## Findings

**2026-10-01, `exploration`** (FloorPlan_Train1_3, 5 min per run, 4 runs per bot, interleaved,
no errors). Means:

| per 5-min run | `s1_off` | `s1_off_p1` | `s1_off_p2` | `s1_on` | `s1_on_p1` | `s1_on_p2` |
|---|---|---|---|---|---|---|
| floor coverage | 70.4% | 82.2% | 78.0% | 82.2% | 77.2% | **93.6%** |
| destinations seen | 35.2 | 35.5 | 39.5 | 38.2 | 36.2 | **45.8** |
| objects seen | 27.0 | 30.2 | 31.0 | 29.8 | 28.2 | **34.0** |
| drives through passages | 4.0 | 2.0 | 9.5 | 3.5 | 3.2 | **12.5** |
| drives that went nowhere | 52% | 33% | 4% | 59% | 36% | **10.5%** |
| tokens | 737k | 581k | 624k | 610k | 548k | **363k** |
| Gemini calls | 134.5 | 116.8 | 96.2 | 103 | 92.2 | **51.5** |

`s1_on_p2` explores most and costs least: Gemini sets every goal, seeing the map
(about 14 goal decisions per run), and System 1 carries the goals out, deciding about 17 of the
21 drives per run. The places memory alone (`_p1`) leaves a third of the drives going nowhere
because Gemini still picks destinations marked `here`; since then the executor refuses such
drives. The `guard` suite (3 runs each) confirmed it: wasted drives fell from 33% to 18% for
`s1_off_p1` and from 10.5% to 5% for `s1_on_p2`. The `baseline` suite (same day, 4 runs each)
found no exploration gap between `s1_off` and `s1_on` once runs were interleaved and Gemini
calls had a timeout.

What still varies most is failed drives: the `_p2` bots drive farther and fail 3 to 11 drives
per run (the others 0 to 3), mostly "blocked" on the way. In the worst run the robot was stuck:
8 drives in a row failed near the same spot, and coverage fell to 63%. Getting unstuck (backing
off, or handing the turn to Gemini after repeated blocked drives) is the next thing to try.

**2026-10-02, cloud models** (`cloud-google`, `cloud-round2`, `cloud-finals`, `cloud-luna`; results
in `results/cloud_models/`, report and analysis in `cloud_models/`). The `s1_on_p2` brain with 18
small-model settings as goal setter and executor, 75 valid 5-minute runs in FloorPlan_Train1_3
(9 left out in `cloud_models/excluded_runs.json`: an adapter bug, fixed, and the Gemini credits
running out). Means, cost per hour of continuous running at 2026-10-02 prices:

| setting | runs | floor seen | places | p95 call | cost / hour |
|---|---|---|---|---|---|
| Gemini 3.5 Flash (reference) | 5 | 87.0% | 43.2 | 6.0 s | $5.79 |
| Gemini 3.1 Flash-Lite | 4 | 92.3% | 46.2 | 3.7 s | $1.04 |
| Gemini 3.5 Flash-Lite, `image_detail = low` | 2 | 97.4% | 50.0 | 3.5 s | $0.85 |
| GPT-4.1 mini (`openai/gpt-4.1-mini`) | 7 | 89.5% | 43.9 | 2.9 s | $1.25 |
| GPT-5.6 Luna, reasoning none | 8 | 86.4% | 40.5 | 3.4 s | $0.56 |
| Gemini Robotics-ER 2, low thinking | 4 | 94.7% | 49.8 | 3.7 s | $5.08 ($10.17 from 2027) |
| Claude Haiku 4.5 | 4 | 81.5% | 38.5 | 4.7 s | $3.36 |
| GPT-6 Luna, reasoning none | 8 | 77.9% | 37.2 | 3.3 s | $0.32 |

Gemini 3.1 Flash-Lite explores at least as well as Gemini 3.5 Flash for a fifth of the cost and is
the recommendation; 3.5 Flash-Lite with low-resolution images did best of all but in only 2 runs
(its finals, `suites/cloud_finals_gemini.json`, wait for Gemini credits). GPT-4.1 mini is the
fastest and the best model outside Google. GPT-5.6 Luna is the budget pick: as much floor as
the reference on average for a tenth of the cost, but 2 of its 8 runs stalled near 65% (it keeps
choosing drives that fail). GPT-6 Luna and GPT-5.4 nano keep abandoning or repeating goals. Every setting, metric and caveat: `cloud_models/report.html`.

**2026-10-02/03, fixes** (`suites/fixes_*.json`; results in `results/fixes/` for FloorPlan_Train1_3
and `results/fixes_scene2/` for FloorPlan_Train4_1; report and per-round analyses in `fixes/`).
The session analysis of GPT-5.6 Luna traced its lost time to failed drives: the robot wedged itself
against walls and furniture and failed drive after drive. ai2thor-mcp got switchable navigation
fixes (`AI2THOR_MCP_FIXES`, see its README) and the brain four flags (`[brain] failed_guard`,
`failure_guidance`, `[s1] unstuck_rule`, `explore_ticks`); 84 runs of 5 minutes with Luna as
System 2, 4 per arm, interleaved, no errors. Means:

| per 5-min run | floor seen | failed drives | model calls | cost / hour |
|---|---|---|---|---|
| Train1_3, no fixes (round 1) | 83.1% | 10.5 | 84 | $0.59 |
| Train1_3, simulator fixes and grid paths (round 1) | 95.3% | 0.25 | 52 | $0.36 |
| Train4_1, no fixes (round 3) | 81.9% | 9.0 | 89 | $0.65 |
| Train4_1, simulator fixes with snap (round 3) | 99.8% | 0 | 53 | $0.42 |
| Train4_1, all fixes (round 4) | 100% | 0 | 48 | $0.35 |
| Train4_1, all fixes, System 1 rules only (round 4) | 99.4% | 0 | 81 | $0.49 |

The simulator's fixes ended the failed drives in both apartments, and because every failure had
cost more model calls, they cut the cost by 36 to 39% as well. Train4_1 is an apartment none of the
fixes was tuned on. `snap` was added after round 2, where 22 of 27 failed grid drives clipped one
doorway's frame. When navigation keeps failing (round 2b, no simulator fixes), the failed guard
and the unstuck rule halve the failed drives but do not raise the floor seen by themselves.
Explore ticks and failure guidance cut about a fifth of the cost, low-detail images about a tenth
more. System 1 without nimble (`[s1] rules_only`) explored best, for 40% more model spend; System 1
switched off (rules too) explored worst and cost 81% more. Every arm, measure and caveat:
`fixes/report.html`.

**2026-10-03, cloud models with the fixes** (`suites/cloud_fixed*.json`; results in
`results/cloud_fixed/`, report and analysis in `cloud_fixed/`). Every model of the cloud study again,
with the fixes the fixes study recommends, plus nine new OpenRouter models: 3 runs of 3 minutes
each in FloorPlan_Train1_3, interleaved; 69 valid runs (4 left out in
`cloud_fixed/excluded_runs.json`: our setting and our bug). Means, cost per hour of continuous
running at 2026-10-03 prices:

| model | floor seen in 3 min | mean over the run | p95 call | cost / hour (first study) |
|---|---|---|---|---|
| GPT-4.1 mini | 91.4% | 69.3% | 2.6 s | $0.62 ($1.25) |
| Gemini 3.1 Flash-Lite | 90.9% | 72.3% | 2.7 s | $0.60 ($1.04) |
| Gemini 3.5 Flash-Lite | 86.5% | 73.0% | 3.0 s | $0.79 ($1.46) |
| GPT-6 Luna, reasoning none | 86.0% | 70.3% | 2.8 s | $0.18 ($0.32) |
| GPT-5.6 Luna, reasoning none | 85.9% | 66.4% | 3.6 s | $0.31 ($0.56) |
| Gemini 3.5 Flash (reference) | 81.3% | 65.6% | 4.6 s | $3.86 ($5.79) |
| DeepSeek V4.1 Flash (OpenRouter) | 81.7% | 66.6% | 4.2 s | $0.22 (new) |
| Ling 3.0 Flash VL (OpenRouter) | 76.2% | 66.2% | 4.6 s | $0.04 (new) |

Failed drives are gone for every model (one in 69 runs), and 12 of the 14 models of the first study
cost 33 to 55% less per hour. Gemini 3.5 Flash no longer explores best: GPT-4.1 mini saw clearly more
floor and saw it sooner, and Gemini 3.5 Flash-Lite, GPT-6 Luna and GPT-5.4 mini saw it sooner, at a
third to a twentieth of its cost. Recommended: GPT-4.1 mini or Gemini 3.1 Flash-Lite; GPT-6 Luna on
a budget. Slow models (Llama 4 Maverick, GLM-5.3 Flash) explore least. The rematch found a bug: a
model call could outlast its 20 s timeout (httpx bounded each read; OpenRouter keeps connections
open with whitespace); `model_api.post_json` now bounds the whole call. Every model, measure and
caveat: `cloud_fixed/report.html`.

**2026-10-03, a person walks by** (`suites/person_walkby.json`, results in `results/person/`,
study in `person/`). The simulator now has a person, someone who walks about and whom the robot
sees as a destination labeled `Person`. A test's `events` script what the person does: appear out
of sight, wander, walk past the pet at 60 s, come over and stand 1.2 m away at 120 s. The three
recommended models ran it 3 times each with the brain unchanged.

- **They all see the person but none cares.** Every run saw the person, 12 to 27 s after they
  appeared on average, and reported them, but only one of 123 goal decisions set a social goal
  ("find and approach a person", Gemini 3.1 Flash-Lite).
- **Why:** the places memory marks a person who walks up as `here` (already reached), the goal
  setter decides only 9 to 21 times in 3 minutes because System 1 continues exploration goals,
  and nothing makes a person in view interrupt the goal.
- **People get in the way:** 2.7 to 4 drives per run failed with the person in the path, and the
  pet retried the same drive.

Next, with the user's go-ahead: the deferred salient-event interrupt, a places memory that knows
people move, and handling a drive blocked by a person. Per-run measures: `person/README.md`.
