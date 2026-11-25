# gpt-pet-v2

A LLM Powered Pet that continuously explores, learns, and interacts with people and animals.

The pet is autonomous: it decides a goal, stores it in its goal memory, executes it through MCP
tools, marks it done or abandoned on its own, and picks the next one. No human in the loop.

```
gpt-pet-v2/
  python/   the brain: Google ADK agents, MCP client, CLI  (uv project)
  ui/       GPTPet Management Portal: React + Vite, run with Bun
  notebooks/research.ipynb
```

## The brain (`python/`)

One **tick** of the pet's life is a Google ADK `Workflow`:

```
goal_setter  ->  update_goal_memory  ->  executor
(LLM, no tools)  (deterministic)         (LLM + MCP tools)
```

- `goal_setter` reads the goal memory and the executor's last report and emits a decision:
  continue, done, abandoned, or a new goal (optionally with sub-goals).
- `update_goal_memory` applies that decision to session state (`current_goal`, `goal_history`).
- `executor` drives the robot with the MCP tools only and ends with a short report.

`gpt-pet run` loops ticks. `adk web src` (from `python/`) runs one tick per message for debugging.

**System 1** (`[s1]` in the profile, `python/src/gpt_pet/s1.py`) puts a local decision model in
front of Gemini: nimble (Bespoke Labs) served by Ollama, asked typed questions through the
System One API (`POST /v1/systemone`). It takes the turns it can decide with confidence and hands
the rest to Gemini, which still writes every new goal:

- goal setter: a confident "still in progress" skips Gemini; new, finished and uncertain goals go to Gemini;
- executor: rules take the mechanical turns (look first, look again after moving); nimble picks the
  next destination or turn from what the MCP server offers, or decides the success criteria are met
  and writes the report. Once Gemini takes a turn, it keeps the rest of that tick.

`enabled = true|false` switches it on or off (on in `sim`, off in `real`); `min_confidence` sets
how decisive an answer must be. It needs Ollama >= 0.35 with the model pulled
(`ollama pull nimble`); if Ollama is unreachable, Gemini simply takes every turn. The portal's
Stats panel shows S1's share of the turns and why it handed turns to Gemini, and the Events Log
marks S1's own calls with an **S1** badge.

All actuation goes through an MCP server chosen by a profile:

| profile | server                                       | ini file                              |
|---------|----------------------------------------------|---------------------------------------|
| `sim`   | ai2thor-mcp (AI2-THOR simulator)             | `python/src/gpt_pet/config/sim.ini`   |
| `real`  | gpt-pet-mcp (robot; server not yet written)  | `python/src/gpt_pet/config/real.ini`  |

Each profile also carries `[brain]` budgets (LLM calls per tick, goal attempts), `[tool_limits]`,
per-tick call caps per tool that bound how many times the executor can look, turn or retry, and
`[nav]`. `set_nav_goal` returns to the executor only once the drive has ended: the brain checks
`get_nav_status` itself every `poll_interval_s` (giving up after `max_wait_s`), so no LLM calls
are spent while the robot drives.

**When Gemini fails.** `[model] request_timeout_s` (20 s) bounds every Gemini call. A call that
times out or fails transiently (dropped connection, rate limit, server error) costs one turn, not
the tick: the executor's fallback reports what the robot did so far, and the goal setter's keeps
the active goal (a fallback never writes one). With no goal to keep, the tick fails and the loop
retries after a backoff (2 s, doubling up to 30 s); the Events Log says why.

**Exploration** (on in `sim`, off in `real`; measured in `experiments/README.md`, Findings):

- `[brain] places_memory` remembers places across ticks (`python/src/gpt_pet/places.py`) and marks
  each view's destinations: `here` (within reach, so a drive there goes nowhere; `here_m`, plus a
  reach learned from drives), `visited`, `failed`. With S1 on, it skips and ranks its options by
  the marks, and the executor refuses a drive to a place marked `here`.
- `[s1] scan_rule` turns by rule when nothing new is in view, and lets S1 write the report when the
  tick's looks run out.
- `[brain] map_for_goals` shows Gemini the robot's top-down map (`get_map`) when it decides a goal,
  plus a summary of what the pet has done and the places it has seen but not visited; a new goal
  may name one of them as `target_id`.
- `[s1] explore_policy` carries that out by rule: drive to the target first; on an explore goal,
  take unvisited passages first, then the farthest unvisited objects. Gemini still sets every goal.

**When drives keep failing** (on in `sim` since 2026-10-03, off in `real`, whose places memory and
System 1 are off; measured in `experiments/README.md`, Findings):

- `[brain] failed_guard` remembers where each drive failed: a view marks a destination
  `failed_here` when a drive there already failed from where the robot stands, the executor
  refuses another such drive (as it refuses drives to `here`), and the goal setter's list notes
  the places drives failed to reach and puts them last.
- `[s1] unstuck_rule`: after two drives in a row failed without the robot moving, System 1 backs
  up half a metre and turns around before anything else.
- `[s1] explore_ticks`: an explore goal goes on by rule for up to that many stretches while
  unvisited places remain, so the pet waits less for its goal decisions; one that reaches its last
  attempt still exploring counts as done (`[brain] max_goal_attempts` must leave room).
- `[brain] failure_guidance` tells the executor not to retry a failed drive from the same spot,
  and the goal setter that one failed drive is no reason to abandon a goal.

In the simulator, ai2thor-mcp runs its own navigation fixes by default (re-planning, backing up, grid
paths followed exactly, short failure reasons, step-wise moves); `AI2THOR_MCP_FIXES` picks some or
none, see its README.

**People** (`[people]`, off by default; `python/src/gpt_pet/people.py`; measured in
`experiments/people/README.md`). People move, so they are never places: the places memory gives
them no marks, exploring never drives to them, and the executor gets ai2thor-mcp's attention tools
(`set_attention`, which returns once the robot has framed the person). Three things change, all
without model calls of their own:

- the person watch looks every `watch_every_s` (1.5 s) while the robot drives and while it rests
  between ticks: a person coming into view stops a drive that is not about them (outcome
  `interrupted`) and wakes a resting pet; a pet that saw someone the goal setter has not heard
  about yet does not rest at all;
- System 1 turns to a person who comes into view and ends the stretch with a report naming them;
  on a social goal it attends, looks, and hands the turn to the model to say what the person is
  doing (or turns toward where they were last seen);
- the goal setter is asked (System 1 does not decide) whenever someone new is near, and its prompt
  lists who was seen and allows dropping the current goal for them.

The pet reacts to the same person at most once every `react_every_s` (60 s) unless its goal is
about them. A drive that failed because a person stood in the way ("blocked by Person", which
ai2thor-mcp reports after waiting 10 s for them) does not count against the destination or as the
robot being stuck: the person moves on.

**Does this person want to interact?** (`[people] engage`, off by default; `python/src/gpt_pet/social.py`;
gptpet-plans: People Step 3; measured in `experiments/people/README.md`). Turning to someone whose
interest it does not know, the pet watches them for up to `engage_window_s` (10 s), looking every
2 s, and decides from what a robot with a camera and depth can perceive: each look's frame goes to a
small vision call (the cue reader: facing, posture, gesture, busy with, looking at me, read from
the person's box cropped out of the frame at low image detail; `cue_model`, by default the
executor's model, GPT-4.1 mini recommended: it reads body language far better than GPT-6 Luna,
`experiments/people/cue_benchmark.py`; a pose detector can replace it on the robot), and the person's
track gives approach, stop near, pass by and leave. A wave, beckon, clap or reach toward the pet,
or crouching down facing it, decides at once; looking at the pet while standing near it, or coming
over and staying, counts less; being busy with something, walking past or leaving count against.

- Interested: the Events Log says "Human wants to interact with me!" with what they did, and the
  goal setter picks an interaction goal (greet them close up, follow them if they beckoned and
  walk off, go where they point). `set_attention` takes `standoff_m`, `follow` and `greet` for
  that.
- Not interested: "... leaving them be for 2 min"; attention ends, and the watch, System 1 and the
  goal setter ignore them for `ignore_for_s` (120 s), doubling for each disinterest in a row up to
  `ignore_max_s` (15 min). Exceptions: seen within 2.5 m (and not so close that a level camera
  shows only their legs), they get a glance (one cue reading, at most every 15 s), and a wave,
  beckon, clap, reach or crouch ends the leave-be; so does walking right up to the pet (within
  1.5 m, closer than when it decided) and staying, and, once per decision, staying right beside it
  (within 1 m, 20 s or more after it decided).
- The check waits for someone still walking up to the pet (that time does not count toward the
  window, up to 25 s in all): people often wave only once they have arrived.
- It learns: each decision is checked afterwards against what the robot sees (still near 15 s
  after "interested"; not coming up within a minute after "not interested"), and each cue's weight
  follows its record (0.5x to 1.5x). The goal setter reads the lessons ("waved at me" meant they
  wanted to interact 3 of 3 times). `memory_file` keeps it all across sessions.

**Traces: what the pet saw, did, and where.** `python/src/gpt_pet/trace.py` records a run as
`trace.jsonl` plus the JPEG of every view the brain received (as the model saw it); each view line
carries the robot's pose, camera tilt, attention, the people in view and every destination, and
the simulator's ground truth (robot, person, attention, nav) is sampled each second. `gpt-pet
trace render DIR` draws it: `timeline.txt` (one line per decision, call, result and view),
`storyboard-NN.png` (12 captioned frames a sheet: time, tick, who decided, pose, tilt, people,
next action), `trajectory.png` (robot and person paths, numbered views with headings) and
`index.html`. Record with `gpt-pet serve --trace DIR`, or per experiment run with `trace: true` in
the test or `gpt-pet experiment --trace` (into `runs/<run_id>.trace/`, drawn when the run ends).

**Other models for the goal setter and the executor.** `[model] goal_setter` / `executor` take
any Gemini model id, or another provider's model by prefix (`python/src/gpt_pet/agents/models.py`):

| name | provider | key (in `python/src/gpt_pet/.env`) |
|---|---|---|
| `gemini-3.5-flash`, `gemini-3.5-flash-lite`, ... | Gemini API (ADK's own client) | `GOOGLE_API_KEY` |
| `openai/<model>`, e.g. `openai/gpt-5-mini` | OpenAI Chat Completions | `OPENAI_API_KEY` |
| `anthropic/<model>`, e.g. `anthropic/claude-haiku-4-5` | Anthropic Messages | `ANTHROPIC_API_KEY` |
| `openrouter/<vendor>/<model>` | OpenRouter (many vendors, one key) | `OPENROUTER_API_KEY` |
| `ollama/<tag>`, e.g. `ollama/qwen3-vl:8b-instruct` | a local model served by Ollama | none |

`[model] reasoning_effort` (`minimal` .. `high`; empty keeps each API's default) sets Gemini's
thinking level and OpenAI's reasoning effort; `max_output_tokens` caps OpenAI and Anthropic
answers. Every adapter reports tokens the way Gemini does (cached and thinking tokens apart), so
the Stats panel and the experiment harness count every model alike, and the harness prices each
run from `experiments/pricing.json`. `[s1] rules_only` keeps System 1's rules without its
decision model, for machines that cannot hold nimble next to a local System 2.

Swapping the backend is an ini change. `GPTPET_PROFILE` selects the profile (the CLI flag
`--profile` overrides it); `GPTPET_CONFIG_DIR` points at another directory of ini files.

### Setup

```bash
cd python
uv sync
# python/src/gpt_pet/.env (git-ignored):
#   GOOGLE_API_KEY=...
#   GPTPET_PROFILE=sim
```

Start the simulator server from the ai2thor-mcp checkout (it listens on port 8000):

```bash
uv run python/ai2thor_mcp/main.py --http --scene FloorPlan_Train1_3
```

### Run

```bash
cd python
uv run gpt-pet serve --profile sim           # pet loop + portal API on :8080 (launches the simulator if needed)
uv run gpt-pet run --profile sim --ticks 3   # terminal only; 0 (default) runs until Ctrl-C
uv run adk web src                           # ADK dev UI; send "tick" to run one tick
```

`gpt-pet serve` serves the built portal at http://127.0.0.1:8080/ when `ui/dist` exists, and the
API under `/api`. Pacing knobs for testing live in `[brain]`: `goal_delay_s` (rest after a tick
that starts a new goal, 60 s by default), `tick_delay_s`, and `max_goals_per_run` (10; the loop
pauses itself when reached, and Resume in the portal grants another window).

### Tests

No mocks anywhere.

```bash
cd python
uv run pytest -m "not live"   # pure functions: settings, goal memory, workflow wiring
uv run pytest -m live         # real ai2thor-mcp server (attached or launched) + real Gemini
```

The live suite attaches to a server already running on port 8000, or launches one from the
sibling `../../ai2thor-mcp` checkout (override with `AI2THOR_MCP_DIR`, scene with `AI2THOR_SCENE`).
`GPTPET_TEST_MODEL` puts another model in the profile's place for goal setter and executor, e.g.
`GPTPET_TEST_MODEL=openai/gpt-5.6-luna uv run pytest -m live` for a fraction of Gemini's cost.

## The portal (`ui/`)

A single-page management portal (React 19, Vite 8, Tailwind 4, shadcn/ui) with five panels:
goals queue, live stats, camera view, top view, and an events log of MCP tool calls plus runtime
notices (such as the rest between ticks and why it happens). Bun installs packages and runs
scripts; Vite runs on Node.

**Stats** are counted in the brain (`python/src/gpt_pet/stats.py`): an ADK plugin times every
Gemini and tool call and reads the token usage, the drive wait reports drives, and the runtime
reports ticks and goals. The panel shows LLM calls, tokens (input, output, thinking), LLM and
tick latency, goals per minute, time driving, tool calls, and System 1: its share of the model
turns (by rule or by nimble), its hand-offs to Gemini with their reasons, the decision model's
calls and latency, and the status checks the brain makes while it waits out drives. Reset starts
the counters over.

```bash
cd ui
bun install
bun run dev          # http://localhost:5173, proxies /api to the pet server on :8080
bun run build        # ui/dist, served by the pet server in production
bun run lint && bun run typecheck && bun run test
bun run test:e2e     # Playwright; the @live tier needs the real pet server and simulator
```

The portal talks to `gpt-pet serve`: `GET /api/events` (SSE), `GET /api/state`, `GET /api/status`,
`GET /api/stats`, `POST /api/stats/reset`, `POST /api/goals`, `GET /api/frame.jpg`,
`GET /api/map.png`, `GET /api/settings`, `POST /api/control/{pause|resume|tick|extend}`, and
`POST /api/profile`. Without the server the portal shows an offline state. The top view
re-renders every second while a tick runs; `stats` frames arrive at most once a second while
counters change and every 5 s otherwise.

**A submitted goal replaces the goal in progress.** `POST /api/goals` cancels whatever goal the
pet is working on: a tick running at that moment refuses its remaining tool calls ("goal
canceled") and stops a drive under way with `cancel_nav`, so it ends within a few seconds; the
next tick starts at once (no rest), records the old goal in the history as `canceled`, and adopts
the owner's goal (System 1 does this by rule, without a model call). The Events Log gets a
`cancel` notice and Stats counts canceled goals apart from abandoned ones. Goals submitted while
another owner goal is still waiting queue behind it.

**Top view = occupancy map.** ai2thor-mcp builds the map the way the real robot's SLAM stack
will: every rendered depth frame is turned into a planar range scan at camera height and
ray-cast into a log-odds grid (unexplored grey, free white, obstacles black; robot, path,
destinations and goal overlaid). The simulator supplies the pose, so it is the mapping half of
SLAM; gpt-pet-mcp will serve slam_toolbox's map in the same convention.

**Simulator-only free camera.** With `[features] free_camera = true` (the `sim` profile), the
Camera View panel gains a **Free** source: a camera you fly around the room (drag to look,
wheel to zoom, WASD/Q/E to move, arrows to look, chase/front/top presets, follow robot). It is
served by ai2thor-mcp's `sim_camera_view/move/reset` tools through `/api/sim/camera*`; the pet's
executor never sees those tools, and the `real` profile has no such option.

## The robot (`ros2_ws/`)

Two ROS 2 Humble workspaces: `ros2_ws/src/bot_ws` runs on the robot (Jetson Nano: motors, lidar, IMU),
`ros2_ws/src/server_ws` on the server (EKF, slam_toolbox, Nav2). See `CLAUDE.md` for launching both and
`bin/nav/README.md` for the navigation test tools.

### Client setup


Install submodules

```bash
# install libfreenect
sudo apt update
sudo apt install -y git cmake build-essential libusb-1.0-0-dev pkg-config
git clone https://github.com/OpenKinect/libfreenect.git
cd libfreenect
mkdir -p build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release -DBUILD_EXAMPLES=OFF
make -j"$(nproc)"
sudo make install
sudo ldconfig

# clone repo
cd ..
git clone https://github.com/pj0620/gpt-pet-v2.git
cd gpt-pet-v2

# submodules
git submodule update --init --recursive

# Install ROS2 Humble (see site)
# ..steps...

# Install colcon
sudo apt update
sudo apt install -y python3-colcon-common-extensions python3-rosdep python3-vcstool

# ROS Control
sudo apt install -y ros-humble-ros2-control ros-humble-ros2-controllers

# ROS Dep
cd ros_ws/src/bot_ws
rosdep install --from-paths src --ignore-src -r -y

# Ros Dep


```