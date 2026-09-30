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
