"""`gpt-pet run`: the autonomous tick loop. `gpt-pet serve`: the loop plus the portal API.
`gpt-pet experiment`: timed runs per test and bot config, totals appended to a CSV.
`gpt-pet trace render DIR`: draw a recorded trace (what the pet saw, did, and where it was)."""
from __future__ import annotations

import argparse
import asyncio
import logging
import sys
from importlib import resources
from pathlib import Path

from dotenv import load_dotenv

from gpt_pet.brain import build_brain
from gpt_pet.runtime import PetRuntime, TickResult, final_texts
from gpt_pet.settings import Settings, load_settings

log = logging.getLogger("gpt_pet")

DEFAULT_PORT = 8080  # the simulator owns 8000


def load_env() -> None:
    """Load `src/gpt_pet/.env` (API key, default profile) the way `adk web` does."""
    load_dotenv(str(resources.files("gpt_pet").joinpath(".env")))


def configure_logging(verbose: bool) -> None:
    logging.basicConfig(
        level=logging.WARNING,
        format="%(asctime)s %(levelname)s %(name)s: %(message)s",
        stream=sys.stderr,
    )
    logging.getLogger("gpt_pet").setLevel(logging.DEBUG if verbose else logging.INFO)
    if verbose:
        logging.getLogger("google_adk").setLevel(logging.INFO)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(prog="gpt-pet", description="GPTPet v2: an autonomous LLM pet.")
    parser.add_argument("-v", "--verbose", action="store_true", help="debug logging, including ADK")
    commands = parser.add_subparsers(dest="command", required=True)

    run = commands.add_parser("run", help="run the autonomous tick loop in the terminal")
    run.add_argument("--profile", choices=["sim", "real"], default=None, help="settings profile (default: $GPTPET_PROFILE, else sim)")
    run.add_argument("--ticks", type=int, default=0, help="number of ticks to run; 0 runs until Ctrl-C (default)")
    run.add_argument("--user-id", default="pet", help="ADK session user id")

    serve = commands.add_parser("serve", help="run the tick loop and serve the portal API (and the built portal)")
    serve.add_argument("--profile", choices=["sim", "real"], default=None, help="settings profile (default: $GPTPET_PROFILE, else sim)")
    serve.add_argument("--host", default="127.0.0.1")
    serve.add_argument("--port", type=int, default=DEFAULT_PORT)
    serve.add_argument("--paused", action="store_true", help="start with the loop paused")
    serve.add_argument("--ui-dist", type=Path, default=None, help="built portal directory to serve at / (default: ../../ui/dist)")
    serve.add_argument("--no-launch-sim", action="store_true", help="do not launch ai2thor-mcp when nothing answers on the sim URL")
    serve.add_argument("--no-map-refresh", action="store_true", help="do not fetch the map after each tick")
    serve.add_argument("--scene", default=None, help="AI2-THOR scene when launching the simulator")
    serve.add_argument(
        "--trace",
        type=Path,
        default=None,
        help="record a trace (frames, actions, poses) into DIR/<session>; render it with `gpt-pet trace render`",
    )

    experiment = commands.add_parser(
        "experiment",
        help="run the pet for a fixed time per test and bot config and append the totals to a CSV",
    )
    experiment.add_argument("suite", nargs="?", help="suite JSON file: {name, test or tests, bots}")
    experiment.add_argument("--test", help="test config (JSON file or inline JSON), used with --bot")
    experiment.add_argument("--bot", action="append", default=[], help="bot config (JSON file or inline JSON); repeat for more bots")
    experiment.add_argument("--name", default="adhoc", help="experiment name for --test/--bot runs")
    experiment.add_argument("--repeats", type=int, default=None, help="override every test's repeats")
    experiment.add_argument(
        "--order",
        choices=["interleaved", "blocked"],
        default=None,
        help="run order of the bots (default: the suite's, else interleaved: A B, B A, ...)",
    )
    experiment.add_argument("--results", type=Path, default=None, help="results directory (default: gpt-pet-v2/experiments/results)")
    experiment.add_argument("--summary", action="store_true", help="only print the summary of the results so far")
    experiment.add_argument(
        "--trace", action="store_true", help="record and render a trace of every run (runs/<run_id>.trace/), as `trace: true` in a test does"
    )

    trace = commands.add_parser("trace", help="work with recorded traces (gpt_pet.trace)")
    trace_commands = trace.add_subparsers(dest="trace_command", required=True)
    render = trace_commands.add_parser("render", help="draw a trace: timeline.txt, storyboard-NN.png, trajectory.png, index.html")
    render.add_argument("directory", type=Path, help="a trace directory (holds trace.jsonl)")
    render.add_argument("--last", type=int, default=None, help="only the last N views in the storyboard")
    attach = trace_commands.add_parser("attach", help="record a trace of a running `gpt-pet serve` without touching it")
    attach.add_argument("--portal", default=f"http://127.0.0.1:{DEFAULT_PORT}", help="the pet server (default: %(default)s)")
    attach.add_argument("--out", type=Path, default=Path("traces"), help="directory for attach-<time>/ (default: %(default)s)")
    attach.add_argument("--sim", default=None, help="MCP URL for ground truth (default: the pet's own, from /api/status)")
    attach.add_argument("--seconds", type=float, default=None, help="stop after this long (default: until Ctrl-C)")
    attach.add_argument("--no-render", action="store_true", help="do not draw the trace when recording stops")
    return parser


def trace(args: argparse.Namespace) -> int:
    from gpt_pet.trace_report import render

    if args.trace_command == "attach":
        from gpt_pet.trace_attach import attach

        try:
            directory = asyncio.run(attach(args.portal, args.out, sim_url=args.sim, seconds=args.seconds))
        except KeyboardInterrupt:
            directories = sorted(Path(args.out).glob("attach-*"))
            directory = directories[-1] if directories else None
        if directory is None:
            return 1
        print(f"trace: {directory}")
        if args.no_render:
            return 0
        args.directory, args.last = directory, None
    out = render(args.directory, last=args.last)
    print(f"timeline:   {out['timeline']}")
    print(f"trajectory: {out['trajectory']}")
    for sheet in out["storyboards"]:
        print(f"storyboard: {sheet}")
    print(f"page:       {out['html']}")
    return 0


def experiment(args: argparse.Namespace) -> int:
    from gpt_pet.experiment import (
        DEFAULT_RESULTS_DIR,
        RESULTS_CSV,
        build_suite,
        format_summary,
        run_suite,
        write_summary,
    )

    results_dir = args.results or DEFAULT_RESULTS_DIR
    if args.summary:
        print(format_summary(write_summary(results_dir)))
        return 0
    suite = build_suite(args.suite, test=args.test, bots=args.bot, name=args.name, repeats=args.repeats, order=args.order)

    def report(row: dict) -> None:
        outcome = f"error: {row['error']}" if row["error"] else (
            f"{row['duration_s']} s, {row['llm_calls']} LLM calls, {row['tokens_total']} tokens, "
            f"{row['s1_turns']} S1 turns, {row['goals_done']} goals done"
        )
        log.info("%s: %s", row["run_id"], outcome)

    if args.trace:
        suite = suite.model_copy(update={"tests": [test.model_copy(update={"trace": True}) for test in suite.tests]})
    rows = asyncio.run(run_suite(suite, results_dir=results_dir, on_row=report))
    print(format_summary(write_summary(results_dir), experiment=suite.name))
    print(f"\n{len(rows)} runs appended to {results_dir / RESULTS_CSV}")
    return 1 if any(row["error"] for row in rows) else 0


def log_tick(result: TickResult) -> None:
    for author, text in final_texts(result.events):
        log.info("%s: %s", author, text)
    log.info(result.summary())


async def run_ticks(settings: Settings, *, ticks: int, user_id: str) -> int:
    brain = build_brain(settings)
    log.info(
        "profile=%s transport=%s mcp=%s models=%s/%s",
        settings.profile.name,
        settings.mcp.transport,
        getattr(settings.mcp, "url", None) or getattr(settings.mcp, "command", None),
        settings.model.goal_setter,
        settings.model.executor,
    )
    async with PetRuntime(brain, user_id=user_id, on_tick=log_tick) as pet:
        return await pet.run_loop(max_ticks=ticks)


def serve(settings: Settings, args: argparse.Namespace) -> int:
    import uvicorn

    from gpt_pet.server import create_app
    from gpt_pet.simlaunch import ensure_mcp_server, stop_mcp_server

    sim_process = None
    url = getattr(settings.mcp, "url", None)
    if url is not None and settings.profile.name == "sim" and not args.no_launch_sim:
        log_path = Path(str(resources.files("gpt_pet"))) / ".adk" / "ai2thor-mcp.log"
        log_path.parent.mkdir(parents=True, exist_ok=True)
        sim_process = ensure_mcp_server(str(url), scene=args.scene, log_path=log_path)
    app = create_app(
        settings, ui_dist=args.ui_dist, start_paused=args.paused, refresh_map=not args.no_map_refresh, trace_dir=args.trace
    )
    log.info("portal API on http://%s:%d/api (portal at / when ui/dist exists)", args.host, args.port)
    try:
        uvicorn.run(app, host=args.host, port=args.port, log_level="info" if args.verbose else "warning")
    finally:
        stop_mcp_server(sim_process)
    return 0


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    configure_logging(args.verbose)
    load_env()
    if args.command == "experiment":
        return experiment(args)
    if args.command == "trace":
        return trace(args)
    settings = load_settings(args.profile)
    try:
        if args.command == "serve":
            return serve(settings, args)
        asyncio.run(run_ticks(settings, ticks=args.ticks, user_id=args.user_id))
    except KeyboardInterrupt:
        log.info("interrupted; shut down cleanly")
        return 130
    return 0


if __name__ == "__main__":
    sys.exit(main())
