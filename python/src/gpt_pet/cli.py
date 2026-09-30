"""`gpt-pet run`: the autonomous tick loop. `gpt-pet serve`: the loop plus the portal API."""
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
    return parser


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
    app = create_app(settings, ui_dist=args.ui_dist, start_paused=args.paused, refresh_map=not args.no_map_refresh)
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
