"""Waiting out a drive without the model.

`set_nav_goal` only starts a drive: like nav2's NavigateToPose, the goal runs in the background
and `get_nav_status` reports on it. Left to the executor, every status check is an LLM turn whose
only possible decision is "check again". The after-tool callback built here polls
`get_nav_status` itself and hands the model the drive's outcome inside the `set_nav_goal`
response, so no LLM calls are made while the robot drives. The model is called again once the
drive has succeeded, failed or been canceled, or after `[nav] max_wait_s`.

Detecting a stuck robot is the navigation stack's job (ai2thor-mcp fails a blocked drive with a
reason; nav2 has progress checkers), so waiting for a terminal state is enough.
"""
from __future__ import annotations

import asyncio
import logging
import time
from collections.abc import Awaitable, Callable
from dataclasses import dataclass
from typing import TYPE_CHECKING, Any

from gpt_pet.mcp import mcp_media_callback
from gpt_pet.settings import NavSettings

if TYPE_CHECKING:
    from google.adk.tools.mcp_tool.mcp_toolset import McpToolset

NAV_GOAL_TOOL = "set_nav_goal"
NAV_STATUS_TOOL = "get_nav_status"
TERMINAL_STATES = frozenset({"succeeded", "failed", "canceled"})
DRIVE_OUTCOMES = TERMINAL_STATES | {"preempted", "idle", "timeout", "error"}
"""`preempted`: another goal replaced this one. `idle`: no drive is running. `timeout`: still
driving after `max_wait_s`. `error`: get_nav_status failed, so the drive's fate is unknown."""

log = logging.getLogger("gpt_pet.navwait")

StatusFn = Callable[[], Awaitable[dict[str, Any]]]


class NavStatusError(RuntimeError):
    """get_nav_status failed or answered with something that is not a nav status."""


def status_from_tool_result(raw: Any) -> dict[str, Any]:
    """The status dict inside a dumped `get_nav_status` result (the shape ADK hands callbacks)."""
    if isinstance(raw, dict) and "error" in raw and "content" not in raw:
        raise NavStatusError(str(raw["error"]))
    parsed = mcp_media_callback(None, {}, None, raw)
    if parsed is None:
        raise NavStatusError(f"unexpected {NAV_STATUS_TOOL} result: {raw!r:.200}")
    if parsed["isError"]:
        raise NavStatusError(f"{NAV_STATUS_TOOL} failed: {parsed['result']!r:.200}")
    status = parsed["result"]
    if not isinstance(status, dict) or not isinstance(status.get("state"), str):
        raise NavStatusError(f"{NAV_STATUS_TOOL} answered without a state: {status!r:.200}")
    return status


def drive_outcome(status: dict[str, Any], goal_id: str | None) -> str | None:
    """How the drive `goal_id` ended according to one status, or None while it is underway."""
    current = (status.get("goal") or {}).get("goal_id")
    if goal_id and current and current != goal_id:
        return "preempted"
    state = status["state"]
    if state in TERMINAL_STATES:
        return state
    if state == "idle":
        return "idle"
    return None


@dataclass(frozen=True)
class DriveWait:
    outcome: str
    waited_s: float
    status: dict[str, Any] | None
    """The last status seen; None when the first check already failed."""
    note: str | None = None
    checks: int = 0
    """get_nav_status calls the brain made instead of the model (for the run stats, not the model)."""

    def as_response(self) -> dict[str, Any]:
        """The `drive` block the model reads in the set_nav_goal response."""
        drive: dict[str, Any] = {"outcome": self.outcome, "waited_s": round(self.waited_s, 1)}
        if self.status is not None:
            drive["final_status"] = self.status
        if self.note:
            drive["note"] = self.note
        return drive


async def wait_for_drive(get_status: StatusFn, goal_id: str | None, nav: NavSettings) -> DriveWait:
    """Check the drive every `poll_interval_s` until it ends, `max_wait_s` passes, or a check fails."""
    started = time.monotonic()
    status: dict[str, Any] | None = None
    checks = 0
    while True:
        checks += 1
        try:
            status = await get_status()
        except NavStatusError as exc:
            note = f"{exc}. The drive may still be running."
            return DriveWait("error", time.monotonic() - started, status, note, checks)
        waited = time.monotonic() - started
        outcome = drive_outcome(status, goal_id)
        if outcome is not None:
            return DriveWait(outcome, waited, status, checks=checks)
        if waited >= nav.max_wait_s:
            note = f"Still driving after {nav.max_wait_s:g} s; stopped waiting. The drive continues."
            return DriveWait("timeout", waited, status, note, checks)
        await asyncio.sleep(min(nav.poll_interval_s, nav.max_wait_s - waited))


def make_nav_wait_callback(toolset: "McpToolset", nav: NavSettings, on_drive: Callable[[DriveWait], None] | None = None):
    """`after_tool_callback` that returns a started drive to the model only once it has ended.

    Register it before the media callback: ADK uses the first after-tool callback that returns
    something, and this one only answers for `set_nav_goal` calls that started a drive. The
    status checks go through the same MCP session but bypass the flow, so they are neither
    events nor counted by the tool budget. `on_drive` receives every finished wait.
    """

    async def wait_out_drive(tool: Any, args: Any, tool_context: Any, tool_response: Any) -> dict[str, Any] | None:
        if getattr(tool, "name", None) != NAV_GOAL_TOOL:
            return None
        response = mcp_media_callback(tool, args, tool_context, tool_response)
        if response is None or response["isError"] or not isinstance(response["result"], dict):
            return None  # refused or failed: nothing is driving
        status_tool = next((t for t in await toolset.get_tools() if t.name == NAV_STATUS_TOOL), None)
        if status_tool is None:
            log.warning("%s is not available; set_nav_goal returns without waiting", NAV_STATUS_TOOL)
            return None

        async def get_status() -> dict[str, Any]:
            try:
                raw = await status_tool.run_async(args={}, tool_context=tool_context)
            except Exception as exc:  # noqa: BLE001 - becomes the drive's "error" outcome for the model
                raise NavStatusError(f"{NAV_STATUS_TOOL} failed: {exc}") from exc
            return status_from_tool_result(raw)

        goal_id = response["result"].get("goal_id")
        wait = await wait_for_drive(get_status, goal_id, nav)
        log.info("drive %s: %s after %.1f s", goal_id, wait.outcome, wait.waited_s)
        if on_drive is not None:
            on_drive(wait)
        return {**response, "drive": wait.as_response()}

    return wait_out_drive
