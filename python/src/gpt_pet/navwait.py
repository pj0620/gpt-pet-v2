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
from dataclasses import dataclass, replace
from typing import TYPE_CHECKING, Any

from gpt_pet.mcp import mcp_media_callback
from gpt_pet.settings import NavSettings

if TYPE_CHECKING:
    from google.adk.tools.mcp_tool.mcp_toolset import McpToolset

    from gpt_pet.goals import GoalInterrupt
    from gpt_pet.people import PersonWatch

NAV_GOAL_TOOL = "set_nav_goal"
NAV_STATUS_TOOL = "get_nav_status"
NAV_CANCEL_TOOL = "cancel_nav"
VIEW_TOOL = "get_current_view"
TERMINAL_STATES = frozenset({"succeeded", "failed", "canceled"})
DRIVE_OUTCOMES = TERMINAL_STATES | {"preempted", "idle", "timeout", "error", "interrupted"}
"""`preempted`: another goal replaced this one. `idle`: no drive is running. `timeout`: still
driving after `max_wait_s`. `error`: get_nav_status failed, so the drive's fate is unknown.
`interrupted`: the person watch stopped the drive because a person came into view (`[people]`)."""
NULL_DRIVE_M = 0.3
"""A drive planned shorter than this goes nowhere: the robot was already there."""

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
    destination_id: str | None = None
    planned_m: float | None = None
    """The planned path's length, from the set_nav_goal response."""

    @property
    def traveled_m(self) -> float | None:
        """How far the robot drove, from the last status."""
        value = ((self.status or {}).get("progress") or {}).get("distance_traveled_m")
        return float(value) if isinstance(value, (int, float)) else None

    @property
    def null(self) -> bool:
        """The drive went nowhere: its destination was already within reach."""
        return self.planned_m is not None and self.planned_m < NULL_DRIVE_M

    def as_response(self) -> dict[str, Any]:
        """The `drive` block the model reads in the set_nav_goal response."""
        drive: dict[str, Any] = {"outcome": self.outcome, "waited_s": round(self.waited_s, 1)}
        if self.status is not None:
            drive["final_status"] = self.status
        if self.note:
            drive["note"] = self.note
        return drive


async def wait_for_drive(
    get_status: StatusFn,
    goal_id: str | None,
    nav: NavSettings,
    stop: Callable[[], str | None] | None = None,
    cancel: Callable[[], Awaitable[None]] | None = None,
    watch: Callable[[], Awaitable[str | None]] | None = None,
    watch_every_s: float = 0.0,
) -> DriveWait:
    """Check the drive every `poll_interval_s` until it ends, `max_wait_s` passes, or a check fails.
    When `stop()` gives a reason (the owner replaced the goal), `cancel()` stops the robot and the
    wait ends as "canceled" at once. Every `watch_every_s` the person watch looks around; when
    `watch()` gives a reason (a person came into view) the drive is canceled and ends "interrupted"."""
    started = time.monotonic()
    status: dict[str, Any] | None = None
    checks = 0
    looked = started
    while True:
        reason = stop() if stop is not None else None
        if reason:
            if cancel is not None:
                await cancel()
            note = f"Stopped: {reason}. The robot is no longer driving."
            return DriveWait("canceled", time.monotonic() - started, status, note, checks)
        if watch is not None and watch_every_s > 0 and time.monotonic() - looked >= watch_every_s:
            looked = time.monotonic()
            seen = await watch()
            if seen:
                if cancel is not None:
                    await cancel()
                note = f"Stopped: {seen}. The robot is no longer driving."
                return DriveWait("interrupted", time.monotonic() - started, status, note, checks)
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


def make_nav_wait_callback(
    toolset: "McpToolset",
    nav: NavSettings,
    on_drive: Callable[[DriveWait], None] | None = None,
    interrupt: "GoalInterrupt | None" = None,
    watch: "PersonWatch | None" = None,
):
    """`after_tool_callback` that returns a started drive to the model only once it has ended.

    Register it before the media callback: ADK uses the first after-tool callback that returns
    something, and this one only answers for `set_nav_goal` calls that started a drive. The
    status checks go through the same MCP session but bypass the flow, so they are neither
    events nor counted by the tool budget. `on_drive` receives every finished wait. While
    `interrupt` is requested the drive is canceled (`cancel_nav`) and the wait ends.
    """

    async def wait_out_drive(tool: Any, args: Any, tool_context: Any, tool_response: Any) -> dict[str, Any] | None:
        if getattr(tool, "name", None) != NAV_GOAL_TOOL:
            return None
        response = mcp_media_callback(tool, args, tool_context, tool_response)
        if response is None or response["isError"] or not isinstance(response["result"], dict):
            return None  # refused or failed: nothing is driving
        tools = await toolset.get_tools()
        status_tool = next((t for t in tools if t.name == NAV_STATUS_TOOL), None)
        cancel_tool = next((t for t in tools if t.name == NAV_CANCEL_TOOL), None)
        view_tool = next((t for t in tools if t.name == VIEW_TOOL), None) if watch is not None else None
        if status_tool is None:
            log.warning("%s is not available; set_nav_goal returns without waiting", NAV_STATUS_TOOL)
            return None

        def stop() -> str | None:
            return interrupt.reason if interrupt is not None and interrupt.requested else None

        async def cancel() -> None:
            if cancel_tool is None:
                log.warning("%s is not available; the canceled goal's drive keeps going", NAV_CANCEL_TOOL)
                return
            try:
                await cancel_tool.run_async(args={}, tool_context=tool_context)
            except Exception as exc:  # noqa: BLE001 - the wait ends either way
                log.warning("%s failed: %s", NAV_CANCEL_TOOL, exc)

        async def get_status() -> dict[str, Any]:
            try:
                raw = await status_tool.run_async(args={}, tool_context=tool_context)
            except Exception as exc:  # noqa: BLE001 - becomes the drive's "error" outcome for the model
                raise NavStatusError(f"{NAV_STATUS_TOOL} failed: {exc}") from exc
            return status_from_tool_result(raw)

        async def look_for_people() -> str | None:
            """One look by the person watch: a reason to stop for a person, else None."""
            from gpt_pet.goals import STATE_CURRENT_GOAL, STATE_TICK
            from gpt_pet.people import view_from_tool_result

            try:
                payload, image, mime = view_from_tool_result(await view_tool.run_async(args={}, tool_context=tool_context))
            except Exception as exc:  # noqa: BLE001 - a missed look never stops a drive
                log.warning("person watch: look failed: %s", exc)
                return None
            if payload is None or watch is None:
                return None
            state = tool_context.state
            sighting = watch.note(payload, image, mime, int(state.get(STATE_TICK) or 0), state.get(STATE_CURRENT_GOAL))
            if sighting is None and await watch.people.glance(payload, image, mime):
                sighting = watch.people.to_react(watch.people_kind, state.get(STATE_CURRENT_GOAL), watch.people.saw(payload, "watch"))
            if sighting is None:
                return None
            watch.people.counts["drive_interrupts"] += 1
            return f"a person came into view: {sighting.words()}"

        started = response["result"]
        goal_id = started.get("goal_id")
        planned = (started.get("path") or {}).get("length_m")
        watching = view_tool is not None and watch is not None
        wait = replace(
            await wait_for_drive(
                get_status,
                goal_id,
                nav,
                stop,
                cancel,
                look_for_people if watching else None,
                watch.every_s if watching and watch is not None else 0.0,
            ),
            destination_id=started.get("destination_id") or (args or {}).get("destination_id"),
            planned_m=float(planned) if isinstance(planned, (int, float)) else None,
        )
        log.info("drive %s: %s after %.1f s", goal_id, wait.outcome, wait.waited_s)
        if on_drive is not None:
            on_drive(wait)
        return {**response, "drive": wait.as_response()}

    return wait_out_drive
