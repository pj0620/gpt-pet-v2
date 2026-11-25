"""The executor: pursues the active goal with the robot's MCP tools and reports back.

Its input is the active goal record (JSON) handed over by the workflow; its output is a short
plain-text report stored in session state under `last_report` for the next goal-setter turn.

Tick latency is bounded deterministically: `ToolBudget` counts tool calls per invocation (one
tick) and refuses calls over the `[tool_limits]` caps with a message telling the model to stop
and report. The prompt asks for restraint; the budget guarantees it.

Drives cost no LLM calls: `set_nav_goal` returns to the model only once the drive has ended
(`gpt_pet.navwait`).

When the owner submits a goal mid-tick (`GoalInterrupt`), every further tool call is refused with
CANCEL_MARKER and a drive being waited out is stopped, so the tick ends within seconds.
"""
from __future__ import annotations

import logging
from collections.abc import Callable, Mapping
from typing import TYPE_CHECKING, Any

from google.adk.agents.llm_agent import LlmAgent
from google.adk.tools.mcp_tool.mcp_toolset import McpToolset

from gpt_pet.agents.content import content_config
from gpt_pet.agents.models import agent_model
from gpt_pet.goals import STATE_LAST_REPORT, STATE_TICK, GoalInterrupt
from gpt_pet.frames import FrameStore
from gpt_pet.mcp import make_media_callback
from gpt_pet.navwait import DriveWait, make_nav_wait_callback
from gpt_pet.people import ATTENTION_LIMITS, People, PersonWatch, make_attention_wait
from gpt_pet.prompts import load_prompt
from gpt_pet.settings import NavSettings, Settings

if TYPE_CHECKING:
    from gpt_pet.places import Places
    from gpt_pet.stats import RunStats

EXECUTOR_NAME = "executor"
BUDGET_MARKER = "budget exhausted"
CANCEL_MARKER = "goal canceled"
HERE_MARKER = "already here"
FAILED_MARKER = "failed from here"
DRIVE_TOOL = "set_nav_goal"
PLACES_GUIDANCE = """

The pet remembers places. In a view, "here" marks destinations already within reach (a drive
there goes nowhere), "visited" marks ones reached on an earlier drive, and "failed" counts failed
drives there. Prefer destinations with none of these marks, and passages that lead somewhere
new. In your report, name the passages in view and their direction."""
TARGET_GUIDANCE = """
If the goal has a target_id, drive there first with set_nav_goal (the id stays valid even when
the place is out of view), unless a view marks it "here": then the pet is already there."""
FAILED_GUIDANCE = """
"failed_here" marks a destination a drive already failed to reach from this spot; a drive there
from here is refused."""
FAILURE_GUIDANCE = """

When a drive fails, do not drive to the same destination again from the same spot: back up
(do_move MoveBack 0.5) or turn, look again, and pick another destination or route."""

PEOPLE_GUIDANCE = """

People: a destination labeled Person is a person, and people move. To engage someone, call
set_attention with their id: the robot turns to them, tilts its camera for their distance and
walks up to about 1.3 m by itself, and the call returns once they are framed (or lost). Then
call get_current_view and say in your report what the person is doing: standing, walking or
sitting, gestures, what they hold, where they face. Describe only what the image shows clearly;
say "unclear" otherwise. While attention is on, do not call do_look, do_rotate or do_move:
attention aims and tilts the camera itself, and any of those ends it. Never drive to a person
with set_nav_goal; a drive ends attention too. A drive that ends "interrupted" was stopped
because a person came into view: look, and report them."""

ENGAGE_GUIDANCE = """

Whether someone wants to interact: turning to a person whose interest is unknown (set_attention)
also watches them for a few seconds; the set_attention response then has an "engagement" block:
"interested" (with what they did: waved, beckoned, crouched down, came over) or "not_interested"
(attention has ended; leave them be and carry on with the goal). To interact with someone who
wants to: set_attention with standoff_m 0.9 and greet true walks up close, looks up at them and
wiggles hello; with follow true the robot keeps up with them when they walk off (when they beckoned
and walk away, follow them). One set_attention call is enough: the robot keeps at it by itself, and
calling again only changes the options. Then get_current_view (more than once if they move) and
report how they respond: what they do, whether they keep looking at you, crouch, gesture or leave.
If they point at something, you may drive to it with set_nav_goal."""

log = logging.getLogger("gpt_pet.executor")


class ToolBudget:
    """Per-invocation tool call counter with caps. Pure Python; no ADK state involved."""

    def __init__(self, limits: Mapping[str, int]) -> None:
        self.limits = dict(limits)
        self._invocation_id: str | None = None
        self._counts: dict[str, int] = {}

    def counts_for(self, invocation_id: str) -> dict[str, int]:
        if invocation_id != self._invocation_id:
            self._invocation_id = invocation_id
            self._counts = {}
        return self._counts

    def admit(self, invocation_id: str, tool_name: str) -> str | None:
        """Record one call. Returns None when allowed, otherwise the refusal message."""
        counts = self.counts_for(invocation_id)
        used = counts.get(tool_name, 0)
        limit = self.limits.get(tool_name)
        if limit is not None and used >= limit:
            return (
                f"{BUDGET_MARKER}: {tool_name} was already called {used} times this tick "
                f"(limit {limit}). Do not call it again; finish now and write your report."
            )
        counts[tool_name] = used + 1
        return None


def is_budget_refusal(response: Any) -> bool:
    """A call the brain's own guards refused: over the tool budget, or a drive to a place the
    robot is already at. The executor's caps working, not a tool failure."""
    if not isinstance(response, dict):
        return False
    error = str(response.get("error", ""))
    return BUDGET_MARKER in error or HERE_MARKER in error or FAILED_MARKER in error or CANCEL_MARKER in error


def is_cancel_refusal(response: Any) -> bool:
    """A call refused because the owner canceled the goal this tick was working on."""
    return isinstance(response, dict) and CANCEL_MARKER in str(response.get("error", ""))


def cancel_refusal(reason: str) -> str:
    return f"{CANCEL_MARKER}: {reason}. Do not call any more tools; finish now and write your report."


def here_refusal(places: "Places", destination_id: str) -> str | None:
    """Why a drive to `destination_id` is refused: the robot is already there. None to allow."""
    if not places.is_here(destination_id):
        return None
    label = places.destinations[destination_id].label
    return (
        f"{HERE_MARKER}: {label} is within reach, so a drive there would go nowhere. Choose a "
        "destination not marked here, or turn to see more."
    )


def failed_refusal(places: "Places", destination_id: str) -> str | None:
    """Why a drive to `destination_id` is refused: one already failed from here. None to allow."""
    if not places.failed_guard or not places.failed_here(destination_id):
        return None
    label = places.destinations[destination_id].label
    return (
        f"{FAILED_MARKER}: a drive to {label} already failed from this spot. Back up or turn first, "
        "or choose another destination."
    )


def make_tool_budget_callback(
    limits: Mapping[str, int], places: "Places | None" = None, interrupt: GoalInterrupt | None = None
):
    """`before_tool_callback` that enforces the caps and, with the places memory, refuses drives
    to where the robot already is (without spending a drive); a returned dict replaces the call.
    While `interrupt` is requested (the owner replaced the goal), every call is refused."""
    budget = ToolBudget(limits)

    def before_tool(tool, args, tool_context) -> dict[str, Any] | None:
        if interrupt is not None and interrupt.requested:
            refusal = cancel_refusal(interrupt.reason or "the owner canceled this goal")
            log.info("refused %s: %s", tool.name, refusal)
            return {"error": refusal}
        refusal = None
        if places is not None and tool.name == DRIVE_TOOL:
            destination_id = str((args or {}).get("destination_id") or "")
            refusal = here_refusal(places, destination_id) or failed_refusal(places, destination_id)
        refusal = refusal or budget.admit(tool_context.invocation_id, tool.name)
        if refusal is None:
            return None
        log.info("refused %s: %s", tool.name, refusal)
        return {"error": refusal}

    return before_tool


def make_after_tool_callback(
    toolset: McpToolset,
    nav: NavSettings,
    frames: FrameStore | None = None,
    places: "Places | None" = None,
    on_drive: Callable[[DriveWait], None] | None = None,
    interrupt: GoalInterrupt | None = None,
    people: People | None = None,
    watch: PersonWatch | None = None,
    attention_wait_s: float = 15.0,
    people_settings: Any = None,
    reader: Any = None,
):
    """The executor's one `after_tool_callback`: the drive wait (set_nav_goal returns once the
    drive is over), else the media conversion, then the places memory, which learns from the
    shaped response and marks it. One composed callback, because ADK uses the first after-tool
    callback that returns something: a separate observer would never see a waited-out drive."""
    wait_out_drive = make_nav_wait_callback(toolset, nav, on_drive, interrupt, watch)
    wait_for_attention = None
    if people is not None:
        from gpt_pet.settings import PeopleSettings

        wait_for_attention = make_attention_wait(
            toolset, people_settings or PeopleSettings(attention_wait_s=attention_wait_s), people, interrupt, reader, watch
        )
    media = make_media_callback(frames)

    async def after_tool(tool: Any, args: Any, tool_context: Any, tool_response: Any) -> dict[str, Any] | None:
        response = await wait_out_drive(tool, args, tool_context, tool_response)
        if response is None and wait_for_attention is not None:
            response = await wait_for_attention(tool, args, tool_context, tool_response)
        if response is None:
            response = media(tool, args, tool_context, tool_response)
        if people is not None and getattr(tool, "name", "") == "get_current_view" and isinstance(response, dict) and not response.get("isError"):
            view = response.get("result") if isinstance(response.get("result"), dict) else None
            people.saw(view, "executor")
            if people.reader is not None:
                parts = response.get("media") or []
                blob = getattr(parts[0], "inline_data", None) if parts else None
                if blob is not None and blob.data:
                    await people.glance(view, bytes(blob.data), blob.mime_type)
        if places is None or not isinstance(response, dict):
            return response
        tick = int(tool_context.state.get(STATE_TICK) or 0)
        return places.remember(str(getattr(tool, "name", "")), args, response, tick)

    return after_tool


def build_executor(
    settings: Settings,
    toolset: McpToolset,
    frames: FrameStore | None = None,
    stats: "RunStats | None" = None,
    before_model_callback: Callable[..., Any] | None = None,
    on_model_error_callback: Callable[..., Any] | None = None,
    places: "Places | None" = None,
    interrupt: GoalInterrupt | None = None,
    people: People | None = None,
    watch: PersonWatch | None = None,
    reader: Any = None,
) -> LlmAgent:
    """`before_model_callback` is System 1 (`gpt_pet.s1`), when enabled;
    `on_model_error_callback` stands in for a failed Gemini call (`gpt_pet.agents.fallbacks`);
    `places` is the pet's memory of places (`[brain] places_memory`), which marks every view;
    `interrupt` ends the tick early when the owner replaces the goal."""
    prompt = load_prompt("executor")
    if places is not None:
        prompt += PLACES_GUIDANCE + (TARGET_GUIDANCE if settings.brain.map_for_goals else "")
        prompt += FAILED_GUIDANCE if settings.brain.failed_guard else ""
    if settings.brain.failure_guidance:
        prompt += FAILURE_GUIDANCE
    if people is not None:
        prompt += PEOPLE_GUIDANCE
        if people.social is not None:
            prompt += ENGAGE_GUIDANCE
    limits = {**ATTENTION_LIMITS, **settings.tool_limits} if people is not None else dict(settings.tool_limits)
    on_drive = stats.record_drive if stats is not None else None

    def instruction(ctx) -> str:  # callable: bypasses `{key}` state templating
        return prompt

    return LlmAgent(
        name=EXECUTOR_NAME,
        model=agent_model(settings.model.executor, settings.model),
        description="Executes the current goal with the robot's tools and reports what happened.",
        instruction=instruction,
        tools=[toolset],
        before_tool_callback=make_tool_budget_callback(limits, places, interrupt),
        after_tool_callback=make_after_tool_callback(
            toolset, settings.nav, frames, places, on_drive, interrupt, people, watch, settings.people.attention_wait_s,
            settings.people, reader,
        ),
        output_key=STATE_LAST_REPORT,
        include_contents="none",
        generate_content_config=content_config(settings.model, settings.model.executor),
        before_model_callback=before_model_callback,
        on_model_error_callback=on_model_error_callback,
    )
