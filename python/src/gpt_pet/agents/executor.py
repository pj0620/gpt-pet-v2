"""The executor: pursues the active goal with the robot's MCP tools and reports back.

Its input is the active goal record (JSON) handed over by the workflow; its output is a short
plain-text report stored in session state under `last_report` for the next goal-setter turn.

Tick latency is bounded deterministically: `ToolBudget` counts tool calls per invocation (one
tick) and refuses calls over the `[tool_limits]` caps with a message telling the model to stop
and report. The prompt asks for restraint; the budget guarantees it.

Drives cost no LLM calls: `set_nav_goal` returns to the model only once the drive has ended
(`gpt_pet.navwait`).
"""
from __future__ import annotations

import logging
from collections.abc import Callable, Mapping
from typing import TYPE_CHECKING, Any

from google.adk.agents.llm_agent import LlmAgent
from google.adk.tools.mcp_tool.mcp_toolset import McpToolset
from google.genai import types

from gpt_pet.goals import STATE_LAST_REPORT
from gpt_pet.frames import FrameStore
from gpt_pet.mcp import make_media_callback
from gpt_pet.navwait import make_nav_wait_callback
from gpt_pet.prompts import load_prompt
from gpt_pet.settings import Settings

if TYPE_CHECKING:
    from gpt_pet.stats import RunStats

EXECUTOR_NAME = "executor"
BUDGET_MARKER = "budget exhausted"

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
    return isinstance(response, dict) and BUDGET_MARKER in str(response.get("error", ""))


def make_tool_budget_callback(limits: Mapping[str, int]):
    """`before_tool_callback` that enforces the caps; a returned dict replaces the tool call."""
    budget = ToolBudget(limits)

    def before_tool(tool, args, tool_context) -> dict[str, Any] | None:
        refusal = budget.admit(tool_context.invocation_id, tool.name)
        if refusal is None:
            return None
        log.info("refused %s: %s", tool.name, refusal)
        return {"error": refusal}

    return before_tool


def build_executor(
    settings: Settings,
    toolset: McpToolset,
    frames: FrameStore | None = None,
    stats: "RunStats | None" = None,
    before_model_callback: Callable[..., Any] | None = None,
) -> LlmAgent:
    """`before_model_callback` is System 1 (`gpt_pet.s1`), when enabled."""
    prompt = load_prompt("executor")
    on_drive = stats.record_drive if stats is not None else None

    def instruction(ctx) -> str:  # callable: bypasses `{key}` state templating
        return prompt

    return LlmAgent(
        name=EXECUTOR_NAME,
        model=settings.model.executor,
        description="Executes the current goal with the robot's tools and reports what happened.",
        instruction=instruction,
        tools=[toolset],
        before_tool_callback=make_tool_budget_callback(settings.tool_limits),
        # First non-None wins: the drive wait answers only for set_nav_goal, media for the rest.
        after_tool_callback=[make_nav_wait_callback(toolset, settings.nav, on_drive), make_media_callback(frames)],
        output_key=STATE_LAST_REPORT,
        include_contents="none",
        generate_content_config=types.GenerateContentConfig(temperature=settings.model.temperature),
        before_model_callback=before_model_callback,
    )
