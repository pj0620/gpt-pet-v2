"""The goal setter: decides what the pet wants next and closes finished goals.

It has no tools. It reads the goal memory and the executor's last report from session state and
emits a `GoalDecision`. The prompt is rendered by a callable instruction (so ADK's `{key}` state
templating is bypassed and the prompt file may contain braces).
"""
from __future__ import annotations

import json
from collections.abc import Callable, Mapping
from string import Template
from typing import Any

from google.adk.agents.llm_agent import LlmAgent
from google.adk.agents.readonly_context import ReadonlyContext
from google.genai import types

from gpt_pet.goals import (
    STATE_CURRENT_GOAL,
    STATE_GOAL_DECISION,
    STATE_GOAL_HISTORY,
    STATE_LAST_REPORT,
    STATE_PENDING_GOALS,
    GoalDecision,
)
from gpt_pet.prompts import load_prompt
from gpt_pet.settings import Settings

GOAL_SETTER_NAME = "goal_setter"


def _dump(value: Any) -> str:
    if value is None or value == "" or value == [] or value == {}:
        return "none"
    if isinstance(value, str):
        return value
    return json.dumps(value, ensure_ascii=False, indent=2)


def render_goal_setter_prompt(state: Mapping[str, Any], settings: Settings) -> str:
    template = Template(load_prompt("goal_setter"))
    return template.safe_substitute(
        current_goal=_dump(state.get(STATE_CURRENT_GOAL)),
        last_report=_dump(state.get(STATE_LAST_REPORT)),
        goal_history=_dump(state.get(STATE_GOAL_HISTORY)),
        pending_goals=_dump(state.get(STATE_PENDING_GOALS)),
        max_goal_attempts=str(settings.brain.max_goal_attempts),
        default_goal=settings.brain.default_goal,
    )


def build_goal_setter(settings: Settings, before_model_callback: Callable[..., Any] | None = None) -> LlmAgent:
    """`before_model_callback` is System 1's gate (`gpt_pet.s1`), when enabled."""

    def instruction(ctx: ReadonlyContext) -> str:
        return render_goal_setter_prompt(ctx.state, settings)

    return LlmAgent(
        name=GOAL_SETTER_NAME,
        model=settings.model.goal_setter,
        description="Decides what the pet wants to do next and closes finished goals.",
        instruction=instruction,
        output_schema=GoalDecision,
        output_key=STATE_GOAL_DECISION,
        include_contents="none",
        generate_content_config=types.GenerateContentConfig(temperature=settings.model.temperature),
        before_model_callback=before_model_callback,
    )
