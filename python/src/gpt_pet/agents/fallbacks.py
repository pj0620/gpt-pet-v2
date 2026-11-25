"""Stand-ins for Gemini calls that fail.

A Gemini call can hang (one run lost 137 s to a single call; `gpt_pet.agents.content` now sets a
timeout) or fail transiently (timeout, dropped connection, rate limit, server error). The agents'
`on_model_error_callback`s turn such a failure into a response so it costs one turn, not the tick:

- executor: a short report of what was done so far;
- goal setter: "continue the current goal", only while a goal is active, so a fallback never
  writes a goal. Without an active goal the error propagates and the runtime retries next tick.
"""
from __future__ import annotations

import asyncio
from collections.abc import Mapping
from typing import Any

import httpx
from google.adk.models.llm_response import LlmResponse
from google.genai import errors as genai_errors
from google.genai import types

from gpt_pet.agents.model_api import ModelApiError
from gpt_pet.goals import STATE_CURRENT_GOAL, GoalDecision
from gpt_pet.s1 import LOOK, describe, tool_steps

try:
    import aiohttp

    _CONNECTION_ERRORS: tuple[type[BaseException], ...] = (aiohttp.ClientError, httpx.TransportError)
except ImportError:  # genai falls back to httpx without aiohttp
    _CONNECTION_ERRORS = (httpx.TransportError,)

RETRYABLE_STATUS = frozenset({408, 429})


def is_transient_model_error(error: BaseException) -> bool:
    """A call that may well succeed next turn: a timeout, a dropped connection, a rate limit or a
    server error. Never a cancellation or a bad request."""
    if isinstance(error, BaseExceptionGroup):
        return all(is_transient_model_error(inner) for inner in error.exceptions)
    if isinstance(error, asyncio.CancelledError):
        return False
    if isinstance(error, (TimeoutError, *_CONNECTION_ERRORS, genai_errors.ServerError)):
        return True
    if isinstance(error, ModelApiError):  # another provider's model, or a local one
        return error.transient
    return isinstance(error, genai_errors.ClientError) and error.code in RETRYABLE_STATUS


def error_name(error: BaseException) -> str:
    """Short and log-friendly: the exception type, plus the HTTP status for API errors."""
    if isinstance(error, BaseExceptionGroup):
        return ", ".join(sorted({error_name(inner) for inner in error.exceptions}))
    if isinstance(error, genai_errors.APIError):
        return f"{type(error).__name__} {error.code}"
    if isinstance(error, ModelApiError):
        return f"{type(error).__name__} {error.status}"
    return type(error).__name__


def model_name(llm_request: Any) -> str:
    """Who failed, as a report says it: Gemini, or the local model's tag."""
    name = str(getattr(llm_request, "model", None) or "")
    return name if name and not name.startswith("gemini") else "Gemini"


def _tagged(agent: str, error: BaseException, text: str) -> LlmResponse:
    return LlmResponse(
        content=types.Content(role="model", parts=[types.Part(text=text)]),
        custom_metadata={"fallback": {"agent": agent, "error": error_name(error)}},
    )


def executor_fallback_report(contents: list[types.Content], error: BaseException, model: str = "Gemini") -> str:
    """The executor's report when Gemini fails mid-stretch: what the robot did until then."""
    steps = tool_steps(contents)
    did = "; ".join(describe(step) for step in steps if step.name != LOOK) or ("looked around" if steps else "nothing yet")
    return (
        f"{model} did not answer ({error_name(error)}); this stretch stopped early.\n"
        f"Did: {did}.\nSuccess criteria not checked."
    )


def goal_setter_fallback_decision(state: Mapping[str, Any], error: BaseException, model: str = "Gemini") -> GoalDecision | None:
    """Keep the active goal when Gemini fails; None (no stand-in) when there is no goal to keep."""
    goal = state.get(STATE_CURRENT_GOAL)
    if not isinstance(goal, dict) or goal.get("status") != "active":
        return None
    reason = f"{model} did not answer ({error_name(error)}); keeping the current goal."
    return GoalDecision(previous_goal_status="continue", reasoning=reason)


async def executor_on_model_error(callback_context: Any, llm_request: Any, error: Exception) -> LlmResponse | None:
    if not is_transient_model_error(error):
        return None
    return _tagged("executor", error, executor_fallback_report(llm_request.contents, error, model_name(llm_request)))


async def goal_setter_on_model_error(callback_context: Any, llm_request: Any, error: Exception) -> LlmResponse | None:
    if not is_transient_model_error(error):
        return None
    decision = goal_setter_fallback_decision(callback_context.state, error, model_name(llm_request))
    if decision is None:
        return None
    return _tagged("goal_setter", error, decision.model_dump_json(exclude_none=True))
