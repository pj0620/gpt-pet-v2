"""Offline: what stands in for a failed Gemini call, on real exception and genai objects. The
failures themselves (a real timeout against real Gemini) are exercised live
(tests/live/test_fallbacks_live.py)."""
from __future__ import annotations

import asyncio

import aiohttp
import httpx
from google.adk.events.event import Event
from google.adk.models.llm_request import LlmRequest
from google.genai import errors as genai_errors
from google.genai import types

from gpt_pet.agents.fallbacks import (
    error_name,
    executor_fallback_report,
    executor_on_model_error,
    goal_setter_fallback_decision,
    is_transient_model_error,
)
from gpt_pet.goals import STATE_CURRENT_GOAL, GoalDecision
from gpt_pet.runtime import TickResult, is_fallback_event, model_call_count

GOAL = {"id": 3, "goal": "find the fridge", "success_criteria": "The fridge is in view", "status": "active", "attempts": 1}


def api_error(kind: type[genai_errors.APIError], code: int) -> genai_errors.APIError:
    return kind(code, {"error": {"code": code, "message": "from a test", "status": "X"}})


def call(name: str, args: dict, call_id: str) -> types.Content:
    return types.Content(role="model", parts=[types.Part(function_call=types.FunctionCall(id=call_id, name=name, args=args))])


def answer(name: str, response: dict, call_id: str) -> types.Content:
    return types.Content(
        role="user", parts=[types.Part(function_response=types.FunctionResponse(id=call_id, name=name, response=response))]
    )


def stretch() -> list[types.Content]:
    """An executor tick so far: a look, a drive that arrived, a turn."""
    drive = {
        "result": {"goal_id": "g1", "label": "Fridge", "path": {"length_m": 2.4}},
        "isError": False,
        "drive": {"outcome": "succeeded", "waited_s": 6.1},
    }
    return [
        types.Content(role="user", parts=[types.Part(text='{"goal": "find the fridge"}')]),
        call("get_current_view", {}, "c1"),
        answer("get_current_view", {"result": {"destinations": []}, "isError": False}, "c1"),
        call("set_nav_goal", {"destination_id": "obj:Fridge|1"}, "c2"),
        answer("set_nav_goal", drive, "c2"),
        call("do_rotate", {"action": "RotateLeft", "degrees": 90}, "c3"),
        answer("do_rotate", {"result": "Rotated", "isError": False}, "c3"),
    ]


def test_timeouts_dropped_connections_rate_limits_and_server_errors_are_transient() -> None:
    for error in (
        TimeoutError(),
        asyncio.TimeoutError(),
        aiohttp.ServerTimeoutError("read timed out"),
        aiohttp.ClientConnectionError("connection reset"),
        httpx.ConnectError("refused"),
        httpx.ReadTimeout("slow"),
        api_error(genai_errors.ServerError, 503),
        api_error(genai_errors.ServerError, 500),
        api_error(genai_errors.ClientError, 429),
        api_error(genai_errors.ClientError, 408),
        ExceptionGroup("node failed", [TimeoutError(), api_error(genai_errors.ServerError, 503)]),
    ):
        assert is_transient_model_error(error), error


def test_bad_requests_bugs_and_cancellations_are_not_transient() -> None:
    for error in (
        api_error(genai_errors.ClientError, 400),
        api_error(genai_errors.ClientError, 403),
        ValueError("a bug"),
        asyncio.CancelledError(),
        ExceptionGroup("node failed", [TimeoutError(), ValueError("a bug")]),
        BaseExceptionGroup("cancelled", [asyncio.CancelledError()]),
    ):
        assert not is_transient_model_error(error), error


def test_error_names_are_short_and_carry_the_http_status() -> None:
    assert error_name(TimeoutError()) == "TimeoutError"
    assert error_name(api_error(genai_errors.ServerError, 503)) == "ServerError 503"
    group = ExceptionGroup("node failed", [TimeoutError(), api_error(genai_errors.ServerError, 503), TimeoutError()])
    assert error_name(group) == "ServerError 503, TimeoutError"


def test_the_executor_fallback_reports_what_the_robot_did_until_gemini_failed() -> None:
    report = executor_fallback_report(stretch(), TimeoutError())
    assert report.splitlines() == [
        "Gemini did not answer (TimeoutError); this stretch stopped early.",
        "Did: drove to Fridge (succeeded after 6.1 s); turned left 90 degrees.",
        "Success criteria not checked.",
    ]
    only_looked = executor_fallback_report(stretch()[:3], TimeoutError())
    assert "Did: looked around." in only_looked
    nothing = executor_fallback_report(stretch()[:1], api_error(genai_errors.ServerError, 503))
    assert nothing.startswith("Gemini did not answer (ServerError 503)") and "Did: nothing yet." in nothing


def test_the_goal_setter_fallback_only_ever_keeps_an_active_goal() -> None:
    decision = goal_setter_fallback_decision({STATE_CURRENT_GOAL: GOAL}, TimeoutError())
    assert decision is not None
    assert decision.previous_goal_status == "continue" and decision.goal is None
    assert GoalDecision.model_validate_json(decision.model_dump_json(exclude_none=True)) == decision
    assert goal_setter_fallback_decision({STATE_CURRENT_GOAL: None}, TimeoutError()) is None
    assert goal_setter_fallback_decision({}, TimeoutError()) is None
    resting = {**GOAL, "id": 0, "status": "limit"}
    assert goal_setter_fallback_decision({STATE_CURRENT_GOAL: resting}, TimeoutError()) is None


async def test_the_executor_callback_answers_transient_errors_only_and_tags_its_answer() -> None:
    request = LlmRequest(contents=stretch())
    response = await executor_on_model_error(None, request, TimeoutError())
    assert response is not None
    assert response.custom_metadata == {"fallback": {"agent": "executor", "error": "TimeoutError"}}
    assert response.content.role == "model"
    assert response.content.parts[0].text == executor_fallback_report(request.contents, TimeoutError())
    assert await executor_on_model_error(None, request, api_error(genai_errors.ClientError, 400)) is None


def test_fallback_turns_are_not_counted_as_gemini_calls() -> None:
    def model_turn(text: str, metadata: dict | None = None) -> Event:
        content = types.Content(role="model", parts=[types.Part(text=text)])
        return Event(author="executor", invocation_id="i1", content=content, custom_metadata=metadata)

    gemini = model_turn("I drove to the fridge.")
    fallback = model_turn("Gemini did not answer.", {"fallback": {"agent": "executor", "error": "TimeoutError"}})
    s1 = model_turn("Done.", {"s1": {"by": "rules"}})
    assert is_fallback_event(fallback) and not is_fallback_event(gemini) and not is_fallback_event(s1)
    assert model_call_count([gemini, fallback, s1]) == 1
    result = TickResult(number=4, seconds=3.0, events=[gemini, fallback], state={})
    assert result.fallbacks == ["executor"] and not result.failed
    assert result.summary_dict()["fallbacks"] == 1 and result.summary_dict()["error"] is None
