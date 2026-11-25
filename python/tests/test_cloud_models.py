"""Offline: other providers' cloud models in Gemini's place, on real genai and ADK objects: ADK's
request as OpenAI's Chat Completions and Anthropic's Messages bodies, their replies as ADK's
responses, which model each `[model]` name selects, and what a run's calls cost. The real APIs
are exercised live (tests/live/test_cloud_models_live.py)."""
from __future__ import annotations

import asyncio
import json
import time

from google.adk.models.llm_request import LlmRequest
from google.genai import types

from gpt_pet.agents.anthropic import RESPOND, AnthropicError, AnthropicLlm, anthropic_messages, messages_request, messages_response
from gpt_pet.agents.content import content_config
from gpt_pet.agents.fallbacks import error_name, is_transient_model_error
from gpt_pet.agents.model_api import post_json
from gpt_pet.agents.models import agent_model, is_gemini, missing_keys
from gpt_pet.agents.openai_chat import OpenAIChatLlm, OpenAIError, chat_messages, chat_request, chat_response, strict_schema
from gpt_pet.experiment import cost_row, price_of, run_cost
from gpt_pet.goals import TargetedGoalDecision
from gpt_pet.settings import ModelSettings
from gpt_pet.stats import RunStats

JPEG = b"\xff\xd8\xff\xe0 a frame"
DRIVE = types.FunctionDeclaration(
    name="set_nav_goal",
    description="Drive to a destination.",
    parameters_json_schema={"type": "object", "properties": {"destination_id": {"type": "string"}}, "required": ["destination_id"]},
)


def turns() -> list[types.Content]:
    """The goal, a look that returned a camera frame, then a drive."""
    view = types.FunctionResponse(
        id="adk-1",
        name="get_current_view",
        response={"result": {"destinations": [{"id": "obj:Sofa|1", "distance_m": 3.1}]}, "isError": False},
        parts=[types.FunctionResponsePart.from_bytes(data=JPEG, mime_type="image/jpeg")],
    )
    return [
        types.Content(role="user", parts=[types.Part(text='{"goal": "find the sofa"}')]),
        types.Content(role="model", parts=[types.Part(function_call=types.FunctionCall(id="adk-1", name="get_current_view", args={}))]),
        types.Content(role="user", parts=[types.Part(function_response=view)]),
        types.Content(
            role="model",
            parts=[
                types.Part(text="thinking it over", thought=True),
                types.Part(function_call=types.FunctionCall(id="adk-2", name="set_nav_goal", args={"destination_id": "obj:Sofa|1"})),
            ],
        ),
        types.Content(
            role="user",
            parts=[types.Part(function_response=types.FunctionResponse(id="adk-2", name="set_nav_goal", response={"error": "blocked"}))],
        ),
    ]


def goal_request() -> LlmRequest:
    config = types.GenerateContentConfig(system_instruction="Decide.", temperature=0.2, response_schema=TargetedGoalDecision)
    return LlmRequest(contents=[types.Content(role="user", parts=[types.Part(text="map"), types.Part.from_bytes(data=JPEG, mime_type="image/jpeg")])], config=config)


def executor_request() -> LlmRequest:
    config = types.GenerateContentConfig(system_instruction="Drive.", temperature=0.2, tools=[types.Tool(function_declarations=[DRIVE])])
    return LlmRequest(contents=turns(), config=config)


# --- OpenAI-compatible ---------------------------------------------------------------------


def test_chat_messages_pair_tool_calls_and_results_by_id_and_follow_results_with_their_images() -> None:
    messages = chat_messages(turns(), "Drive.")
    assert [m["role"] for m in messages] == ["system", "user", "assistant", "tool", "user", "assistant", "tool"]
    look, view, frame, drive, blocked = messages[2:]
    assert look["tool_calls"] == [{"id": "adk-1", "type": "function", "function": {"name": "get_current_view", "arguments": "{}"}}]
    assert look["content"] is None
    assert view == {"role": "tool", "tool_call_id": "adk-1", "content": view["content"]} and "obj:Sofa|1" in view["content"]
    note, image = frame["content"]
    assert note == {"type": "text", "text": "Images returned by get_current_view."}
    assert image["image_url"]["url"].startswith("data:image/jpeg;base64,")
    assert json.loads(drive["tool_calls"][0]["function"]["arguments"]) == {"destination_id": "obj:Sofa|1"}  # thought dropped
    assert blocked["tool_call_id"] == "adk-2"


def test_strict_schema_requires_every_field_and_keeps_its_type() -> None:
    schema = strict_schema(TargetedGoalDecision.model_json_schema())
    assert schema["additionalProperties"] is False
    assert set(schema["required"]) == set(TargetedGoalDecision.model_fields)
    assert schema["properties"]["previous_goal_status"]["enum"] == ["none", "continue", "done", "abandoned"]
    assert schema["properties"]["sub_goals"]["type"] == "array"  # not nullable: the decision's list refuses null
    assert {"type": "null"} in schema["properties"]["target_id"]["anyOf"]  # nullable already
    text = json.dumps(schema)
    assert '"default"' not in text and '"title"' not in text
    # a property literally named "title" survives
    assert strict_schema({"type": "object", "properties": {"title": {"type": "string"}}})["required"] == ["title"]


def test_a_reasoning_model_gets_its_effort_and_no_temperature() -> None:
    body = chat_request("gpt-5-mini", goal_request(), max_tokens=2048, reasoning_effort="low")
    assert body["reasoning_effort"] == "low" and "temperature" not in body and body["max_completion_tokens"] == 2048
    assert body["response_format"]["type"] == "json_schema" and body["response_format"]["json_schema"]["strict"] is True
    assert body["response_format"]["json_schema"]["name"] == "TargetedGoalDecision"
    assert "temperature" not in chat_request("gpt-5-nano", goal_request(), max_tokens=2048)  # reasoning by name
    luna = chat_request("gpt-6-luna", executor_request(), max_tokens=2048, reasoning_effort="none")
    assert luna["reasoning_effort"] == "none" and "temperature" not in luna  # tools on Chat Completions need "none"
    plain = chat_request("gpt-4.1-mini", executor_request(), max_tokens=512)
    assert plain["temperature"] == 0.2 and "reasoning_effort" not in plain and "response_format" not in plain
    assert plain["tools"][0]["function"]["name"] == "set_nav_goal"


def test_openrouter_takes_its_own_reasoning_and_token_fields_and_reports_cost() -> None:
    body = chat_request("mistralai/mistral-small", executor_request(), max_tokens=512, reasoning_effort="low", openrouter=True)
    assert body["reasoning"] == {"effort": "low"} and body["max_tokens"] == 512 and body["usage"] == {"include": True}
    assert "max_completion_tokens" not in body and "reasoning_effort" not in body and "provider" not in body
    off = chat_request("deepseek/deepseek-v4.1-flash", goal_request(), max_tokens=512, reasoning_effort="none", openrouter=True)
    assert off["reasoning"] == {"enabled": False}
    assert off["provider"] == {"require_parameters": True}  # only providers that keep the strict schema


def test_a_chat_completion_becomes_text_tool_calls_and_split_tokens() -> None:
    reply = {
        "model": "gpt-5-mini-2025-08-07",
        "choices": [
            {
                "finish_reason": "tool_calls",
                "message": {
                    "role": "assistant",
                    "content": "Driving.",
                    "tool_calls": [{"id": "call_9", "type": "function", "function": {"name": "set_nav_goal", "arguments": '{"destination_id": "psg:1"}'}}],
                },
            }
        ],
        "usage": {
            "prompt_tokens": 5000,
            "completion_tokens": 300,
            "prompt_tokens_details": {"cached_tokens": 4096, "cache_write_tokens": 600},
            "completion_tokens_details": {"reasoning_tokens": 256},
        },
    }
    response = chat_response(reply)
    text, call = response.content.parts
    assert text.text == "Driving." and (call.function_call.id, call.function_call.args) == ("call_9", {"destination_id": "psg:1"})
    usage = response.usage_metadata
    assert (usage.prompt_token_count, usage.cached_content_token_count, usage.candidates_token_count, usage.thoughts_token_count) == (5000, 4096, 44, 256)
    assert usage.total_token_count == 5300 and response.model_version == "gpt-5-mini-2025-08-07"
    assert response.custom_metadata == {"billing": {"cache_write_tokens": 600}}  # priced above plain input
    billed = chat_response({"choices": [{"message": {"content": "{}"}, "finish_reason": "length"}], "usage": {"prompt_tokens": 10, "completion_tokens": 2, "cost": 0.00012}})
    assert billed.custom_metadata == {"billing": {"cost_usd": 0.00012}} and billed.finish_reason == types.FinishReason.MAX_TOKENS


# --- Anthropic -----------------------------------------------------------------------------


def test_claude_reads_tool_images_inside_the_tool_result() -> None:
    messages = anthropic_messages(turns())
    assert [m["role"] for m in messages] == ["user", "assistant", "user", "assistant", "user"]
    [result] = messages[2]["content"]
    assert result["type"] == "tool_result" and result["tool_use_id"] == "adk-1" and "is_error" not in result
    text, image = result["content"]
    assert "obj:Sofa|1" in text["text"] and image["source"]["media_type"] == "image/jpeg"
    assert messages[3]["content"] == [{"type": "tool_use", "id": "adk-2", "name": "set_nav_goal", "input": {"destination_id": "obj:Sofa|1"}}]
    assert messages[4]["content"][0]["is_error"] is True


def test_the_executor_caches_the_system_prompt_and_the_tick_so_far() -> None:
    body = messages_request("claude-haiku-4-5", executor_request(), max_tokens=1024)
    assert body["system"] == [{"type": "text", "text": "Drive.", "cache_control": {"type": "ephemeral"}}]
    assert body["messages"][-1]["content"][-1]["cache_control"] == {"type": "ephemeral"}
    assert [tool["name"] for tool in body["tools"]] == ["set_nav_goal"] and "tool_choice" not in body
    assert body["max_tokens"] == 1024 and body["temperature"] == 0.2
    plain = messages_request("claude-haiku-4-5", executor_request(), max_tokens=1024, cache=False)
    assert plain["system"] == "Drive." and "cache_control" not in json.dumps(plain)


def test_the_goal_setter_answers_through_a_tool_it_must_call() -> None:
    body = messages_request("claude-haiku-4-5", goal_request(), max_tokens=1024)
    assert body["system"] == "Decide." and "cache_control" not in json.dumps(body)  # a new prompt every tick
    [respond] = body["tools"]
    assert respond["name"] == RESPOND and respond["input_schema"] == TargetedGoalDecision.model_json_schema()
    assert body["tool_choice"] == {"type": "tool", "name": RESPOND}
    decision = {"previous_goal_status": "none", "goal": "find the sofa", "reasoning": "nothing yet"}
    reply = {
        "model": "claude-haiku-4-5-20251001",
        "content": [{"type": "tool_use", "id": "toolu_1", "name": RESPOND, "input": decision}],
        "stop_reason": "tool_use",
        "usage": {"input_tokens": 900, "cache_creation_input_tokens": 0, "cache_read_input_tokens": 0, "output_tokens": 60},
    }
    response = messages_response(reply, answer_tool=True)
    [part] = response.content.parts
    assert TargetedGoalDecision.model_validate_json(part.text).goal == "find the sofa"
    assert response.custom_metadata is None


def test_a_claude_reply_counts_cache_reads_as_cached_and_cache_writes_apart() -> None:
    reply = {
        "content": [{"type": "text", "text": "Looking."}, {"type": "tool_use", "id": "toolu_2", "name": "get_current_view", "input": {}}],
        "stop_reason": "max_tokens",
        "usage": {"input_tokens": 50, "cache_creation_input_tokens": 1200, "cache_read_input_tokens": 3000, "output_tokens": 40},
    }
    response = messages_response(reply)
    text, call = response.content.parts
    assert text.text == "Looking." and call.function_call.id == "toolu_2"
    usage = response.usage_metadata
    assert (usage.prompt_token_count, usage.cached_content_token_count, usage.candidates_token_count) == (4250, 3000, 40)
    assert response.custom_metadata == {"billing": {"cache_write_tokens": 1200}} and response.finish_reason == types.FinishReason.MAX_TOKENS


# --- routing, settings, errors and cost ----------------------------------------------------


def test_model_names_pick_the_provider() -> None:
    model = ModelSettings(reasoning_effort="low", max_output_tokens=1024, request_timeout_s=30)
    assert agent_model("gemini-3.5-flash", model) == "gemini-3.5-flash" and is_gemini("gemini-3.5-flash-lite")
    openai = agent_model("openai/gpt-5-mini", model)
    assert isinstance(openai, OpenAIChatLlm) and (openai.model, openai.api_key_env, openai.reasoning_effort) == ("gpt-5-mini", "OPENAI_API_KEY", "low")
    router = agent_model("openrouter/mistralai/mistral-small-3.2", model)
    assert isinstance(router, OpenAIChatLlm) and router.openrouter and router.model == "mistralai/mistral-small-3.2"
    claude = agent_model("anthropic/claude-haiku-4-5", model)
    assert isinstance(claude, AnthropicLlm) and (claude.model, claude.max_output_tokens, claude.timeout_s) == ("claude-haiku-4-5", 1024, 30)
    assert not is_gemini("anthropic/claude-haiku-4-5")


def test_gemini_takes_the_reasoning_effort_as_its_thinking_level() -> None:
    assert content_config(ModelSettings(), "gemini-3.5-flash").thinking_config is None  # the API's default
    config = content_config(ModelSettings(reasoning_effort="minimal"), "gemini-3.5-flash")
    assert config.thinking_config.thinking_level == types.ThinkingLevel.MINIMAL
    assert content_config(ModelSettings(reasoning_effort="minimal"), "openai/gpt-5-mini").thinking_config is None


def test_rate_limits_and_overload_are_transient_and_bad_requests_are_not() -> None:
    for status in (429, 500, 503, 529):
        assert is_transient_model_error(AnthropicError(status, "x")) and is_transient_model_error(OpenAIError(status, "x"))
    for status in (400, 401, 404):
        assert not is_transient_model_error(OpenAIError(status, "x"))
    assert error_name(AnthropicError(529, "overloaded")) == "AnthropicError 529"


def test_a_run_costs_its_tokens_at_their_prices() -> None:
    prices = {
        "gpt-5-mini": {"input": 0.25, "cached_input": 0.025, "output": 2.0},
        "claude-haiku-4-5": {"input": 1.0, "cached_input": 0.1, "cache_write": 1.25, "output": 5.0},
    }
    assert price_of("gpt-5-mini-2025-08-07", prices) is prices["gpt-5-mini"] and price_of("gpt-5", prices) is None
    by_model = {
        "gpt-5-mini": {"input": 1_000_000, "cached": 400_000, "cache_write": 0, "output": 10_000, "thinking": 20_000, "billed_usd": 0},
        "claude-haiku-4-5": {"input": 100_000, "cached": 50_000, "cache_write": 20_000, "output": 1_000, "thinking": 0, "billed_usd": 0},
        "router-model": {"input": 5, "cached": 0, "cache_write": 0, "output": 5, "thinking": 0, "billed_usd": 0.5},
    }
    # gpt-5-mini: 0.6M x 0.25 + 0.4M x 0.025 + 30k x 2.0 = 0.15 + 0.01 + 0.06
    # haiku: 30k x 1.0 + 50k x 0.1 + 20k x 1.25 + 1k x 5.0 = 0.03 + 0.005 + 0.025 + 0.005
    cost, unpriced = run_cost(by_model, prices)
    assert cost == round(0.22 + 0.065 + 0.5, 5) and unpriced == []
    assert run_cost({"mystery": {"input": 10, "output": 1}}, prices) == (None, ["mystery"])
    row = cost_row({"llm": {"by_model": {"gpt-5-mini": by_model["gpt-5-mini"]}}}, 300.0, prices)
    assert row == {"cost_usd": 0.22, "cost_per_hour_usd": 2.64, "cost_unpriced": ""}


def test_stats_count_tokens_per_model_with_cache_writes_and_billed_cost() -> None:
    stats = RunStats(started=0.0, wall_started=0.0)
    usage = types.GenerateContentResponseUsageMetadata(prompt_token_count=4000, cached_content_token_count=3000, candidates_token_count=50, total_token_count=4050)
    stats.record_llm_call("executor", 1.0, usage, model="claude-haiku-4-5", billing={"cache_write_tokens": 800})
    stats.record_llm_call("executor", 1.0, usage, model="claude-haiku-4-5")
    stats.record_llm_call("goal_setter", 1.0, usage, model="openrouter-model", billing={"cost_usd": 0.001})
    by_model = stats.snapshot(now=60.0)["llm"]["by_model"]
    assert by_model["claude-haiku-4-5"] == {"calls": 2, "input": 8000, "cached": 6000, "cache_write": 800, "output": 100, "thinking": 0, "billed_usd": 0}
    assert by_model["openrouter-model"]["billed_usd"] == 0.001


def test_a_suite_knows_which_api_keys_its_models_need(monkeypatch) -> None:
    for env in ("OPENAI_API_KEY", "ANTHROPIC_API_KEY", "OPENROUTER_API_KEY", "GEMINI_API_KEY"):
        monkeypatch.delenv(env, raising=False)
    monkeypatch.setenv("GOOGLE_API_KEY", "set")
    assert missing_keys(ModelSettings()) == []
    both = ModelSettings(goal_setter="openai/gpt-5-mini", executor="anthropic/claude-haiku-4-5")
    assert missing_keys(both) == ["OPENAI_API_KEY", "ANTHROPIC_API_KEY"]
    monkeypatch.setenv("OPENAI_API_KEY", "set")
    assert missing_keys(ModelSettings(goal_setter="openai/gpt-5-mini", executor="openai/gpt-5-mini")) == []
    assert missing_keys(ModelSettings(goal_setter="ollama/qwen3-vl:8b", executor="openrouter/x/y")) == ["OPENROUTER_API_KEY"]
    monkeypatch.delenv("GOOGLE_API_KEY")
    assert missing_keys(ModelSettings()) == ["GOOGLE_API_KEY"]


def test_a_key_without_a_workspace_names_its_workspace(monkeypatch) -> None:
    monkeypatch.setenv("ANTHROPIC_API_KEY", "set")
    monkeypatch.delenv("ANTHROPIC_WORKSPACE_ID", raising=False)
    assert "anthropic-workspace-id" not in AnthropicLlm(model="claude-haiku-4-5")._headers()
    monkeypatch.setenv("ANTHROPIC_WORKSPACE_ID", "wrkspc_01")
    assert AnthropicLlm(model="claude-haiku-4-5")._headers()["anthropic-workspace-id"] == "wrkspc_01"


def test_image_detail_reaches_gemini_and_openai_images() -> None:
    config = content_config(ModelSettings(image_detail="low"), "gemini-3.5-flash-lite")
    assert config.media_resolution == types.MediaResolution.MEDIA_RESOLUTION_LOW
    assert content_config(ModelSettings(), "gemini-3.5-flash-lite").media_resolution is None
    assert content_config(ModelSettings(image_detail="low"), "openai/gpt-6-luna").media_resolution is None
    body = chat_request("gpt-6-luna", executor_request(), max_tokens=512, reasoning_effort="none", image_detail="low")
    [frame] = [m for m in body["messages"] if m["role"] == "user" and isinstance(m["content"], list)]
    assert frame["content"][1]["image_url"]["detail"] == "low"
    plain = chat_request("gpt-6-luna", executor_request(), max_tokens=512, reasoning_effort="none")
    [frame] = [m for m in plain["messages"] if m["role"] == "user" and isinstance(m["content"], list)]
    assert "detail" not in frame["content"][1]["image_url"]
    assert agent_model("openai/gpt-6-luna", ModelSettings(image_detail="medium")).image_detail == "medium"


def handed_over_turns() -> list[types.Content]:
    """A tick System 1 began and Claude took over, as ADK hands it to a model: the ids ADK made up
    for S1's calls (`adk-...`) stripped, Claude's own kept."""
    def call(name: str, call_id: str | None = None, **args) -> types.Content:
        return types.Content(role="model", parts=[types.Part(function_call=types.FunctionCall(id=call_id, name=name, args=args))])

    def result(name: str, call_id: str | None = None) -> types.Content:
        return types.Content(role="user", parts=[types.Part(function_response=types.FunctionResponse(id=call_id, name=name, response={"result": "ok"}))])

    return [
        types.Content(role="user", parts=[types.Part(text='{"goal": "explore"}')]),
        call("get_current_view"), result("get_current_view"),
        call("do_rotate", degrees=90), result("do_rotate"),
        call("get_current_view"), result("get_current_view"),
        call("set_nav_goal", "toolu_01abc", destination_id="psg:6"), result("set_nav_goal", "toolu_01abc"),
    ]


def test_calls_whose_ids_adk_stripped_get_ids_their_results_share() -> None:
    import re

    from gpt_pet.agents.model_api import with_call_ids

    contents = handed_over_turns()
    filled = with_call_ids(contents)
    calls = [p.function_call for c in filled for p in c.parts if p.function_call]
    results = [p.function_response for c in filled for p in c.parts if p.function_response]
    assert [c.id for c in calls] == [r.id for r in results] == ["call_1", "call_2", "call_3", "toolu_01abc"]
    assert contents[1].parts[0].function_call.id is None  # the session's own content is untouched

    config = types.GenerateContentConfig(tools=[types.Tool(function_declarations=[DRIVE])])
    body = messages_request("claude-haiku-4-5", LlmRequest(contents=contents, config=config), max_tokens=512)
    uses = [b for m in body["messages"] for b in m["content"] if b["type"] == "tool_use"]
    answers = [b for m in body["messages"] for b in m["content"] if b["type"] == "tool_result"]
    assert all(re.fullmatch(r"[a-zA-Z0-9_-]+", u["id"]) for u in uses)  # Anthropic's rule
    assert [u["id"] for u in uses] == [a["tool_use_id"] for a in answers]
    chat = chat_request("gpt-6-luna", LlmRequest(contents=contents, config=config), max_tokens=512, reasoning_effort="none")
    ids = [c["id"] for m in chat["messages"] if m.get("tool_calls") for c in m["tool_calls"]]
    assert ids == [m["tool_call_id"] for m in chat["messages"] if m["role"] == "tool"] and all(ids)


async def test_a_call_kept_open_with_whitespace_times_out_as_a_whole() -> None:
    """OpenRouter keeps a slow answer's connection open with whitespace, so a per-read timeout
    never fires; `post_json` bounds the whole call (one hung 100 s under a 20 s timeout,
    2026-10-03). Against a real local server that trickles a space every 50 ms."""

    async def trickle(reader: asyncio.StreamReader, writer: asyncio.StreamWriter) -> None:
        await reader.readuntil(b"\r\n\r\n")
        writer.write(b"HTTP/1.1 200 OK\r\nContent-Type: application/json\r\nContent-Length: 100000\r\n\r\n")
        try:
            for _ in range(200):
                writer.write(b" ")
                await writer.drain()
                await asyncio.sleep(0.05)
        except ConnectionError:
            pass
        writer.close()

    server = await asyncio.start_server(trickle, "127.0.0.1", 0)
    port = server.sockets[0].getsockname()[1]
    async with server:
        started = time.monotonic()
        try:
            await post_json(OpenAIError, f"http://127.0.0.1:{port}", "/chat/completions", {"model": "x"}, timeout_s=0.5)
        except TimeoutError as raised:
            error: BaseException = raised
        else:
            raise AssertionError("the call should have timed out")
        assert time.monotonic() - started < 2.0
    assert is_transient_model_error(error) and "Timeout" in error_name(error)  # the fallback answers for it
