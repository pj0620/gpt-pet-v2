"""Offline: a local model in Gemini's place (`gpt_pet.agents.ollama`), on real genai and ADK
objects: ADK's request as Ollama's `/api/chat` body, and Ollama's reply as ADK's response. The
real Ollama is exercised live (tests/live/test_ollama_live.py)."""
from __future__ import annotations

import json

from google.adk.models.llm_request import LlmRequest
from google.genai import types

from gpt_pet.agents.fallbacks import error_name, executor_fallback_report, is_transient_model_error, model_name
from gpt_pet.agents.model_api import json_schema
from gpt_pet.agents.models import agent_model
from gpt_pet.agents.ollama import (
    OllamaError,
    OllamaLlm,
    chat_messages,
    chat_request,
    chat_response,
    local_models,
)
from gpt_pet.goals import TargetedGoalDecision
from gpt_pet.settings import ModelSettings, Settings, load_settings
from gpt_pet.stats import RunStats

JPEG = b"\xff\xd8\xff\xe0 a frame"
MAP = b"\x89PNG a map"
OPTIONS = {"num_ctx": 16384, "num_predict": 1024, "keep_alive": "30m"}


def local(settings: Settings, name: str = "ollama/qwen3-vl:8b-instruct") -> Settings:
    model = settings.model.model_copy(update={"goal_setter": name, "executor": name, "request_timeout_s": 90})
    return settings.model_copy(update={"model": model})


def executor_turns() -> list[types.Content]:
    """The goal, a look that returned a camera frame, then a drive."""
    view = types.FunctionResponse(
        id="c1",
        name="get_current_view",
        response={"result": {"destinations": [{"id": "obj:Sofa|1", "distance_m": 3.1}]}, "isError": False},
        parts=[types.FunctionResponsePart.from_bytes(data=JPEG, mime_type="image/jpeg")],
    )
    return [
        types.Content(role="user", parts=[types.Part(text='{"goal": "find the sofa"}')]),
        types.Content(role="model", parts=[types.Part(function_call=types.FunctionCall(id="c1", name="get_current_view", args={}))]),
        types.Content(role="user", parts=[types.Part(function_response=view)]),
        types.Content(
            role="model",
            parts=[
                types.Part(text="planning", thought=True),
                types.Part(function_call=types.FunctionCall(id="c2", name="set_nav_goal", args={"destination_id": "obj:Sofa|1"})),
            ],
        ),
    ]


def test_turns_become_chat_messages_with_tool_images_after_their_results() -> None:
    messages = chat_messages(executor_turns(), "You drive a robot.")
    assert [m["role"] for m in messages] == ["system", "user", "assistant", "tool", "user", "assistant"]
    assert messages[0]["content"] == "You drive a robot."
    assert messages[2] == {"role": "assistant", "content": "", "tool_calls": [{"function": {"name": "get_current_view", "arguments": {}}}]}
    tool, frame = messages[3], messages[4]
    assert tool["tool_name"] == "get_current_view" and json.loads(tool["content"])["result"]["destinations"][0]["id"] == "obj:Sofa|1"
    assert frame["content"] == "Images returned by get_current_view." and len(frame["images"]) == 1
    assert "images" not in tool
    # thoughts are not replayed; the drive's arguments are
    assert messages[5]["content"] == ""
    assert messages[5]["tool_calls"][0]["function"] == {"name": "set_nav_goal", "arguments": {"destination_id": "obj:Sofa|1"}}


def test_a_user_turn_keeps_its_own_images() -> None:
    turn = types.Content(role="user", parts=[types.Part(text="Where next?"), types.Part.from_bytes(data=MAP, mime_type="image/png")])
    [message] = chat_messages([turn])
    assert message["content"] == "Where next?" and len(message["images"]) == 1


def test_the_goal_setter_request_carries_its_schema_as_format_and_no_tools() -> None:
    config = types.GenerateContentConfig(system_instruction="Decide.", temperature=0.2, response_schema=TargetedGoalDecision)
    body = chat_request("qwen3-vl:8b-instruct", LlmRequest(contents=[types.Content(role="user", parts=[types.Part(text="go")])], config=config), **OPTIONS)
    assert body["format"] == TargetedGoalDecision.model_json_schema()
    assert "tools" not in body and "think" not in body and body["stream"] is False
    assert body["options"] == {"num_ctx": 16384, "num_predict": 1024, "temperature": 0.2}
    assert body["keep_alive"] == "30m" and body["messages"][0] == {"role": "system", "content": "Decide."}


def test_the_executor_request_declares_its_tools_and_says_whether_to_think() -> None:
    declared = types.FunctionDeclaration(
        name="set_nav_goal",
        description="Drive to a destination.",
        parameters_json_schema={"type": "object", "properties": {"destination_id": {"type": "string"}}, "required": ["destination_id"]},
    )
    legacy = types.FunctionDeclaration(
        name="do_rotate",
        parameters=types.Schema(type=types.Type.OBJECT, properties={"degrees": types.Schema(type=types.Type.INTEGER)}),
    )
    config = types.GenerateContentConfig(tools=[types.Tool(function_declarations=[declared, legacy])], max_output_tokens=256)
    body = chat_request("m", LlmRequest(contents=executor_turns(), config=config), **OPTIONS, think=False)
    assert body["think"] is False and body["options"]["num_predict"] == 256
    drive, turn = (tool["function"] for tool in body["tools"])
    assert drive["name"] == "set_nav_goal" and drive["parameters"]["required"] == ["destination_id"]
    assert turn["parameters"] == {"type": "object", "properties": {"degrees": {"type": "integer"}}}


def test_json_schema_reads_dicts_models_and_genai_schemas() -> None:
    assert json_schema(None) is None
    assert json_schema({"type": "object"}) == {"type": "object"}
    assert json_schema(TargetedGoalDecision)["required"] == ["previous_goal_status", "reasoning"]
    nested = types.Schema(type=types.Type.ARRAY, items=types.Schema(type=types.Type.STRING), max_items=3)
    assert json_schema(nested) == {"type": "array", "items": {"type": "string"}, "maxItems": 3}


def test_a_reply_becomes_thinking_text_tool_calls_tokens_and_timings() -> None:
    reply = {
        "message": {
            "role": "assistant",
            "thinking": "the sofa is ahead",
            "content": "Driving.",
            "tool_calls": [
                {"function": {"name": "set_nav_goal", "arguments": {"destination_id": "obj:Sofa|1"}}},
                {"function": {"name": "do_rotate", "arguments": '{"degrees": 90}'}},
            ],
        },
        "done_reason": "stop",
        "load_duration": 2_500_000_000,
        "prompt_eval_count": 1200,
        "prompt_eval_duration": 1_500_000_000,
        "eval_count": 40,
        "eval_duration": 400_000_000,
        "total_duration": 4_500_000_000,
    }
    response = chat_response(reply)
    thought, text, drive, turn = response.content.parts
    assert thought.thought and thought.text == "the sofa is ahead" and text.text == "Driving."
    assert drive.function_call.args == {"destination_id": "obj:Sofa|1"} and turn.function_call.args == {"degrees": 90}
    assert response.usage_metadata.prompt_token_count == 1200 and response.usage_metadata.total_token_count == 1240
    assert response.finish_reason == types.FinishReason.STOP
    assert response.custom_metadata["ollama"] == {
        "load_s": 2.5, "prompt_tokens": 1200, "prompt_s": 1.5, "output_tokens": 40, "output_s": 0.4, "total_s": 4.5
    }


def test_an_empty_or_cut_off_reply_is_still_a_response() -> None:
    response = chat_response({"message": {"content": ""}, "done_reason": "length"})
    assert [part.text for part in response.content.parts] == [""]
    assert response.finish_reason == types.FinishReason.MAX_TOKENS


def test_profiles_name_local_models_with_an_ollama_prefix() -> None:
    settings = local(load_settings("sim"))
    assert agent_model("gemini-3.5-flash", settings.model) == "gemini-3.5-flash"
    model = agent_model(settings.model.executor, settings.model)
    assert isinstance(model, OllamaLlm) and model.model == "qwen3-vl:8b-instruct"
    assert (model.base_url, model.num_ctx, model.timeout_s, model.think) == ("http://127.0.0.1:11434", 16384, 90, False)
    [only] = local_models(settings.model)  # one model for both agents
    assert only.model == "qwen3-vl:8b-instruct"
    assert local_models(ModelSettings()) == []
    mixed = ModelSettings(goal_setter="gemini-3.5-flash", executor="ollama/gemma4:e4b", ollama_num_ctx=8192, ollama_think=True)
    [executor] = local_models(mixed)
    assert (executor.model, executor.num_ctx, executor.think) == ("gemma4:e4b", 8192, True)


def test_a_failed_runner_is_transient_and_a_bad_request_is_not() -> None:
    assert is_transient_model_error(OllamaError(500, "llama runner process has terminated"))
    assert is_transient_model_error(OllamaError(503, "server busy"))
    assert not is_transient_model_error(OllamaError(400, "qwen3-vl does not support tools"))
    assert not is_transient_model_error(OllamaError(404, "model not found"))
    assert error_name(OllamaError(500, "x")) == "OllamaError 500"


def test_a_fallback_names_the_model_that_failed() -> None:
    assert model_name(LlmRequest(model="gemini-3.5-flash")) == "Gemini"
    assert model_name(LlmRequest()) == "Gemini"
    assert model_name(LlmRequest(model="qwen3-vl:8b-instruct")) == "qwen3-vl:8b-instruct"
    report = executor_fallback_report([], TimeoutError(), "qwen3-vl:8b-instruct")
    assert report.startswith("qwen3-vl:8b-instruct did not answer (TimeoutError)")


def test_stats_split_a_local_models_time() -> None:
    stats = RunStats(started=0.0, wall_started=0.0)
    assert stats.snapshot(now=60.0)["llm"]["local"] == {"calls": 0}
    usage = types.GenerateContentResponseUsageMetadata(prompt_token_count=1000, candidates_token_count=50, total_token_count=1050)
    stats.record_llm_call("executor", 3.0, usage, {"load_s": 0.01, "prompt_tokens": 1000, "prompt_s": 2.0, "output_tokens": 50, "output_s": 0.5})
    stats.record_llm_call("goal_setter", 9.0, usage, {"load_s": 6.0, "prompt_tokens": 3000, "prompt_s": 2.0, "output_tokens": 30, "output_s": 0.5})
    stats.record_llm_call("goal_setter", 1.0, usage)  # Gemini: no timings of its own
    local_times = stats.snapshot(now=60.0)["llm"]["local"]
    assert local_times == {
        "calls": 2,
        "reloads": 1,
        "load_s": 6.0,
        "prompt_s": 4.0,
        "output_s": 1.0,
        "prompt_tok_s": 1000.0,
        "output_tok_s": 80.0,
        "max_prompt_tokens": 3000,
    }
