"""Claude in Gemini's place: Anthropic's Messages API as an ADK model (`anthropic/<model>`, e.g.
`anthropic/claude-haiku-4-5`).

One ADK request becomes one `POST /v1/messages`: the system instruction, the turns as content
blocks with base64 images (a tool's images go inside its `tool_result`, where Claude reads them),
the tool declarations, and the goal setter's output schema as a tool Claude must call (`respond`),
whose input is the JSON answer. Requests with tools (the executor) mark two cache breakpoints, the
end of the system prompt and the latest turn, so a tick's later calls re-read what came before
from the cache: a tenth of the input price, and not counted against the input-token rate limit.
Extended thinking stays off.

The API key comes from the environment (`ANTHROPIC_API_KEY`), like Gemini's.
"""
from __future__ import annotations

import json
import os
from collections.abc import AsyncGenerator, Iterable, Mapping
from typing import Any

from google.adk.models.base_llm import BaseLlm
from google.adk.models.llm_request import LlmRequest
from google.adk.models.llm_response import LlmResponse
from google.genai import types

from gpt_pet.agents.model_api import (
    ModelApiError,
    as_text,
    function_declarations,
    image_blob,
    output_schema,
    post_json,
    system_text,
    tool_result_images,
    usage_metadata,
    with_call_ids,
)
from gpt_pet.settings import ModelSettings

ANTHROPIC_PREFIX = "anthropic/"
API_URL = "https://api.anthropic.com"
API_VERSION = "2023-06-01"
KEY_ENV = "ANTHROPIC_API_KEY"
WORKSPACE_ENV = "ANTHROPIC_WORKSPACE_ID"
"""Only for a key that is not scoped to a workspace (a user key, `sk-ant-usr...`)."""
RESPOND = "respond"
"""The tool Claude must call when an answer has a schema: its input is the answer."""
CACHE = {"type": "ephemeral"}
FINISH_REASONS = {"max_tokens": types.FinishReason.MAX_TOKENS, "refusal": types.FinishReason.SAFETY}


def is_anthropic(model: str) -> bool:
    return model.startswith(ANTHROPIC_PREFIX)


class AnthropicError(ModelApiError):
    provider = "Anthropic"


# --- request -------------------------------------------------------------------------------


def _image(mime_type: str, data: str) -> dict[str, Any]:
    return {"type": "image", "source": {"type": "base64", "media_type": mime_type, "data": data}}


def _is_error(response: Any) -> bool:
    return isinstance(response, Mapping) and bool(response.get("isError") or response.get("error"))


def anthropic_messages(contents: Iterable[types.Content]) -> list[dict[str, Any]]:
    """ADK's turns as alternating user / assistant messages of content blocks. Thoughts are not
    replayed; a user turn's tool results come first, as the API requires."""
    messages: list[dict[str, Any]] = []
    for content in contents:
        role = "assistant" if content.role == "model" else "user"
        results: list[dict[str, Any]] = []
        blocks: list[dict[str, Any]] = []
        for part in content.parts or []:
            if part.thought:
                continue
            if part.function_call:
                call = part.function_call
                blocks.append({"type": "tool_use", "id": call.id or "", "name": call.name or "", "input": dict(call.args or {})})
            elif part.function_response:
                response = part.function_response
                inner = [{"type": "text", "text": as_text(response.response)}, *(_image(*image) for image in tool_result_images(response))]
                result: dict[str, Any] = {"type": "tool_result", "tool_use_id": response.id or "", "content": inner}
                if _is_error(response.response):
                    result["is_error"] = True
                results.append(result)
            elif part.text and part.text.strip():
                blocks.append({"type": "text", "text": part.text})
            elif image := image_blob(part.inline_data):
                blocks.append(_image(*image))
        blocks = results + blocks
        if not blocks:
            continue
        if messages and messages[-1]["role"] == role:
            merged = messages[-1]["content"] + blocks
            messages[-1]["content"] = sorted(merged, key=lambda block: block["type"] != "tool_result") if role == "user" else merged
        else:
            messages.append({"role": role, "content": blocks})
    if messages and messages[0]["role"] != "user":
        messages.insert(0, {"role": "user", "content": [{"type": "text", "text": "Begin."}]})
    return messages


def messages_request(model: str, request: LlmRequest, *, max_tokens: int, cache: bool = True) -> dict[str, Any]:
    """The `/v1/messages` body for one ADK request."""
    config = request.config
    body: dict[str, Any] = {
        "model": model,
        "max_tokens": (config.max_output_tokens if config else None) or max_tokens,
        "messages": anthropic_messages(with_call_ids(request.contents or [])),
    }
    system = system_text(config)
    tools = [
        {"name": name, "description": description, "input_schema": parameters}
        for name, description, parameters in function_declarations(config)
    ]
    if cache and tools:  # the executor's tool loop: re-read the tick so far from the cache
        if system:
            body["system"] = [{"type": "text", "text": system, "cache_control": CACHE}]
        last = body["messages"][-1]["content"][-1] if body["messages"] else None
        if last is not None:
            last["cache_control"] = CACHE
    elif system:
        body["system"] = system
    schema = output_schema(config)
    if schema:
        tools.append({"name": RESPOND, "description": "Give your answer.", "input_schema": schema})
        body["tool_choice"] = {"type": "tool", "name": RESPOND}
    if tools:
        body["tools"] = tools
    if config is not None and config.temperature is not None:
        body["temperature"] = config.temperature
    return body


# --- response ------------------------------------------------------------------------------


def messages_response(data: Mapping[str, Any], *, answer_tool: bool = False) -> LlmResponse:
    """One `/v1/messages` reply as ADK's response. With `answer_tool`, the `respond` call's input is
    the answer's JSON text. Cache writes are reported apart (`custom_metadata["billing"]`): they
    cost more than plain input."""
    parts: list[types.Part] = []
    for block in data.get("content") or []:
        kind = block.get("type")
        if kind == "thinking" and block.get("thinking"):
            parts.append(types.Part(text=block["thinking"], thought=True))
        elif kind == "text" and block.get("text"):
            parts.append(types.Part(text=block["text"]))
        elif kind == "tool_use" and answer_tool and block.get("name") == RESPOND:
            parts.append(types.Part(text=json.dumps(block.get("input") or {}, ensure_ascii=False)))
        elif kind == "tool_use":
            call = types.FunctionCall(id=block.get("id"), name=block.get("name") or "", args=dict(block.get("input") or {}))
            parts.append(types.Part(function_call=call))
    if not parts:
        parts.append(types.Part(text=""))
    usage = data.get("usage") or {}
    written = int(usage.get("cache_creation_input_tokens") or 0)
    read = int(usage.get("cache_read_input_tokens") or 0)
    return LlmResponse(
        content=types.Content(role="model", parts=parts),
        usage_metadata=usage_metadata(prompt=int(usage.get("input_tokens") or 0) + written + read, output=int(usage.get("output_tokens") or 0), cached=read),
        finish_reason=FINISH_REASONS.get(str(data.get("stop_reason")), types.FinishReason.STOP),
        model_version=data.get("model"),
        custom_metadata={"billing": {"cache_write_tokens": written}} if written else None,
    )


# --- the model -----------------------------------------------------------------------------


class AnthropicLlm(BaseLlm):
    """Claude standing in for Gemini (see the module docstring). `model` is the API's model id,
    e.g. `claude-haiku-4-5`."""

    base_url: str = API_URL
    max_output_tokens: int = 2048
    timeout_s: float = 0
    """Per call; 0 waits forever."""
    cache: bool = True

    @classmethod
    def from_settings(cls, name: str, model: ModelSettings) -> "AnthropicLlm":
        return cls(model=name.removeprefix(ANTHROPIC_PREFIX), max_output_tokens=model.max_output_tokens, timeout_s=model.request_timeout_s)

    def _headers(self) -> dict[str, str]:
        key = os.environ.get(KEY_ENV)
        if not key:
            raise AnthropicError(401, f"{KEY_ENV} is not set (put it in python/src/gpt_pet/.env)")
        headers = {"x-api-key": key, "anthropic-version": API_VERSION}
        if workspace := os.environ.get(WORKSPACE_ENV):  # a key not scoped to one workspace needs it
            headers["anthropic-workspace-id"] = workspace
        return headers

    async def generate_content_async(self, llm_request: LlmRequest, stream: bool = False) -> AsyncGenerator[LlmResponse, None]:
        self._maybe_append_user_content(llm_request)
        body = messages_request(self.model, llm_request, max_tokens=self.max_output_tokens, cache=self.cache)
        data = await post_json(AnthropicError, self.base_url, "/v1/messages", body, timeout_s=self.timeout_s, headers=self._headers())
        yield messages_response(data, answer_tool=RESPOND in {tool["name"] for tool in body.get("tools", [])} and "tool_choice" in body)
