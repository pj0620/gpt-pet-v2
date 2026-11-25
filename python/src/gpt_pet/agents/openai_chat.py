"""Cloud models behind OpenAI's Chat Completions API in Gemini's place: OpenAI's own
(`openai/<model>`, e.g. `openai/gpt-5-mini`) and OpenRouter's (`openrouter/<vendor>/<model>`), which
serves many vendors' models behind the same API.

One ADK request becomes one `POST /chat/completions`: the system instruction, the turns with their
images as data URLs, the tool calls with their ids and the tool results (a tool message carries
text only, so a tool's images follow its result as a user message), the tool declarations, and
the goal setter's output schema as a strict `json_schema` response format. Strict mode needs every
property required and no others, so the schema is tightened first: a field that may be left out
must now be given (a field that may be null still may be, and a list may be empty). Reasoning models (gpt-5 and later, the o-series) get
`[model] reasoning_effort` and no temperature, which they reject. On Chat Completions, gpt-5.4 and
later take tools only with `reasoning_effort = none` (the gpt-5 / 5-mini / 5-nano trio has
`minimal` instead); other efforts there need the Responses API, which this adapter does not speak.
On OpenRouter `none` switches reasoning off, and a request with an output schema only goes to
providers that honour it.

The API key comes from the environment (`OPENAI_API_KEY`, `OPENROUTER_API_KEY`), like Gemini's.
"""
from __future__ import annotations

import json
import os
import re
from collections.abc import AsyncGenerator, Iterable, Mapping
from typing import Any

from google.adk.models.base_llm import BaseLlm
from google.adk.models.llm_request import LlmRequest
from google.adk.models.llm_response import LlmResponse
from google.genai import types

from gpt_pet.agents.model_api import (
    ModelApiError,
    as_text,
    call_arguments,
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

PROVIDERS: dict[str, tuple[str, str]] = {
    "openai/": ("https://api.openai.com/v1", "OPENAI_API_KEY"),
    "openrouter/": ("https://openrouter.ai/api/v1", "OPENROUTER_API_KEY"),
}
"""Name prefix -> (API base URL, environment variable holding the key)."""
REASONING_MODELS = re.compile(r"^(gpt-[5-9]|o\d)")
TOOL_IMAGES = "Images returned by {names}."
FINISH_REASONS = {"length": types.FinishReason.MAX_TOKENS, "content_filter": types.FinishReason.SAFETY}


def is_openai_chat(model: str) -> bool:
    return model.startswith(tuple(PROVIDERS))


class OpenAIError(ModelApiError):
    provider = "OpenAI-compatible API"


# --- request -------------------------------------------------------------------------------


DETAILS = {"low": "low", "medium": "auto", "high": "high"}
"""`[model] image_detail` -> the image's `detail`."""


def _data_url(mime_type: str, data: str, detail: str = "") -> dict[str, Any]:
    image: dict[str, Any] = {"url": f"data:{mime_type};base64,{data}"}
    if detail in DETAILS:
        image["detail"] = DETAILS[detail]
    return {"type": "image_url", "image_url": image}


def chat_messages(contents: Iterable[types.Content], system: str = "", *, detail: str = "") -> list[dict[str, Any]]:
    """ADK's turns as Chat Completions messages, images at `detail`. Thoughts are not replayed."""
    messages: list[dict[str, Any]] = [{"role": "system", "content": system}] if system else []
    for content in contents:
        texts: list[str] = []
        images: list[dict[str, Any]] = []
        calls: list[dict[str, Any]] = []
        results: list[dict[str, Any]] = []
        result_images: list[dict[str, Any]] = []
        imaging_tools: list[str] = []
        for part in content.parts or []:
            if part.thought:
                continue
            if part.function_call:
                call = part.function_call
                arguments = json.dumps(dict(call.args or {}), ensure_ascii=False)
                calls.append({"id": call.id or "", "type": "function", "function": {"name": call.name or "", "arguments": arguments}})
            elif part.function_response:
                response = part.function_response
                results.append({"role": "tool", "tool_call_id": response.id or "", "content": as_text(response.response)})
                found = [_data_url(*image, detail) for image in tool_result_images(response)]
                if found:
                    result_images += found
                    imaging_tools.append(response.name or "a tool")
            elif part.text:
                texts.append(part.text)
            elif image := image_blob(part.inline_data):
                images.append(_data_url(*image, detail))
        if content.role == "model":
            message: dict[str, Any] = {"role": "assistant", "content": "\n".join(texts) or None}
            if calls:
                message["tool_calls"] = calls
            messages.append(message)
            continue
        messages += results
        if result_images:
            note = {"type": "text", "text": TOOL_IMAGES.format(names=", ".join(imaging_tools))}
            messages.append({"role": "user", "content": [note, *result_images]})
        if images:
            messages.append({"role": "user", "content": [*({"type": "text", "text": text} for text in texts), *images]})
        elif texts:
            messages.append({"role": "user", "content": "\n".join(texts)})
    return messages


def strict_schema(schema: Mapping[str, Any]) -> dict[str, Any]:
    """`schema` as OpenAI's strict structured outputs accept it: every object requires all its
    properties and allows no others, with no `default` or `title` keywords. Properties keep their
    types: making an optional one nullable would let the model send null where the answer's
    model (e.g. a list with a default) refuses it."""

    def tighten(node: Any) -> Any:
        if isinstance(node, list):
            return [tighten(item) for item in node]
        if not isinstance(node, Mapping):
            return node
        out: dict[str, Any] = {}
        for key, value in node.items():
            if key in ("default", "title"):
                continue
            if key in ("properties", "$defs", "definitions") and isinstance(value, Mapping):
                out[key] = {name: tighten(item) for name, item in value.items()}
            else:
                out[key] = tighten(value)
        if out.get("type") == "object" and isinstance(out.get("properties"), dict):
            out["required"] = list(out["properties"])
            out["additionalProperties"] = False
        return out

    return tighten(schema)


def chat_request(
    model: str,
    request: LlmRequest,
    *,
    max_tokens: int,
    reasoning_effort: str = "",
    image_detail: str = "",
    openrouter: bool = False,
) -> dict[str, Any]:
    """The `/chat/completions` body for one ADK request."""
    config = request.config
    messages = chat_messages(with_call_ids(request.contents or []), system_text(config), detail=image_detail)
    body: dict[str, Any] = {"model": model, "messages": messages}
    body["max_tokens" if openrouter else "max_completion_tokens"] = (config.max_output_tokens if config else None) or max_tokens
    reasoning = bool(reasoning_effort) or bool(REASONING_MODELS.match(model.split("/")[-1]))
    if reasoning_effort:
        if openrouter:
            body["reasoning"] = {"enabled": False} if reasoning_effort == "none" else {"effort": reasoning_effort}
        else:
            body["reasoning_effort"] = reasoning_effort
    if not reasoning and config is not None and config.temperature is not None:
        body["temperature"] = config.temperature
    tools = [
        {"type": "function", "function": {"name": name, "description": description, "parameters": parameters}}
        for name, description, parameters in function_declarations(config)
    ]
    if tools:
        body["tools"] = tools
    schema = output_schema(config)
    if schema:
        name = re.sub(r"[^a-zA-Z0-9_-]", "_", str(schema.get("title") or "answer"))
        body["response_format"] = {"type": "json_schema", "json_schema": {"name": name, "schema": strict_schema(schema), "strict": True}}
    if openrouter:
        body["usage"] = {"include": True}  # OpenRouter then reports what the call cost
        if schema:
            body["provider"] = {"require_parameters": True}  # no provider that would drop the schema
    return body


# --- response ------------------------------------------------------------------------------


def chat_response(data: Mapping[str, Any]) -> LlmResponse:
    """One `/chat/completions` reply as ADK's response: text, tool calls with their ids, tokens
    (cached and reasoning ones apart)."""
    choice = (data.get("choices") or [{}])[0]
    message = choice.get("message") or {}
    parts: list[types.Part] = []
    reasoning = message.get("reasoning") or message.get("reasoning_content")
    if reasoning:
        parts.append(types.Part(text=str(reasoning), thought=True))
    if message.get("content"):
        parts.append(types.Part(text=message["content"]))
    elif message.get("refusal"):
        parts.append(types.Part(text=message["refusal"]))
    for call in message.get("tool_calls") or []:
        function = call.get("function") or {}
        parts.append(
            types.Part(function_call=types.FunctionCall(id=call.get("id"), name=function.get("name") or "", args=call_arguments(function.get("arguments"))))
        )
    if not parts:
        parts.append(types.Part(text=""))
    usage = data.get("usage") or {}
    completion = int(usage.get("completion_tokens") or 0)
    thinking = int((usage.get("completion_tokens_details") or {}).get("reasoning_tokens") or 0)
    billing: dict[str, Any] = {"cost_usd": usage["cost"]} if usage.get("cost") is not None else {}
    written = int((usage.get("prompt_tokens_details") or {}).get("cache_write_tokens") or 0)
    if written:  # GPT-5.6 and later bill cache writes above plain input
        billing["cache_write_tokens"] = written
    return LlmResponse(
        content=types.Content(role="model", parts=parts),
        usage_metadata=usage_metadata(
            prompt=int(usage.get("prompt_tokens") or 0),
            output=completion - thinking,
            cached=int((usage.get("prompt_tokens_details") or {}).get("cached_tokens") or 0),
            thinking=thinking,
        ),
        finish_reason=FINISH_REASONS.get(str(choice.get("finish_reason")), types.FinishReason.STOP),
        model_version=data.get("model"),
        custom_metadata={"billing": billing} if billing else None,
    )


# --- the model -----------------------------------------------------------------------------


class OpenAIChatLlm(BaseLlm):
    """A model behind the Chat Completions API standing in for Gemini (see the module docstring).
    `model` is the API's model id, e.g. `gpt-5-mini` or `mistralai/mistral-small-3.2-24b-instruct`."""

    base_url: str = "https://api.openai.com/v1"
    api_key_env: str = "OPENAI_API_KEY"
    reasoning_effort: str = ""
    image_detail: str = ""
    max_output_tokens: int = 2048
    timeout_s: float = 0
    """Per call; 0 waits forever."""

    @classmethod
    def from_settings(cls, name: str, model: ModelSettings) -> "OpenAIChatLlm":
        prefix = next(prefix for prefix in PROVIDERS if name.startswith(prefix))
        base_url, key_env = PROVIDERS[prefix]
        return cls(
            model=name.removeprefix(prefix),
            base_url=base_url,
            api_key_env=key_env,
            reasoning_effort=model.reasoning_effort,
            image_detail=model.image_detail,
            max_output_tokens=model.max_output_tokens,
            timeout_s=model.request_timeout_s,
        )

    @property
    def openrouter(self) -> bool:
        return "openrouter.ai" in self.base_url

    def _headers(self) -> dict[str, str]:
        key = os.environ.get(self.api_key_env)
        if not key:
            raise OpenAIError(401, f"{self.api_key_env} is not set (put it in python/src/gpt_pet/.env)")
        return {"Authorization": f"Bearer {key}"}

    async def generate_content_async(self, llm_request: LlmRequest, stream: bool = False) -> AsyncGenerator[LlmResponse, None]:
        self._maybe_append_user_content(llm_request)
        body = chat_request(
            self.model,
            llm_request,
            max_tokens=self.max_output_tokens,
            reasoning_effort=self.reasoning_effort,
            image_detail=self.image_detail,
            openrouter=self.openrouter,
        )
        data = await post_json(OpenAIError, self.base_url, "/chat/completions", body, timeout_s=self.timeout_s, headers=self._headers())
        yield chat_response(data)
