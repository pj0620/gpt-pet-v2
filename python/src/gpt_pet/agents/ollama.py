"""Local models in Gemini's place: Ollama's chat API as an ADK model.

`[model] goal_setter` and `executor` name a local model as `ollama/<tag>` (e.g.
`ollama/qwen3-vl:8b`). The executor's model needs Ollama's `tools` capability, and both need
`vision` to read the camera frames and the map.

`OllamaLlm` turns one ADK request into one `POST /api/chat` (not streamed): the system
instruction, the turns with their images, the tool calls and tool results (a tool's images follow
its result as a user message, since tool messages carry text only), the tool declarations, and
the goal setter's output schema as `format`, which Ollama enforces while decoding. Ollama's own
timings come back in the response's `custom_metadata["ollama"]`, so the stats can tell loading
the model, reading the prompt and writing the answer apart.

Ollama fits a prompt longer than the context by dropping whole earlier messages (it keeps the
system prompt and the latest turn) without saying so: the executor would lose its goal, which is
its first message. So the context is set explicitly (`[model] ollama_num_ctx`) and a prompt that
nearly fills it is logged. A model loads with the options of the request that loads it, so
`warm_up` loads it with the same ones; otherwise the first call reloads it.
"""
from __future__ import annotations

import logging
from collections.abc import AsyncGenerator, Iterable, Mapping
from typing import Any

import httpx
from google.adk.models.base_llm import BaseLlm
from google.adk.models.llm_request import LlmRequest
from google.adk.models.llm_response import LlmResponse
from google.genai import types
from pydantic import PrivateAttr

from gpt_pet.agents.model_api import (
    CONNECT_TIMEOUT_S,
    ModelApiError,
    as_text,
    call_arguments,
    function_declarations,
    image_blob,
    json_schema,
    output_schema,
    post_json,
    system_text,
    tool_result_images,
    usage_metadata,
)
from gpt_pet.settings import ModelSettings

__all__ = ["OllamaError", "OllamaLlm", "chat_messages", "chat_request", "chat_response", "json_schema", "local_models"]

OLLAMA_PREFIX = "ollama/"
CHAT = "/api/chat"
SHOW = "/api/show"
PS = "/api/ps"
LOAD_TIMEOUT_S = 300.0
"""For loading, unloading and describing a model, whatever the per-call timeout."""
RELOAD_S = 1.0
"""A call that spent longer than this loading its model found it unloaded (evicted, or cold)."""
FULL_CONTEXT = 0.9
"""A prompt this share of the context or more has probably lost earlier messages."""
TOOL_IMAGES = "Images returned by {names}."

log = logging.getLogger("gpt_pet.ollama")


def is_ollama(model: str) -> bool:
    return model.startswith(OLLAMA_PREFIX)


class OllamaError(ModelApiError):
    """Ollama answered with an HTTP error: a bad request (4xx), or a model runner that failed (5xx)."""

    provider = "Ollama"


# --- request -------------------------------------------------------------------------------


def chat_messages(contents: Iterable[types.Content], system: str = "") -> list[dict[str, Any]]:
    """ADK's turns as Ollama chat messages. Thoughts are not replayed."""
    messages: list[dict[str, Any]] = [{"role": "system", "content": system}] if system else []
    for content in contents:
        texts: list[str] = []
        images: list[str] = []
        calls: list[dict[str, Any]] = []
        results: list[dict[str, Any]] = []
        result_images: list[str] = []
        imaging_tools: list[str] = []
        for part in content.parts or []:
            if part.thought:
                continue
            if part.function_call:
                call = part.function_call
                calls.append({"function": {"name": call.name or "", "arguments": dict(call.args or {})}})
            elif part.function_response:
                response = part.function_response
                results.append({"role": "tool", "tool_name": response.name or "", "content": as_text(response.response)})
                found = [data for _, data in tool_result_images(response)]
                if found:
                    result_images += found
                    imaging_tools.append(response.name or "a tool")
            elif part.text:
                texts.append(part.text)
            elif image := image_blob(part.inline_data):
                images.append(image[1])
        if content.role == "model":
            message: dict[str, Any] = {"role": "assistant", "content": "\n".join(texts)}
            if calls:
                message["tool_calls"] = calls
            messages.append(message)
            continue
        messages += results
        if result_images:
            note = TOOL_IMAGES.format(names=", ".join(imaging_tools))
            messages.append({"role": "user", "content": note, "images": result_images})
        if texts or images:
            messages.append({"role": "user", "content": "\n".join(texts), **({"images": images} if images else {})})
    return messages


def chat_request(
    model: str,
    request: LlmRequest,
    *,
    num_ctx: int,
    num_predict: int,
    keep_alive: str,
    think: bool | None = None,
) -> dict[str, Any]:
    """The `/api/chat` body for one ADK request. `think` is sent only when given (models that
    cannot think reject it)."""
    config = request.config
    options: dict[str, Any] = {"num_ctx": num_ctx, "num_predict": (config.max_output_tokens if config else None) or num_predict}
    if config is not None and config.temperature is not None:
        options["temperature"] = config.temperature
    body: dict[str, Any] = {
        "model": model,
        "messages": chat_messages(request.contents or [], system_text(config)),
        "stream": False,
        "keep_alive": keep_alive,
        "options": options,
    }
    tools = [
        {"type": "function", "function": {"name": name, "description": description, "parameters": parameters}}
        for name, description, parameters in function_declarations(config)
    ]
    if tools:
        body["tools"] = tools
    schema = output_schema(config)
    if schema:
        body["format"] = schema
    if think is not None:
        body["think"] = think
    return body


# --- response ------------------------------------------------------------------------------


def timings(data: Mapping[str, Any]) -> dict[str, Any]:
    """Ollama's account of one call (it reports nanoseconds)."""

    def seconds(key: str) -> float:
        return round(int(data.get(key) or 0) / 1e9, 3)

    return {
        "load_s": seconds("load_duration"),
        "prompt_tokens": int(data.get("prompt_eval_count") or 0),
        "prompt_s": seconds("prompt_eval_duration"),
        "output_tokens": int(data.get("eval_count") or 0),
        "output_s": seconds("eval_duration"),
        "total_s": seconds("total_duration"),
    }


def chat_response(data: Mapping[str, Any]) -> LlmResponse:
    """One `/api/chat` reply as ADK's response: thinking, text, tool calls, token counts, timings."""
    message = data.get("message") or {}
    parts: list[types.Part] = []
    if message.get("thinking"):
        parts.append(types.Part(text=message["thinking"], thought=True))
    if message.get("content"):
        parts.append(types.Part(text=message["content"]))
    for call in message.get("tool_calls") or []:
        function = call.get("function") or {}
        parts.append(types.Part(function_call=types.FunctionCall(name=function.get("name") or "", args=call_arguments(function.get("arguments")))))
    if not parts:
        parts.append(types.Part(text=""))
    timed = timings(data)
    finished = types.FinishReason.MAX_TOKENS if data.get("done_reason") == "length" else types.FinishReason.STOP
    return LlmResponse(
        content=types.Content(role="model", parts=parts),
        usage_metadata=usage_metadata(prompt=timed["prompt_tokens"], output=timed["output_tokens"]),
        finish_reason=finished,
        custom_metadata={"ollama": timed},
    )


# --- the model -----------------------------------------------------------------------------


class OllamaLlm(BaseLlm):
    """A model served by Ollama standing in for Gemini (see the module docstring). `model` is the
    Ollama tag, e.g. `qwen3-vl:8b`."""

    base_url: str = "http://127.0.0.1:11434"
    num_ctx: int = 16384
    num_predict: int = 1024
    keep_alive: str = "30m"
    think: bool = False
    timeout_s: float = 0
    """Per call; 0 waits forever."""
    _capabilities: frozenset[str] | None = PrivateAttr(default=None)

    @classmethod
    def from_settings(cls, name: str, model: ModelSettings) -> "OllamaLlm":
        return cls(
            model=name.removeprefix(OLLAMA_PREFIX),
            base_url=str(model.ollama_url).rstrip("/"),
            num_ctx=model.ollama_num_ctx,
            num_predict=model.ollama_num_predict,
            keep_alive=model.ollama_keep_alive,
            think=model.ollama_think,
            timeout_s=model.request_timeout_s,
        )

    async def _post(self, path: str, body: Mapping[str, Any], *, timeout_s: float | None = None) -> dict[str, Any]:
        return await post_json(OllamaError, self.base_url, path, body, timeout_s=self.timeout_s if timeout_s is None else timeout_s)

    async def model_capabilities(self) -> frozenset[str]:
        """What Ollama says the model can do (`vision`, `tools`, `thinking`, ...), asked once."""
        if self._capabilities is None:
            shown = await self._post(SHOW, {"model": self.model}, timeout_s=LOAD_TIMEOUT_S)
            self._capabilities = frozenset(shown.get("capabilities") or [])
        return self._capabilities

    async def warm_up(self) -> float:
        """Load the model with the options it is called with; returns how long loading took."""
        await self.model_capabilities()
        body = {"model": self.model, "messages": [], "keep_alive": self.keep_alive, "options": {"num_ctx": self.num_ctx}}
        return timings(await self._post(CHAT, body, timeout_s=LOAD_TIMEOUT_S))["load_s"]

    async def unload(self) -> None:
        await self._post(CHAT, {"model": self.model, "messages": [], "keep_alive": 0}, timeout_s=LOAD_TIMEOUT_S)

    async def loaded(self) -> list[dict[str, Any]]:
        """Every model Ollama holds in memory now: name, size and the part of it on the GPU (GB),
        and context, as `ollama ps` shows them."""
        async with httpx.AsyncClient(base_url=self.base_url, timeout=CONNECT_TIMEOUT_S) as client:
            response = await client.get(PS)
        response.raise_for_status()
        return [
            {
                "name": item.get("name") or item.get("model"),
                "size_gb": round(int(item.get("size") or 0) / 1e9, 2),
                "gpu_gb": round(int(item.get("size_vram") or 0) / 1e9, 2),
                "context": item.get("context_length"),
            }
            for item in response.json().get("models") or []
        ]

    async def generate_content_async(self, llm_request: LlmRequest, stream: bool = False) -> AsyncGenerator[LlmResponse, None]:
        self._maybe_append_user_content(llm_request)
        think = self.think if "thinking" in await self.model_capabilities() else None
        body = chat_request(
            self.model, llm_request, num_ctx=self.num_ctx, num_predict=self.num_predict, keep_alive=self.keep_alive, think=think
        )
        response = chat_response(await self._post(CHAT, body))
        timed = (response.custom_metadata or {})["ollama"]
        if timed["load_s"] > RELOAD_S:
            log.info("%s was not loaded: %.1f s to load it", self.model, timed["load_s"])
        if timed["prompt_tokens"] >= FULL_CONTEXT * self.num_ctx:
            log.warning("%s: a %d-token prompt nearly fills the context (%d); earlier messages may be lost", self.model, timed["prompt_tokens"], self.num_ctx)
        yield response


def local_models(model: ModelSettings) -> list[OllamaLlm]:
    """The distinct local models the goal setter and the executor run (none for Gemini)."""
    names = dict.fromkeys(name for name in (model.goal_setter, model.executor) if is_ollama(name))
    return [OllamaLlm.from_settings(name, model) for name in names]
