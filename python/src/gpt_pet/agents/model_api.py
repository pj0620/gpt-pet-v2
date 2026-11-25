"""What the agents' non-Gemini models share: reading ADK's request (system instruction, tool
declarations, output schema, images) and calling an HTTP API.

ADK hands every model the same `LlmRequest` built for Gemini; each adapter (`gpt_pet.agents.ollama`,
`openai_chat`, `anthropic`) turns it into its API's body with these helpers and turns the reply
back into an `LlmResponse` with genai usage metadata, so the stats count every model alike.
"""
from __future__ import annotations

import asyncio
import base64
import json
from collections import deque
from collections.abc import Iterable, Mapping
from typing import Any

import httpx
from google.genai import types
from pydantic import BaseModel

CONNECT_TIMEOUT_S = 5.0
TRANSIENT_STATUS = frozenset({408, 409, 429, 529})
"""Besides 5xx: timeout, conflict, rate limit, Anthropic's "overloaded"."""


class ModelApiError(RuntimeError):
    """A model API answered with an HTTP error: a bad request (4xx) or a failure on its side (5xx)."""

    provider = "model API"

    def __init__(self, status: int, message: str) -> None:
        super().__init__(f"{self.provider} {status}: {message}")
        self.status = status

    @property
    def transient(self) -> bool:
        """May well succeed next turn: rate limited, overloaded, or a server error."""
        return self.status >= 500 or self.status in TRANSIENT_STATUS


async def post_json(
    error: type[ModelApiError],
    base_url: str,
    path: str,
    body: Mapping[str, Any],
    *,
    timeout_s: float,
    headers: Mapping[str, str] | None = None,
) -> dict[str, Any]:
    """POST a JSON body and return the JSON reply; an HTTP error raises `error` with its message.
    `timeout_s` bounds the whole call (TimeoutError), 0 waits forever (connecting still times
    out). httpx's own timeouts bound each read only, and OpenRouter keeps a slow answer's
    connection open with whitespace: one call hung for 100 s under a 20 s timeout (2026-10-03)."""
    timeout = httpx.Timeout(timeout_s or None, connect=CONNECT_TIMEOUT_S)

    async def send() -> httpx.Response:
        async with httpx.AsyncClient(base_url=base_url, timeout=timeout) as client:
            return await client.post(path, json=body, headers=dict(headers or {}))

    response = await asyncio.wait_for(send(), timeout_s) if timeout_s else await send()
    if response.status_code >= 400:
        raise error(response.status_code, error_message(response))
    return response.json()


def error_message(response: httpx.Response) -> str:
    try:
        found = response.json().get("error")
    except ValueError:
        return response.text[:500]
    if isinstance(found, Mapping):
        found = found.get("message") or json.dumps(found)
    return str(found or response.text)[:500]


# --- reading ADK's request -----------------------------------------------------------------


def image_blob(blob: Any) -> tuple[str, str] | None:
    """(mime type, base64 data) of an image blob; None for anything else."""
    if blob is None or not blob.data or not str(blob.mime_type or "").startswith("image/"):
        return None
    return str(blob.mime_type), base64.b64encode(bytes(blob.data)).decode("ascii")


def tool_result_images(response: types.FunctionResponse) -> list[tuple[str, str]]:
    """The images a tool attached to its result (ADK keeps them beside the JSON)."""
    return [image for part in response.parts or [] if (image := image_blob(part.inline_data))]


def with_call_ids(contents: Iterable[types.Content]) -> list[types.Content]:
    """`contents` with an id on every tool call and result. ADK strips the ids it made up itself
    (`adk-...`, which include System 1's calls) before a request reaches the model, but OpenAI pairs
    a result with its call by id and Anthropic refuses an empty one. A call without an id gets
    `call_<n>`; a result without one takes the id of the earliest unanswered call to the same tool.
    The originals are left alone: they belong to the session's events."""
    pending: dict[str, deque[str]] = {}
    count = 0
    out: list[types.Content] = []
    for content in contents:
        parts = list(content.parts or [])
        changed = False
        for index, part in enumerate(parts):
            if part.function_call is not None:
                call = part.function_call
                call_id = call.id
                if not call_id:
                    count += 1
                    call_id = f"call_{count}"
                    parts[index] = part.model_copy(update={"function_call": call.model_copy(update={"id": call_id})})
                    changed = True
                pending.setdefault(call.name or "", deque()).append(call_id)
            elif part.function_response is not None:
                response = part.function_response
                queue = pending.get(response.name or "")
                if response.id:
                    if queue and response.id in queue:
                        queue.remove(response.id)
                    continue
                if queue:
                    response_id = queue.popleft()
                else:  # a result whose call is not in this request
                    count += 1
                    response_id = f"call_{count}"
                parts[index] = part.model_copy(update={"function_response": response.model_copy(update={"id": response_id})})
                changed = True
        out.append(content.model_copy(update={"parts": parts}) if changed else content)
    return out


def as_text(value: Any) -> str:
    return value if isinstance(value, str) else json.dumps(value, ensure_ascii=False, default=str)


def _lowercase_types(schema: Any) -> Any:
    """A genai schema dump as JSON schema: genai spells types in capitals (`OBJECT`)."""
    if isinstance(schema, list):
        return [_lowercase_types(item) for item in schema]
    if not isinstance(schema, dict):
        return schema
    return {
        key: (value.lower() if key == "type" and isinstance(value, str) else _lowercase_types(value))
        for key, value in schema.items()
    }


def json_schema(schema: Any) -> dict[str, Any] | None:
    """A JSON schema from what ADK hands a model: a dict, a pydantic model class or a genai Schema."""
    if schema is None:
        return None
    if isinstance(schema, dict):
        return schema
    if isinstance(schema, type) and issubclass(schema, BaseModel):
        return schema.model_json_schema()
    if isinstance(schema, types.Schema):
        return _lowercase_types(schema.model_dump(mode="json", by_alias=True, exclude_none=True))
    raise TypeError(f"no JSON schema for {schema!r}")


def output_schema(config: types.GenerateContentConfig | None) -> dict[str, Any] | None:
    """The JSON schema the answer must match (the goal setter's decision), if any."""
    if config is None:
        return None
    return config.response_json_schema or json_schema(config.response_schema)


def system_text(config: types.GenerateContentConfig | None) -> str:
    instruction = config.system_instruction if config is not None else None
    if instruction is None:
        return ""
    if isinstance(instruction, str):
        return instruction
    if isinstance(instruction, types.Content):
        parts: list[Any] = list(instruction.parts or [])
    elif isinstance(instruction, types.Part):
        parts = [instruction]
    else:
        parts = [types.Part(text=item) if isinstance(item, str) else item for item in instruction]
    return "\n".join(part.text for part in parts if getattr(part, "text", None))


def function_declarations(config: types.GenerateContentConfig | None) -> list[tuple[str, str, dict[str, Any]]]:
    """(name, description, JSON schema of the parameters) of every tool the request declares."""
    declared = []
    for tool in (config.tools if config is not None else None) or []:
        for function in getattr(tool, "function_declarations", None) or []:
            parameters = function.parameters_json_schema or json_schema(function.parameters)
            declared.append((function.name or "", function.description or "", parameters or {"type": "object", "properties": {}}))
    return declared


def call_arguments(value: Any) -> dict[str, Any]:
    """A tool call's arguments, whether the API sent an object or a JSON string."""
    if isinstance(value, str):
        try:
            value = json.loads(value)
        except ValueError:
            return {}
    return dict(value) if isinstance(value, Mapping) else {}


def usage_metadata(
    *, prompt: int, output: int, cached: int = 0, thinking: int = 0
) -> types.GenerateContentResponseUsageMetadata:
    """Token counts the way Gemini reports them: `prompt` includes `cached`; `output` excludes
    `thinking`."""
    return types.GenerateContentResponseUsageMetadata(
        prompt_token_count=prompt,
        cached_content_token_count=cached or None,
        candidates_token_count=output,
        thoughts_token_count=thinking or None,
        total_token_count=prompt + output + thinking,
    )
