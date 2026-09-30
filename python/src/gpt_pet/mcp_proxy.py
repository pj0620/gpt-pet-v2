"""A persistent MCP client for direct (non-LLM) calls from the server.

Used for things the portal needs from the simulator without going through the executor: the
top-down map after each tick and the simulator-only free camera. All calls run on one worker
task so the underlying anyio task group is entered and left by the same task.
"""
from __future__ import annotations

import asyncio
import base64
import json
import logging
from contextlib import AsyncExitStack
from dataclasses import dataclass
from datetime import timedelta
from typing import Any

log = logging.getLogger("gpt_pet.mcp_proxy")


@dataclass(frozen=True)
class ToolResult:
    image: bytes | None
    mime_type: str | None
    data: dict[str, Any] | None
    text: str
    is_error: bool


def parse_result(result: Any) -> ToolResult:
    """Split a CallToolResult into its first image, first JSON dict, and plain text."""
    image: bytes | None = None
    mime: str | None = None
    data: dict[str, Any] | None = None
    texts: list[str] = []
    for item in getattr(result, "content", []) or []:
        kind = getattr(item, "type", "")
        if kind == "image" and image is None and getattr(item, "data", None):
            image = base64.b64decode(item.data)
            mime = getattr(item, "mimeType", None) or "image/jpeg"
        elif kind == "text":
            text = getattr(item, "text", "") or ""
            texts.append(text)
            if data is None:
                try:
                    parsed = json.loads(text)
                    if isinstance(parsed, dict):
                        data = parsed
                except ValueError:
                    pass
    structured = getattr(result, "structuredContent", None)
    if data is None and isinstance(structured, dict):
        data = structured
    return ToolResult(image=image, mime_type=mime, data=data, text="\n".join(texts), is_error=bool(getattr(result, "isError", False)))


class McpProxy:
    """Serialised tool calls over one streamable-HTTP MCP session, owned by a worker task."""

    def __init__(self, url: str, *, timeout_s: float = 60.0) -> None:
        self.url = url
        self.timeout = timedelta(seconds=timeout_s)
        self._queue: asyncio.Queue[tuple[str, dict[str, Any], asyncio.Future[Any]] | None] = asyncio.Queue()
        self._worker: asyncio.Task[None] | None = None

    async def call(self, tool: str, args: dict[str, Any] | None = None) -> ToolResult:
        if self._worker is None or self._worker.done():
            self._worker = asyncio.create_task(self._run(), name="mcp-proxy")
        future: asyncio.Future[Any] = asyncio.get_running_loop().create_future()
        await self._queue.put((tool, dict(args or {}), future))
        return parse_result(await future)

    async def close(self) -> None:
        if self._worker is None:
            return
        await self._queue.put(None)
        try:
            await asyncio.wait_for(self._worker, timeout=10)
        except (asyncio.TimeoutError, asyncio.CancelledError, Exception):  # noqa: BLE001
            self._worker.cancel()
        self._worker = None

    async def _run(self) -> None:
        from mcp import ClientSession
        from mcp.client.streamable_http import streamablehttp_client

        stack: AsyncExitStack | None = None
        session: ClientSession | None = None

        async def reset() -> None:
            nonlocal stack, session
            if stack is not None:
                try:
                    await stack.aclose()
                except Exception:  # noqa: BLE001
                    pass
            stack, session = None, None

        try:
            while True:
                item = await self._queue.get()
                if item is None:
                    break
                tool, args, future = item
                if future.cancelled():
                    continue
                for attempt in (1, 2):
                    try:
                        if session is None:
                            stack = AsyncExitStack()
                            read, write, _ = await stack.enter_async_context(
                                streamablehttp_client(self.url, timeout=self.timeout, sse_read_timeout=self.timeout)
                            )
                            session = await stack.enter_async_context(ClientSession(read, write, read_timeout_seconds=self.timeout))
                            await session.initialize()
                        result = await session.call_tool(tool, args)
                        if not future.done():
                            future.set_result(result)
                        break
                    except Exception as exc:  # noqa: BLE001 - reconnect once, then report
                        log.warning("MCP proxy call %s failed (attempt %d): %s", tool, attempt, exc)
                        await reset()
                        if attempt == 2 and not future.done():
                            future.set_exception(exc)
        finally:
            await reset()
