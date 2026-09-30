"""Live: the real ai2thor-mcp server through the real ADK toolset. No LLM calls here."""
from __future__ import annotations

import asyncio
from datetime import timedelta

import pytest
from google.genai import types
from mcp import ClientSession
from mcp.client.streamable_http import streamablehttp_client

from gpt_pet.mcp import build_toolset, mcp_media_callback
from gpt_pet.settings import Settings, load_settings

pytestmark = [pytest.mark.live, pytest.mark.timeout(900)]

FIRST_CALL_TIMEOUT_S = 600  # the first tool call boots Unity and may download the build once


async def test_toolset_lists_exactly_the_filtered_tools(mcp_server: str, sim_settings: Settings) -> None:
    toolset = build_toolset(sim_settings.mcp)
    try:
        tools = await toolset.get_tools()
    finally:
        await toolset.close()
    names = sorted(tool.name for tool in tools)
    assert names == sorted(sim_settings.mcp.tool_filter)
    assert "start_overhead_recording" not in names
    assert "end_overhead_recording" not in names


async def test_get_current_view_real_result_becomes_image_part(mcp_server: str, sim_settings: Settings) -> None:
    async with streamablehttp_client(
        str(sim_settings.mcp.url),
        timeout=timedelta(seconds=FIRST_CALL_TIMEOUT_S),
        sse_read_timeout=timedelta(seconds=FIRST_CALL_TIMEOUT_S),
    ) as (read_stream, write_stream, _):
        async with ClientSession(
            read_stream, write_stream, read_timeout_seconds=timedelta(seconds=FIRST_CALL_TIMEOUT_S)
        ) as session:
            await session.initialize()
            result = await session.call_tool("get_current_view", {})

    # The same shape ADK hands to after_tool_callback: the dumped CallToolResult.
    dumped = result.model_dump(mode="json", by_alias=True, exclude_none=True)
    converted = mcp_media_callback(tool=None, args={}, tool_context=None, tool_response=dumped)

    assert converted is not None
    assert converted["isError"] is False
    assert converted["media"], "the camera frame should become an image part"
    part = converted["media"][0]
    assert isinstance(part, types.Part)
    assert part.inline_data is not None
    assert (part.inline_data.mime_type or "").startswith("image/")
    assert len(part.inline_data.data or b"") > 1000
    assert isinstance(converted["result"], dict)
    assert {"agent", "destinations"} <= set(converted["result"])


async def test_real_profile_is_defined_but_unreachable() -> None:
    """Documents the gap: gpt-pet-mcp has no MCP server yet. Fails the day it answers."""
    settings = load_settings("real")
    toolset = build_toolset(settings.mcp)
    try:
        try:
            tools = await asyncio.wait_for(toolset.get_tools(), timeout=settings.mcp.connect_timeout_s + 60)
        except Exception:  # noqa: BLE001 - any connection failure is the expected outcome today
            return
        assert tools == [], "gpt-pet-mcp answered with tools: promote this into a real-profile smoke test"
    finally:
        await toolset.close()
