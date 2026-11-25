"""Live: the real ai2thor-mcp server through the real ADK toolset. No LLM calls here."""
from __future__ import annotations

import asyncio
import math
from datetime import timedelta

import pytest
from google.genai import types
from mcp import ClientSession
from mcp.client.streamable_http import streamablehttp_client

from gpt_pet.mcp import build_toolset, mcp_media_callback
from gpt_pet.navwait import status_from_tool_result, wait_for_drive
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


async def test_a_real_drive_is_waited_out_until_it_ends(mcp_server: str, sim_settings: Settings) -> None:
    """Start a real drive, then wait with the brain's own status checks: no model involved."""
    seen: list[str] = []
    async with streamablehttp_client(
        str(sim_settings.mcp.url),
        timeout=timedelta(seconds=FIRST_CALL_TIMEOUT_S),
        sse_read_timeout=timedelta(seconds=FIRST_CALL_TIMEOUT_S),
    ) as (read_stream, write_stream, _):
        async with ClientSession(
            read_stream, write_stream, read_timeout_seconds=timedelta(seconds=FIRST_CALL_TIMEOUT_S)
        ) as session:
            await session.initialize()

            async def call(tool: str, args: dict) -> dict:
                result = await session.call_tool(tool, args)
                dumped = result.model_dump(mode="json", by_alias=True, exclude_none=True)
                return mcp_media_callback(None, {}, None, dumped) or dumped

            # An object's distance is not the drive's length (the planner stops beside it), so try
            # places until one plans a drive of a metre or more, which lasts several polls at the
            # default speed: what the camera sees while turning around (passages first, then the
            # farthest objects), then everything the simulator has seen so far, preferring places
            # about 4 m away. The robot may start anywhere; earlier runs leave it where they
            # stopped. Each new goal preempts the one before, and so does turning.
            tried: list[str] = []

            async def first_real_drive(destination_ids: list[str]) -> dict | None:
                for destination_id in destination_ids:
                    if destination_id in tried:
                        continue
                    tried.append(destination_id)
                    response = await call("set_nav_goal", {"destination_id": destination_id})
                    if not response["isError"] and response["result"]["path"]["length_m"] >= 1.0:
                        return response["result"]
                return None

            started = None
            for _ in range(4):
                view = (await call("get_current_view", {}))["result"]
                seen_now = sorted(view["destinations"], key=lambda d: (d["kind"] != "passage", -d.get("distance_m", 0.0)))
                started = await first_real_drive([d["id"] for d in seen_now])
                if started is not None:
                    break
                await call("do_rotate", {"action": "RotateLeft", "degrees": 90})
            if started is None:
                here = view["agent"]
                known = (await call("list_destinations", {}))["result"]["destinations"]
                known.sort(key=lambda d: abs(math.hypot(d["target"][0] - here["x"], d["target"][1] - here["z"]) - 4.0))
                started = await first_real_drive([d["id"] for d in known])
            assert started is not None, f"no known destination plans a drive of 1 m or more; tried {tried}"

            async def get_status() -> dict:
                result = await session.call_tool("get_nav_status", {})
                current = status_from_tool_result(result.model_dump(mode="json", by_alias=True, exclude_none=True))
                seen.append(current["state"])
                return current

            wait = await wait_for_drive(get_status, started["goal_id"], sim_settings.nav)

    assert wait.outcome in {"succeeded", "failed"}, wait  # the drive ended by itself: no timeout, no error
    assert wait.status is not None and wait.status["goal"]["goal_id"] == started["goal_id"]
    assert seen[0] == "following", seen  # it really waited through the drive
    assert seen[-1] == wait.outcome
    assert all(state in {"planning", "following"} for state in seen[:-1]), seen  # stopped at the first terminal state
    assert 0 < wait.waited_s < sim_settings.nav.max_wait_s


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
