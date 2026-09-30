"""The only module that knows how to reach an MCP server.

`build_toolset` turns the validated `[mcp]` section of a profile into an ADK `McpToolset`.
Swapping the robot backend (ai2thor-mcp, gpt-pet-mcp, anything else that speaks MCP) is an ini
change; a new transport kind is one more `case` here.

`mcp_media_callback` rewrites MCP tool results so camera frames reach Gemini as images.
"""
from __future__ import annotations

import base64
import json
from typing import TYPE_CHECKING, Any

from google.adk.tools.mcp_tool.mcp_session_manager import (
    SseConnectionParams,
    StdioConnectionParams,
    StreamableHTTPConnectionParams,
)
from google.adk.tools.mcp_tool.mcp_toolset import McpToolset
from google.genai import types
from mcp import StdioServerParameters

from gpt_pet.settings import (
    McpSettings,
    SseMcpSettings,
    StdioMcpSettings,
    StreamableHttpMcpSettings,
)

if TYPE_CHECKING:
    from gpt_pet.frames import FrameStore

ConnectionParams = StdioConnectionParams | SseConnectionParams | StreamableHTTPConnectionParams


def build_connection_params(mcp: McpSettings) -> ConnectionParams:
    match mcp:
        case StdioMcpSettings():
            return StdioConnectionParams(
                server_params=StdioServerParameters(
                    command=mcp.command,
                    args=list(mcp.args),
                    cwd=str(mcp.cwd) if mcp.cwd else None,
                ),
                timeout=mcp.timeout_s,
            )
        case StreamableHttpMcpSettings():
            return StreamableHTTPConnectionParams(
                url=str(mcp.url),
                timeout=mcp.connect_timeout_s,
                sse_read_timeout=mcp.read_timeout_s,
            )
        case SseMcpSettings():
            return SseConnectionParams(
                url=str(mcp.url),
                timeout=mcp.connect_timeout_s,
                sse_read_timeout=mcp.read_timeout_s,
            )
    raise TypeError(f"unsupported MCP settings: {type(mcp).__name__}")


def build_toolset(mcp: McpSettings) -> McpToolset:
    """Create the toolset. Nothing connects until the first tool call."""
    return McpToolset(
        connection_params=build_connection_params(mcp),
        tool_filter=list(mcp.tool_filter) or None,
        tool_list_cache_ttl_seconds=mcp.tool_list_cache_ttl_s,
    )


def _parse_text(text: Any) -> Any:
    if not isinstance(text, str):
        return text
    try:
        return json.loads(text)
    except ValueError:
        return text


def mcp_media_callback(tool: Any, args: Any, tool_context: Any, tool_response: Any) -> dict[str, Any] | None:
    """`after_tool_callback`: turn MCP image content into image parts Gemini can see.

    ADK hands the model the raw MCP `CallToolResult` dump, in which a camera frame is a base64
    string. This returns `{"result": ..., "isError": ..., "media": [Part, ...]}`; ADK extracts
    `types.Part` objects found one level deep into function-response image parts. JSON text
    blocks are parsed so the model reads structure instead of an escaped string.

    Returns None (keep the original response) for anything that is not an MCP content list.
    """
    if not isinstance(tool_response, dict):
        return None
    content = tool_response.get("content")
    if not isinstance(content, list):
        return None

    texts: list[Any] = []
    media: list[types.Part] = []
    for item in content:
        if not isinstance(item, dict):
            continue
        kind = item.get("type")
        if kind == "image" and item.get("data"):
            try:
                data = base64.b64decode(item["data"])
            except (ValueError, TypeError):
                continue
            mime_type = item.get("mimeType") or item.get("mime_type") or "image/jpeg"
            media.append(types.Part.from_bytes(data=data, mime_type=mime_type))
        elif kind == "text":
            texts.append(_parse_text(item.get("text", "")))

    result: dict[str, Any] = {
        "result": texts[0] if len(texts) == 1 else texts,
        "isError": bool(tool_response.get("isError") or tool_response.get("is_error")),
    }
    if media:
        result["media"] = media
    return result


def make_media_callback(frames: "FrameStore | None" = None):
    """`after_tool_callback` that also stores camera/map frames for the portal.

    Wraps `mcp_media_callback`; when the tool is one of `frames.IMAGE_TOOLS`, the first image
    part's bytes are put into the `FrameStore` (version bumped, listeners notified).
    """
    from gpt_pet.frames import IMAGE_TOOLS

    def callback(tool: Any, args: Any, tool_context: Any, tool_response: Any) -> dict[str, Any] | None:
        result = mcp_media_callback(tool, args, tool_context, tool_response)
        if frames is None or result is None:
            return result
        image_name = IMAGE_TOOLS.get(str(getattr(tool, "name", "")))
        media = result.get("media") or []
        if image_name and media:
            blob = getattr(media[0], "inline_data", None)
            if blob is not None and blob.data:
                frames.put(image_name, bytes(blob.data), blob.mime_type or "image/jpeg")
        return result

    return callback
