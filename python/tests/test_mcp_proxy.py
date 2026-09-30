"""Offline: splitting a real MCP CallToolResult into image, JSON, and text."""
from __future__ import annotations

import base64

from mcp.types import CallToolResult, ImageContent, TextContent

from gpt_pet.mcp_proxy import parse_result


def test_parse_result_extracts_image_json_and_error_flag() -> None:
    jpeg = b"\xff\xd8\xff\xe0fake"
    result = CallToolResult(
        content=[
            ImageContent(type="image", data=base64.b64encode(jpeg).decode(), mimeType="image/jpeg"),
            TextContent(type="text", text='{"yaw": 90.0, "fov": 70.0}'),
        ],
        isError=False,
    )
    parsed = parse_result(result)
    assert parsed.image == jpeg and parsed.mime_type == "image/jpeg"
    assert parsed.data == {"yaw": 90.0, "fov": 70.0}
    assert parsed.is_error is False

    failed = parse_result(CallToolResult(content=[TextContent(type="text", text="Move blocked")], isError=True))
    assert failed.image is None and failed.data is None and failed.text == "Move blocked" and failed.is_error
