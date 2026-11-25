"""Offline: the trace recorder (what the pet saw, did, and where) and its drawings."""
from __future__ import annotations

import asyncio
import io
import json
from types import SimpleNamespace

from google.adk.events.event import Event
from google.genai import types
from PIL import Image

from gpt_pet.trace import TraceRecorder, bearing_from_box, result_summary, view_summary
from gpt_pet.trace_report import load, next_actions, render, timeline

VIEW = {
    "agent": {"x": 3.0, "z": -1.5, "yaw": 90.0},
    "camera": {"height_m": 0.3, "tilt_deg": 27.0},
    "attention": {"state": "framed", "target": "obj:person_0", "distance_m": 1.4, "bearing_deg": -3.0, "tilt_deg": 27.0, "seen_s": 4.0},
    "destinations": [
        {"marker": 1, "id": "obj:person_0", "kind": "object", "label": "Person", "description": "Person, 1.4 m ahead", "distance_m": 1.4, "bbox": [200, 0, 300, 400]},
        {"marker": 2, "id": "psg:3", "kind": "passage", "label": "doorway", "distance_m": 3.2, "region": [10, 10, 60, 200]},
    ],
}


def jpeg(colour: tuple[int, int, int] = (200, 30, 30)) -> bytes:
    buffer = io.BytesIO()
    Image.new("RGB", (500, 500), colour).save(buffer, format="JPEG")
    return buffer.getvalue()


def call_event(call_id: str, name: str, args: dict, s1: dict | None = None) -> Event:
    part = types.Part(function_call=types.FunctionCall(id=call_id, name=name, args=args))
    return Event(author="executor", invocation_id="inv", content=types.Content(role="model", parts=[part]), custom_metadata={"s1": s1} if s1 else None)


def response_event(call_id: str, name: str, response: dict, image: bytes | None = None) -> Event:
    parts = [types.FunctionResponsePart(inline_data=types.FunctionResponseBlob(mime_type="image/jpeg", data=image))] if image else None
    function_response = types.FunctionResponse(id=call_id, name=name, response=response, parts=parts)
    return Event(author="executor", invocation_id="inv", content=types.Content(role="user", parts=[types.Part(function_response=function_response)]))


def text_event(author: str, text: str) -> Event:
    return Event(author=author, invocation_id="inv", content=types.Content(role="model", parts=[types.Part(text=text)]))


class FakeServer:
    """Answers the sampler's three tools like ai2thor-mcp does (the real one is covered live)."""

    def __init__(self, missing: tuple[str, ...] = ()) -> None:
        self.missing = missing
        self.calls: list[str] = []

    async def call(self, tool: str, args: dict | None = None):
        self.calls.append(tool)
        if tool in self.missing:
            return SimpleNamespace(is_error=True, data=None, text=f"Unknown tool: {tool}")
        data = {
            "get_nav_status": {"state": "following", "agent": {"x": 3.04, "z": -1.52, "yaw": 90.0}, "goal": {"label": "doorway"}},
            "get_attention_status": {"state": "framed", "distance_m": 1.4, "bearing_deg": -3.0, "moving": False, "tilt_deg": 27.0},
            "sim_person_state": {"mode": "wander", "x": 4.4, "z": -1.6, "yaw": 270.0, "speed_mps": 0.3, "robot": {"x": 3.04, "z": -1.52, "yaw": 90, "distance_m": 1.36}},
        }[tool]
        return SimpleNamespace(is_error=False, data=data, text=json.dumps(data))


def test_bearing_from_the_box_and_compact_summaries() -> None:
    assert bearing_from_box([200, 0, 300, 400]) == 0.0
    assert bearing_from_box([450, 0, 500, 400]) > 25.0 and bearing_from_box([0, 0, 50, 400]) < -25.0
    assert bearing_from_box(None) is None
    summary = view_summary(VIEW)
    assert summary["people"] == [{"id": "obj:person_0", "distance_m": 1.4, "bearing_deg": 0.0, "description": "Person, 1.4 m ahead"}]
    assert [s["label"] for s in summary["seen"]] == ["Person", "doorway"] and summary["camera"]["tilt_deg"] == 27.0
    assert summary["attention"]["state"] == "framed"
    drive = {"result": {"goal_id": "g1", "path": {"length_m": 2.4}}, "isError": False, "drive": {"outcome": "succeeded", "waited_s": 7.1, "final_status": {"progress": {"distance_traveled_m": 2.3}, "agent": {"x": 5.0, "z": -1.5, "yaw": 90}}}}
    assert result_summary("set_nav_goal", drive)["drive"] == {"outcome": "succeeded", "waited_s": 7.1, "traveled_m": 2.3, "agent": {"x": 5.0, "z": -1.5, "yaw": 90}, "reason": None, "note": None}
    assert result_summary("set_nav_goal", drive)["planned_m"] == 2.4
    assert result_summary("do_rotate", {"error": "goal canceled: the owner gave a new goal"}) == {"ok": False, "error": "goal canceled: the owner gave a new goal"}


def test_the_recorder_keeps_views_with_frames_actions_texts_and_ground_truth(tmp_path) -> None:
    trace = TraceRecorder(tmp_path / "run", url=None, meta={"run_id": "r1"})
    trace.event(text_event("goal_setter", '{"previous_goal_status":"none","goal":"say hi to the person"}'), 1)
    trace.event(call_event("c1", "get_current_view", {}, s1={"by": "rules", "action": "get_current_view"}), 1)
    trace.event(response_event("c1", "get_current_view", {"result": VIEW, "isError": False}, image=jpeg()), 1)
    trace.event(call_event("c2", "set_nav_goal", {"destination_id": "psg:3"}), 1)
    trace.event(response_event("c2", "set_nav_goal", {"result": {"goal_id": "g1"}, "isError": False, "drive": {"outcome": "canceled", "waited_s": 1.5, "note": "Stopped: a person came into view."}}), 1)
    trace.event(text_event("executor", "Saw: Person 1.4 m ahead, standing.\nDid: drove toward the doorway."), 1)
    trace.tick(SimpleNamespace(summary_dict=lambda: {"number": 1, "seconds": 12.0, "tools": ["get_current_view", "set_nav_goal"], "llm_calls": 2}))
    trace.notice({"kind": "rest", "tick": 1, "text": "Resting 10 s."})
    server = FakeServer(missing=())
    sample = asyncio.run(trace.sample_once(server))
    assert sample["robot"] == {"x": 3.04, "z": -1.52, "yaw": 90.0} and sample["nav"] == "following" and sample["nav_goal"] == "doorway"
    assert sample["person"]["mode"] == "wander" and sample["person"]["robot_distance_m"] == 1.36 and sample["tilt_deg"] == 27.0
    trace.close()

    meta, records = load(tmp_path / "run")
    assert meta["run_id"] == "r1"
    assert [r["type"] for r in records] == ["text", "view", "call", "result", "text", "tick", "notice", "sample"]
    view = records[1]
    assert view["by"] == "rules" and view["seq"] == 1 and view["frame"] == "frames/00001-t1.jpg"
    assert (tmp_path / "run" / view["frame"]).stat().st_size > 1000
    assert view["agent"] == {"x": 3.0, "z": -1.5, "yaw": 90.0} and view["people"][0]["distance_m"] == 1.4
    assert records[3]["drive"]["outcome"] == "canceled"
    assert next_actions(records) == {1: "set_nav_goal doorway"}  # the id read as the label the view showed

    text = timeline(records)
    assert "view #1 (rules) at (3.0, -1.5) 90° tilt 27° | Person 1.4 m ahead | 2 destinations | attention framed" in text
    assert "call set_nav_goal psg:3 (model)" in text and "-> set_nav_goal: canceled in 1.5 s (Stopped: a person came into view.)" in text
    assert "truth: robot (3.04, -1.52) 90° | nav following -> doorway | attention framed 1.4 m | person wander (4.4, -1.6) 1.36 m from robot" in text


def test_tools_a_server_lacks_are_skipped_after_the_first_refusal(tmp_path) -> None:
    trace = TraceRecorder(tmp_path / "real", url=None)
    server = FakeServer(missing=("get_attention_status", "sim_person_state"))
    asyncio.run(trace.sample_once(server))
    asyncio.run(trace.sample_once(server))
    assert server.calls.count("sim_person_state") == 1 and server.calls.count("get_nav_status") == 2
    trace.close()


def test_render_draws_the_storyboard_the_map_the_timeline_and_a_page(tmp_path) -> None:
    trace = TraceRecorder(tmp_path / "run", url=None, meta={"run_id": "r2"})
    server = FakeServer()
    for i in range(14):  # more than one sheet
        view = {**VIEW, "agent": {"x": 3.0 + 0.2 * i, "z": -1.5, "yaw": 90.0}}
        trace.event(call_event(f"v{i}", "get_current_view", {}), 1 + i // 5)
        trace.event(response_event(f"v{i}", "get_current_view", {"result": view, "isError": False}, image=jpeg((20 * i % 255, 120, 60))), 1 + i // 5)
        asyncio.run(trace.sample_once(server))
    trace.close()
    out = render(tmp_path / "run")
    assert [p.name for p in out["storyboards"]] == ["storyboard-01.png", "storyboard-02.png"]
    with Image.open(out["storyboards"][0]) as sheet:
        assert sheet.size == (1200, 3 * (300 + 4 * 15 + 6))  # four caption lines
    with Image.open(out["trajectory"]) as picture:
        assert picture.width >= 420 and picture.height > 100
    assert "storyboard-02.png" in out["html"].read_text() and "view #14" in out["timeline"].read_text()
    assert len(render(tmp_path / "run", last=3)["storyboards"]) == 1  # only the last views; old sheets removed


def test_attach_reads_the_portal_stream_and_records_from_its_first_snapshot(tmp_path) -> None:
    import httpx

    from gpt_pet.trace_attach import attach

    view_result = {"result": VIEW, "isError": False}
    frames = [
        ("adk", {"tick": 1, "author": "executor", "timestamp": 1.0, "functionCall": {"id": "old", "name": "do_rotate", "args": {}}}),  # replayed history
        ("status", {"profile": "sim", "paused": False}),
        ("adk", {"tick": 2, "author": "goal_setter", "timestamp": 2.0, "text": '{"previous_goal_status":"none","goal":"say hi"}'}),
        ("adk", {"tick": 2, "author": "executor", "timestamp": 3.0, "functionCall": {"id": "v1", "name": "get_current_view", "args": {}}, "s1": {"by": "rules", "action": "get_current_view"}}),
        ("adk", {"tick": 2, "author": "executor", "timestamp": 3.1, "functionResponse": {"id": "v1", "name": "get_current_view", "response": view_result}}),
        ("adk", {"tick": 2, "author": "executor", "timestamp": 3.2, "functionCall": {"id": "a1", "name": "set_attention", "args": {"destination_id": "obj:person_0"}}}),
        ("adk", {"tick": 2, "author": "executor", "timestamp": 4.0, "functionResponse": {"id": "a1", "name": "set_attention", "response": {"result": {"state": "framed"}, "isError": False}}}),
        ("notice", {"kind": "person", "tick": 2, "text": "A person came into view while resting: Person.", "timestamp": 5.0}),
        ("tick", {"number": 2, "seconds": 4.0, "tools": ["get_current_view", "set_attention"], "llm_calls": 1}),
    ]
    body = ": keep-alive\n\n" + "".join(f"id: {i}\nevent: {name}\ndata: {json.dumps(data)}\n\n" for i, (name, data) in enumerate(frames))

    def handler(request: httpx.Request) -> httpx.Response:
        if request.url.path == "/api/status":
            return httpx.Response(200, json={"profile": "sim", "mcp_url": None})
        if request.url.path == "/api/frame.jpg":
            return httpx.Response(200, content=jpeg((10, 200, 10)), headers={"content-type": "image/jpeg"})
        return httpx.Response(200, text=body, headers={"content-type": "text/event-stream"})

    real_client = httpx.AsyncClient
    try:
        httpx.AsyncClient = lambda **kwargs: real_client(transport=httpx.MockTransport(handler), **kwargs)  # type: ignore[misc]
        directory = asyncio.run(attach("http://pet.test", tmp_path))
    finally:
        httpx.AsyncClient = real_client  # type: ignore[misc]
    meta, records = load(directory)
    assert meta["source"] == "attach" and meta["portal"] == "http://pet.test"
    assert [r["type"] for r in records] == ["text", "view", "call", "result", "notice", "tick"]  # the replayed rotate is skipped
    view = records[1]
    assert view["by"] == "rules" and view["people"][0]["id"] == "obj:person_0" and (directory / view["frame"]).stat().st_size > 1000
    assert records[2]["tool"] == "set_attention" and records[3]["result"] == {"state": "framed"}
