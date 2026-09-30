"""Offline: the server's pure pieces on real ADK event objects (no HTTP, no LLM)."""
from __future__ import annotations

import asyncio
import json

from google.adk.events.event import Event
from google.genai import types

from gpt_pet.server import Broadcaster, event_payloads


def model_event(*parts: types.Part, author: str = "executor") -> Event:
    return Event(author=author, invocation_id="inv-1", content=types.Content(role="model", parts=list(parts)))


def test_event_payloads_split_calls_responses_and_text() -> None:
    call = model_event(types.Part(function_call=types.FunctionCall(id="fc-1", name="get_map", args={})))
    response = model_event(
        types.Part(function_response=types.FunctionResponse(id="fc-1", name="get_map", response={"result": [], "isError": False}))
    )
    text = model_event(types.Part(text="Reached the fridge."))
    empty = Event(author="update_goal_memory", invocation_id="inv-1")

    [call_payload] = event_payloads(call, tick=3)
    assert call_payload["functionCall"] == {"id": "fc-1", "name": "get_map", "args": {}}
    assert call_payload["tick"] == 3 and call_payload["author"] == "executor"
    [response_payload] = event_payloads(response, tick=3)
    assert response_payload["functionResponse"]["response"] == {"result": [], "isError": False}
    [text_payload] = event_payloads(text, tick=3)
    assert text_payload["text"] == "Reached the fridge."
    assert event_payloads(empty, tick=3) == []
    json.dumps(call_payload)  # serialisable


def test_snapshot_frames_reach_subscribers_but_skip_the_replay_buffer() -> None:
    async def scenario() -> None:
        broadcaster = Broadcaster()
        queue = broadcaster.subscribe()
        broadcaster.publish("notice", {"kind": "rest"})
        broadcaster.publish("stats", {"uptime_s": 1.0}, keep=False)
        assert queue.qsize() == 2
        assert [frame.event for frame in broadcaster.replay(after_id=0)] == ["notice"]
        assert broadcaster.last_id == 2  # ids stay monotonic across both kinds

    asyncio.run(scenario())


def test_broadcaster_fans_out_and_replays_after_an_id() -> None:
    async def scenario() -> None:
        broadcaster = Broadcaster(history=3)
        queue = broadcaster.subscribe()
        first = broadcaster.publish("status", {"paused": False})
        broadcaster.publish("tick", {"number": 1})
        third = broadcaster.publish("image", {"name": "camera", "version": 1})
        assert (first.id, third.id) == (1, 3)
        assert broadcaster.last_id == 3
        assert queue.qsize() == 3
        frame = await queue.get()
        assert frame.as_dict() == {"id": "1", "event": "status", "data": json.dumps({"paused": False})}
        assert [f.id for f in broadcaster.replay(after_id=1)] == [2, 3]
        assert broadcaster.replay(after_id=None) == []
        assert [f.id for f in broadcaster.replay(after_id=0)] == [1, 2, 3]  # fresh page: full history
        broadcaster.publish("tick", {"number": 2})  # history capped at 3
        assert [f.id for f in broadcaster.replay(after_id=0)] == [2, 3, 4]
        before = queue.qsize()
        broadcaster.unsubscribe(queue)
        broadcaster.publish("tick", {"number": 3})
        assert queue.qsize() == before  # unsubscribed queues receive nothing more

    asyncio.run(scenario())
