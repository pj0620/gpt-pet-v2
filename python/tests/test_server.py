"""Offline: the server's pure pieces on real ADK event objects (no HTTP, no LLM)."""
from __future__ import annotations

import asyncio
import json

from google.adk.events.event import Event
from google.genai import types

from gpt_pet.server import AttentionWatcher, Broadcaster, attention_notice, event_payloads


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
    assert "s1" not in call_payload and "fallback" not in text_payload


def test_turns_s1_or_a_fallback_took_carry_their_tag() -> None:
    s1 = Event(
        author="executor",
        invocation_id="inv-1",
        content=types.Content(role="model", parts=[types.Part(function_call=types.FunctionCall(name="get_current_view", args={}))]),
        custom_metadata={"s1": {"by": "rules", "action": "get_current_view"}},
    )
    fallback = Event(
        author="executor",
        invocation_id="inv-1",
        content=types.Content(role="model", parts=[types.Part(text="Gemini did not answer (TimeoutError).")]),
        custom_metadata={"fallback": {"agent": "executor", "error": "TimeoutError"}},
    )
    assert event_payloads(s1, tick=1)[0]["s1"] == {"by": "rules", "action": "get_current_view"}
    assert event_payloads(fallback, tick=1)[0]["fallback"] == {"agent": "executor", "error": "TimeoutError"}


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


def attention(state: str, **fields) -> dict:
    base = {
        "state": state, "target": "obj:person_0", "label": "Person", "distance_m": 1.4, "bearing_deg": -3.0,
        "seen_s": 2.0, "unseen_s": 0.0, "moving": False, "tilt_deg": 27.0, "standoff_m": 1.3, "since_s": 4.0,
        "target_xz": [1.0, 2.0], "reason": None,
    }
    return {**base, **fields}


def test_attention_notices_explain_each_state_change() -> None:
    assert attention_notice(None, {"state": "off"}, 0, 1.0) is None
    assert attention_notice(attention("framed"), attention("framed", distance_m=1.2), 1, 1.0) is None
    assert attention_notice(None, attention("aiming", distance_m=3.0, bearing_deg=40.0), 2, 7.5) == {
        "kind": "attention", "state": "aiming", "tick": 2, "timestamp": 7.5,
        "text": "Attention: turning toward Person (3.0 m, 40° right)",
    }
    assert attention_notice(attention("framed"), attention("aiming"), 2, 8.0)["text"] == "Attention: re-aiming at Person (1.4 m, 3° left)"
    assert attention_notice(attention("aiming"), attention("approaching", distance_m=3.1, bearing_deg=0.4), 2, 9.0)["text"] == (
        "Attention: walking up to Person (3.1 m, straight ahead), stopping 1.3 m away"
    )
    assert attention_notice(attention("aiming"), attention("framed"), 2, 9.5)["text"] == "Attention: Person framed (1.4 m, 3° left), camera up 27°"
    assert attention_notice(attention("aiming"), attention("framed", tilt_deg=0.0, distance_m=3.0), 2, 9.5)["text"].endswith("camera level")
    assert attention_notice(attention("framed"), attention("searching", unseen_s=1.0), 3, 10.0)["text"] == (
        "Attention: lost sight of Person; looking where they were"
    )
    assert attention_notice(attention("searching"), attention("lost", unseen_s=5.2, reason="out of sight for 5 s"), 3, 15.0)["text"] == (
        "Attention ended: Person out of sight for 5 s"
    )
    assert attention_notice(attention("framed"), attention("off", reason="a drive started"), 4, 16.0)["text"] == "Attention ended: a drive started"


def test_the_attention_watcher_publishes_changes_and_notices_only_on_state_changes() -> None:
    frames: list[tuple[str, dict, bool]] = []

    def publish(event: str, data: dict, *, keep: bool = True) -> None:
        frames.append((event, data, keep))

    watcher = AttentionWatcher(publish)
    assert watcher.ingest({"state": "off"}, 0, 1.0) is True
    assert watcher.ingest({"state": "off"}, 0, 2.0) is False  # unchanged: nothing sent
    assert frames == [("attention", {"state": "off"}, False)] and not watcher.active
    aiming = attention("aiming", bearing_deg=30.0)
    assert watcher.ingest(aiming, 1, 3.0) is True and watcher.active
    assert [f[0] for f in frames] == ["attention", "attention", "notice"]
    assert frames[-1][1]["text"].startswith("Attention: turning toward Person") and frames[-1][2] is True
    assert watcher.ingest(attention("aiming", bearing_deg=5.0), 1, 3.5) is True
    assert [f[0] for f in frames] == ["attention", "attention", "notice", "attention"]  # same state: no notice
    watcher.ingest(attention("framed"), 1, 4.0)
    assert frames[-1][0] == "notice" and "framed" in frames[-1][1]["text"]
    assert watcher.last == attention("framed")
