"""Offline: does this person want to interact? (`[people] engage`, `gpt_pet.social`): reading cues,
the interest check over a window of looks, what the pet keeps and learns, and how the rest of the
brain (people memory, attention wait, System 1, goal setter) uses it."""
from __future__ import annotations

import asyncio
import json
from pathlib import Path
from types import SimpleNamespace

import pytest

from gpt_pet.agents.goal_setter import render_goal_setter_prompt
from gpt_pet.people import EXECUTOR, WATCH, People, make_attention_wait
from gpt_pet.s1 import LOOK, Step, people_turn
from gpt_pet.settings import PeopleSettings, load_settings
from gpt_pet.social import (
    INTERESTED,
    NOT_INTERESTED,
    CueReading,
    Engagement,
    SocialMemory,
    Track,
    engage,
    reading_cues,
)

EXPLORE = {"id": 3, "goal": "explore the hallway", "success_criteria": "a new room is in view", "status": "active", "kind": "explore"}
LIMITS = {"get_current_view": 4, "set_nav_goal": 2, "set_attention": 2, "do_rotate": 2}


class Clock:
    def __init__(self, t: float = 1000.0) -> None:
        self.t = t

    def __call__(self) -> float:
        return self.t


def reading(**fields) -> CueReading:
    base = {"person_visible": True, "facing": "toward_me", "posture": "standing", "gesture": "none", "busy_with": "", "looking_at_me": "no", "evidence": "a person"}
    return CueReading(**{**base, **fields})


def person(distance: float, bbox=(150, 20, 350, 499)) -> dict:
    return {"id": "obj:person_0", "label": "Person", "kind": "object", "description": f"Person, {distance} m ahead", "distance_m": distance, "bbox": list(bbox)}


def view(distance: float | None) -> dict:
    return {"agent": {"x": 0.0, "z": 0.0, "yaw": 0.0}, "destinations": [person(distance)] if distance is not None else []}


class Scripted:
    """Looks and readings by look number; the fake clock moves `step` per look (plus the sleeps)."""

    def __init__(self, distances: list, readings: list, clock: Clock) -> None:
        self.distances, self.readings, self.clock = distances, readings, clock
        self.n = 0
        self.timeout_s = 1.0

    async def look(self):
        k = min(self.n, len(self.distances) - 1)
        self.n += 1
        return view(self.distances[k]), b"jpeg", "image/jpeg"

    async def read(self, image, mime, context="", bbox=None):
        k = min(self.n - 1, len(self.readings) - 1)
        return self.readings[k]

    async def sleep(self, seconds: float) -> None:
        self.clock.t += seconds
        await asyncio.sleep(0)  # like a real sleep: the readings under way get to finish


def run(distances, readings, *, memory: SocialMemory | None = None, window_s: float = 10.0) -> Engagement:
    clock = Clock()
    script = Scripted(distances, readings, clock)
    memory = memory or SocialMemory(clock=clock)
    return asyncio.run(engage("obj:person_0", script.look, script, Track(), memory, window_s=window_s, look_every_s=2.0, clock=clock, sleep=script.sleep))


# --- cues ------------------------------------------------------------------------------------------


def test_a_reading_shows_the_cues_a_robot_can_see():
    assert reading_cues(reading(gesture="waving"), 1.5) == ["wave"]
    assert reading_cues(reading(gesture="waving", facing="away"), 1.5) == ["facing_away"]  # waving at someone else
    assert reading_cues(reading(posture="crouching"), 1.1) == ["crouch"]
    assert reading_cues(reading(looking_at_me="yes"), 1.6) == ["looks_at_me"]
    assert reading_cues(reading(looking_at_me="yes"), 4.0) == []  # too far to mean me
    assert reading_cues(reading(facing="away", busy_with="working at the table"), 2.0) == ["busy"]
    assert reading_cues(reading(facing="sideways", posture="walking"), 1.2) == ["passing"]
    assert reading_cues(reading(person_visible=False, gesture="waving"), 1.5) == []


# --- the interest check ------------------------------------------------------------------------------


def test_a_wave_decides_interest_at_once():
    result = run([2.0, 1.5], [reading(gesture="waving", looking_at_me="yes")])
    assert result.decision == INTERESTED and result.decided_by == "evidence" and result.looks == 1
    assert result.evidence[0] == "waved at me (2.0 m)" and result.words().startswith("waved at me (2.0 m)")


def test_crouching_down_to_the_pet_is_interest():
    result = run([1.4, 1.1], [reading(looking_at_me="no"), reading(posture="crouching", looking_at_me="yes")])
    assert result.decision == INTERESTED and "crouch" in result.cues


def test_someone_busy_at_a_table_is_not_interested_when_the_window_ends():
    result = run([2.4] * 8, [reading(facing="away", busy_with="working at the table")])
    assert result.decision == NOT_INTERESTED and result.decided_by == "window" and result.cues == ["busy"]
    assert result.watched_s == pytest.approx(10.0, abs=0.01)


def test_a_walk_by_is_not_interested_once_they_have_passed():
    distances = [2.6, 1.8, 1.2, 2.3, 3.6]
    readings = [reading(posture="walking", facing="toward_me"), reading(posture="walking", facing="sideways")]
    result = run(distances, readings)
    assert result.decision == NOT_INTERESTED and result.decided_by == "left" and {"passing", "leaving"} <= set(result.cues)
    assert result.watched_s < 10.0


def test_someone_who_stands_near_and_looks_at_the_pet_is_interested():
    result = run([1.6, 1.6, 1.55, 1.6], [reading(looking_at_me="yes")])
    assert result.decision == INTERESTED and {"looks_at_me", "stays_near"} <= set(result.cues)


def test_nobody_in_view_is_not_interested():
    result = run([None], [reading()])
    assert result.decision == NOT_INTERESTED and result.readings == 0 and result.evidence == []


def test_a_late_reading_still_counts():
    """A reading that arrives after the window still decides (the pet does not miss a wave)."""
    clock = Clock()

    class Slow(Scripted):
        async def read(self, image, mime, context="", bbox=None):
            await asyncio.sleep(0.01)
            return reading(gesture="waving")

    script = Slow([1.5], [], clock)
    result = asyncio.run(engage("obj:person_0", script.look, script, Track(), SocialMemory(clock=clock), window_s=0.5, look_every_s=2.0, clock=clock, sleep=script.sleep))
    assert result.decision == INTERESTED


# --- what the pet keeps ------------------------------------------------------------------------------


def engagement(decision: str, cues=("wave",)) -> Engagement:
    return Engagement("obj:person_0", decision, 3.0 if decision == INTERESTED else 0.0, list(cues), ["waved at me (1.5 m)"] if decision == INTERESTED else [], 4.0, 2, 2, "evidence")


def test_not_interested_leaves_them_be_doubling_each_time_and_interest_resets_it():
    clock = Clock()
    memory = SocialMemory(PeopleSettings(ignore_for_s=120, ignore_max_s=300), clock=clock)
    memory.record(engagement(NOT_INTERESTED, ()))
    assert memory.ignoring("obj:person_0") and memory.leave_be_s("obj:person_0") == pytest.approx(120)
    clock.t += 121
    assert memory.status("obj:person_0") == "unknown"
    memory.record(engagement(NOT_INTERESTED, ()))
    assert memory.leave_be_s("obj:person_0") == pytest.approx(240)
    clock.t += 241
    memory.record(engagement(NOT_INTERESTED, ()))
    assert memory.leave_be_s("obj:person_0") == pytest.approx(300)  # capped
    memory.reopen("obj:person_0", "came right up to me")
    assert memory.status("obj:person_0") == "unknown"
    memory.record(engagement(INTERESTED))
    assert memory.interested("obj:person_0") and "wants to interact" in memory.person_line("obj:person_0")


def test_cues_learn_from_how_decisions_turned_out_and_the_memory_is_kept(tmp_path: Path):
    clock = Clock()
    path = tmp_path / "social.json"
    memory = SocialMemory(clock=clock, path=path)
    assert memory.weight("looks_at_me") == pytest.approx(1.0)  # nothing learned yet
    for _ in range(3):
        memory.record(engagement(INTERESTED, ("looks_at_me",)))
        clock.t += 20
        memory.resolve(lambda pid, since, within: False, lambda pid, since: False)  # they walked off: wrong
    assert memory.cue_stats["looks_at_me"] == [0, 3] and memory.weight("looks_at_me") == pytest.approx(0.7)
    memory.record(engagement(NOT_INTERESTED, ("busy",)))
    clock.t += 61
    memory.resolve(lambda pid, since, within: False, lambda pid, since: False)  # they never came: right
    assert memory.cue_stats["busy"] == [1, 0] and memory.weight("busy") > 1.0
    assert any("looked at me" in line and "0 of 3" in line for line in memory.lessons())
    again = SocialMemory(clock=clock, path=path)  # the next session
    assert again.cue_stats == memory.cue_stats and again.persons["obj:person_0"].encounters == 4
    assert json.loads(path.read_text())["persons"]["obj:person_0"]["interested"] == 3


def test_a_cue_that_kept_misleading_counts_for_less_next_time():
    clock = Clock()
    memory = SocialMemory(clock=clock)
    memory.cue_stats["looks_at_me"] = [0, 8]  # standing and looking never meant anything here
    script = Scripted([1.6, 1.6, 1.55, 1.6], [reading(looking_at_me="yes")], clock)
    result = asyncio.run(engage("obj:person_0", script.look, script, Track(), memory, window_s=10.0, clock=clock, sleep=script.sleep))
    assert result.decision == NOT_INTERESTED  # 2 x 0.6 + stays_near 1.0 < 3


# --- the brain's use of it ------------------------------------------------------------------------------


def social_people(clock: Clock | None = None) -> People:
    clock = clock or Clock()
    return People(["Person"], react_every_s=60.0, clock=clock, social=SocialMemory(clock=clock))


def test_people_left_be_are_not_reacted_to_and_the_setter_hears_only_of_interest():
    clock = Clock()
    memory = social_people(clock)
    [seen] = memory.saw(view(2.0), EXECUTOR)
    assert memory.to_react(WATCH, EXPLORE, [seen]) is seen  # unknown: worth turning to
    assert memory.news_for_setter(EXPLORE) is None  # but not news until they want to interact
    memory.record_engagement(engagement(NOT_INTERESTED, ("busy",)))
    assert memory.to_react(WATCH, EXPLORE, [seen]) is None and memory.to_react(EXECUTOR, EXPLORE, [seen]) is None
    memory.social.reopen("obj:person_0", "test")
    memory.record_engagement(engagement(INTERESTED))
    assert memory.news_for_setter(EXPLORE) is not None and memory.social.persons["obj:person_0"].told
    assert memory.counts["interested"] == 1 and memory.counts["not_interested"] == 1


def test_someone_left_be_who_walks_right_up_to_the_pet_is_looked_at_again():
    clock = Clock()
    memory = social_people(clock)
    memory.saw(view(2.5), EXECUTOR)
    memory.record_engagement(engagement(NOT_INTERESTED, ()))
    for _ in range(4):
        clock.t += 1.0
        memory.saw(view(0.9), WATCH)
    assert not memory.leaving_be("obj:person_0") and memory.counts["reopened"] == 1


def test_social_notices_say_who_wants_to_interact_and_who_is_left_be():
    memory = social_people()
    notices = []
    memory.on_event = notices.append
    memory.saw(view(1.5), EXECUTOR)
    memory.record_engagement(engagement(INTERESTED))
    assert notices[-1]["kind"] == "social" and notices[-1]["text"].startswith("Human wants to interact with me! Person (obj:person_0) waved at me")
    memory.social.persons["obj:person_0"].status = "unknown"
    memory.record_engagement(engagement(NOT_INTERESTED, ("busy",)))
    assert notices[-1]["kind"] == "ignore" and "leaving them be for 2 min" in notices[-1]["text"]


class FakeTool:
    def __init__(self, name: str, answers) -> None:
        self.name, self.answers, self.calls = name, answers, 0

    async def run_async(self, args, tool_context):
        self.calls += 1
        answer = self.answers(self.calls) if callable(self.answers) else self.answers
        return answer


def mcp_result(payload: dict, image: bool = False) -> dict:
    content = [{"type": "text", "text": json.dumps(payload)}]
    if image:
        import base64

        content.append({"type": "image", "data": base64.b64encode(b"\xff\xd8jpeg").decode(), "mimeType": "image/jpeg"})
    return {"content": content, "isError": False}


@pytest.mark.parametrize("wave", [True, False])
def test_turning_to_someone_unknown_runs_the_interest_check(wave: bool):
    memory = social_people(Clock())
    memory.social.clock = __import__("time").time  # the check's own waits are real here (short window)
    memory.saw(view(1.5), EXECUTOR)
    notices = []
    memory.on_event = notices.append
    status = FakeTool("get_attention_status", mcp_result({"state": "framed", "target": "obj:person_0"}))
    looks = FakeTool("get_current_view", mcp_result(view(1.5), image=True))
    cancel = FakeTool("cancel_attention", mcp_result({"state": "off", "target": "obj:person_0", "reason": "canceled"}))
    toolset = SimpleNamespace(get_tools=lambda: asyncio.sleep(0, result=[status, looks, cancel]))

    class Reader:
        timeout_s = 1.0

        async def read(self, image, mime, context="", bbox=None):
            return reading(gesture="waving" if wave else "none", facing="toward_me" if wave else "away", busy_with="" if wave else "reading a book")

    settings = PeopleSettings(engage=True, engage_window_s=0.3, engage_look_every_s=0.1)
    wait = make_attention_wait(toolset, settings, memory, None, Reader())
    started = mcp_result({"state": "framed", "target": "obj:person_0"})
    context = SimpleNamespace(state={"current_goal": EXPLORE, "tick": 2})
    response = asyncio.run(wait(SimpleNamespace(name="set_attention"), {"destination_id": "obj:person_0"}, context, started))
    assert response["engagement"]["decision"] == (INTERESTED if wave else NOT_INTERESTED)
    assert looks.calls >= 1 and memory.counts["cue_reads"] >= 1
    if wave:
        assert cancel.calls == 0 and notices[-1]["kind"] == "social"
    else:
        assert cancel.calls == 1 and response["result"]["state"] == "off" and notices[-1]["kind"] == "ignore"
    # known now: turning to them again does not check again
    again = asyncio.run(wait(SimpleNamespace(name="set_attention"), {"destination_id": "obj:person_0"}, context, started))
    assert "engagement" not in again


def test_system_one_reports_interest_and_carries_on_past_disinterest():
    memory = social_people()
    memory.saw(view(1.5), EXECUTOR)
    look = Step(LOOK, {}, {"result": view(1.5), "isError": False})

    def attended(decision: str) -> Step:
        engagement_block = {"decision": decision, "evidence": ["waved at me (1.5 m)"] if decision == INTERESTED else []}
        return Step("set_attention", {"destination_id": "obj:person_0"}, {"result": {"state": "framed"}, "isError": False, "engagement": engagement_block})

    turn = people_turn(EXPLORE, [look, attended(INTERESTED)], memory, LIMITS)
    assert turn.kind == "report" and "They want to interact: waved at me (1.5 m)." in turn.text
    turn = people_turn(EXPLORE, [look, attended(NOT_INTERESTED)], memory, LIMITS)
    assert turn.kind == "call" and turn.tool == LOOK  # the goal goes on


def test_the_goal_setter_reads_who_wants_to_interact_and_what_the_pet_learned():
    settings = load_settings("sim")
    memory = social_people()
    memory.saw(view(1.5), EXECUTOR)
    memory.social.cue_stats["wave"] = [3, 0]
    memory.record_engagement(engagement(INTERESTED))
    prompt = " ".join(render_goal_setter_prompt({}, settings, None, memory).split())
    assert "wants to interact with you (waved at me (1.5 m))" in prompt
    assert '"target_id": "obj:person_0"' in prompt and "greet the person who wants to interact" in prompt
    assert '"waved at me" meant they wanted to interact 3 of 3 times' in prompt
    assert "or a person wants to interact with the pet" in prompt


def test_once_the_pet_is_off_its_interaction_goal_it_leaves_them_be_for_a_minute():
    clock = Clock()
    memory = social_people(clock)
    [seen] = memory.saw(view(1.5), EXECUTOR)
    memory.record_engagement(engagement(INTERESTED))
    assert memory.news_for_setter(EXPLORE) is not None  # told
    assert memory.to_react(EXECUTOR, EXPLORE, [seen]) is None  # back on another goal: the interaction is over
    assert memory.leaving_be("obj:person_0") and memory.social.persons["obj:person_0"].streak == 0
    clock.t += 61
    assert memory.social.status("obj:person_0") == "unknown"


def test_the_pet_waits_for_someone_walking_up_to_it_before_deciding():
    """Suite step 3, GPT-4.1 mini, wave scenario: the check began with the person 5 m away and
    walking over; its 10 s ran out as they arrived and waved. Time while they are still coming
    closer no longer counts."""
    distances = [5.1, 4.4, 3.6, 2.7, 2.0, 1.45, 1.4, 1.4, 1.4]
    readings = [reading(posture="walking", looking_at_me="no")] * 6 + [reading(gesture="waving", looking_at_me="yes")] * 3
    result = run(distances, readings)
    assert result.decision == INTERESTED and "wave" in result.cues, (result.evidence, result.watched_s)
    assert "passing" not in result.cues  # walking at an angle on the way over is not walking past
    assert result.watched_s > 10.0


def test_a_walk_up_never_holds_the_check_beyond_its_cap():
    result = run([20.0 - 0.5 * k for k in range(40)], [reading(posture="walking")])
    assert result.decision == NOT_INTERESTED and result.watched_s <= 25.0 + 0.01


def test_someone_left_be_is_looked_at_again_only_if_they_come_closer_than_before():
    clock = Clock()
    memory = social_people(clock)
    memory.saw(view(1.6), EXECUTOR)
    decided = engagement(NOT_INTERESTED, ("busy",))
    decided.details = [{"distance_m": 1.6}]
    memory.record_engagement(decided)
    for _ in range(4):  # busy right beside the pet, where they were: still left be
        clock.t += 1.0
        memory.saw(view(1.4), WATCH)
    assert memory.leaving_be("obj:person_0")
    for _ in range(3):  # they come right up to it
        clock.t += 1.0
        memory.saw(view(0.9), WATCH)
    assert not memory.leaving_be("obj:person_0") and memory.counts["reopened"] == 1


def test_one_look_reported_twice_is_one_point_on_the_track():
    """Suite step 3b: the people memory and the check both recorded every look, so the last two
    points were always the same distance and nobody ever counted as still coming over."""
    track = Track()
    for t, d in ((0.0, 5.0), (2.0, 4.1), (4.0, 3.2)):
        track.add(t, d, None)
        track.add(t + 0.01, d, None)  # the same look again
    assert [p[1] for p in track.points] == [5.0, 4.1, 3.2] and track.approaching()


def test_the_check_waits_for_someone_walking_up_even_when_each_look_is_recorded_twice():
    clock = Clock()
    memory = social_people(clock)

    distances = [5.1, 4.3, 3.4, 2.5, 1.8, 1.3, 1.3, 1.3]
    readings = [reading(posture="walking", looking_at_me="yes")] * 6 + [reading(gesture="waving", looking_at_me="yes")] * 2
    script = Scripted(distances, readings, clock)

    async def look():
        payload, image, mime = await script.look()
        memory.saw(payload, "engage")  # what check_interest's look does
        return payload, image, mime

    track = memory.tracks.setdefault("obj:person_0", Track())
    result = asyncio.run(engage("obj:person_0", look, script, track, memory.social, window_s=10.0, clock=clock, sleep=script.sleep))
    assert result.decision == INTERESTED and "wave" in result.cues, (result.evidence, result.watched_s)


def test_someone_left_be_who_waves_nearby_is_noticed_after_all():
    clock = Clock()
    memory = social_people(clock)
    notices = []
    memory.on_event = notices.append
    memory.saw(view(2.0), EXECUTOR)
    memory.record_engagement(engagement(NOT_INTERESTED, ("busy",)))
    reads = []

    class Reader:
        timeout_s = 1.0

        async def read(self, image, mime, context="", bbox=None):
            reads.append(context)
            return reading(gesture="waving" if len(reads) > 1 else "none")

    memory.reader = Reader()
    assert asyncio.run(memory.glance(view(2.0), b"jpeg", "image/jpeg")) is None and reads == []  # just decided: no glance yet
    clock.t += 16
    assert asyncio.run(memory.glance(view(3.5), b"jpeg", "image/jpeg")) is None and reads == []  # too far to glance at
    assert asyncio.run(memory.glance({**view(1.2), "camera": {"tilt_deg": 0.0}}, b"jpeg", "image/jpeg")) is None and reads == []  # legs only
    assert asyncio.run(memory.glance(view(2.0), b"jpeg", "image/jpeg")) is None and len(reads) == 1  # nothing
    clock.t += 5
    assert asyncio.run(memory.glance(view(2.0), b"jpeg", "image/jpeg")) is None and len(reads) == 1  # not again so soon
    clock.t += 15
    assert asyncio.run(memory.glance(view(2.0), b"jpeg", "image/jpeg")) == "obj:person_0"  # a wave
    assert not memory.leaving_be("obj:person_0") and memory.counts["glances"] == 2
    assert notices[-1]["kind"] == "person" and "waved at me (2.0 m) while I was leaving them be" in notices[-1]["text"]
    [seen] = memory.saw(view(2.0), EXECUTOR)
    assert memory.to_react(EXECUTOR, EXPLORE, [seen]) is seen  # turned to again (and checked)


def test_the_executor_callback_records_views_and_glances_at_people_left_be():
    """The executor's after-tool callback with people and the cue reader on (it once crashed on
    every call: a local name shadowed the media callback)."""
    from gpt_pet.agents.executor import make_after_tool_callback
    from gpt_pet.settings import NavSettings

    clock = Clock()
    memory = social_people(clock)
    memory.saw(view(2.0), EXECUTOR)
    memory.record_engagement(engagement(NOT_INTERESTED, ("busy",)))
    reads = []

    class Reader:
        timeout_s = 1.0

        async def read(self, image, mime, context="", bbox=None):
            reads.append(image)
            return reading(gesture="waving")

    memory.reader = Reader()
    clock.t += 16  # past the time right after the decision when the pet does not glance
    toolset = SimpleNamespace(get_tools=lambda: asyncio.sleep(0, result=[]))
    after = make_after_tool_callback(toolset, NavSettings(), people=memory, people_settings=PeopleSettings(engage=True), reader=Reader())
    context = SimpleNamespace(state={"tick": 3, "current_goal": EXPLORE}, invocation_id="i1")
    response = asyncio.run(after(SimpleNamespace(name="get_current_view"), {}, context, mcp_result(view(1.8), image=True)))
    assert response["result"]["destinations"][0]["id"] == "obj:person_0" and response["media"]
    assert reads and not memory.leaving_be("obj:person_0")  # glanced at, waving: looked at again
    other = asyncio.run(after(SimpleNamespace(name="do_rotate"), {}, context, mcp_result({"ok": True})))
    assert other["result"] == {"ok": True}


def test_a_crop_keeps_the_person_and_a_margin_and_enlarges_it():
    import io

    from PIL import Image

    from gpt_pet.social import CROP_PX, crop_to_person

    frame = Image.new("RGB", (500, 500), (200, 200, 200))
    frame.paste((255, 0, 0), (200, 100, 300, 400))  # the person
    buffer = io.BytesIO()
    frame.save(buffer, format="JPEG")
    cropped, mime = crop_to_person(buffer.getvalue(), (200, 100, 300, 400))
    with Image.open(io.BytesIO(cropped)) as cut:
        assert mime == "image/jpeg" and max(cut.size) == CROP_PX and cut.size[0] == cut.size[1]
        assert cut.getpixel((CROP_PX // 2, CROP_PX // 2))[0] > 200  # the person in the middle
    same, _ = crop_to_person(b"not an image", (0, 0, 10, 10))
    assert same == b"not an image"


def test_a_strong_cue_read_very_close_needs_a_second_look():
    """Suite step 3c, GPT-4.1 mini: someone standing 0.7 m from the pet read as "crouching" once and
    was taken for interested. From that close a strong cue counts only when another look agrees."""
    once = run([0.7, 0.7, 0.7, 0.7, 0.7], [reading(facing="away"), reading(posture="crouching")] + [reading(facing="away")] * 3)
    assert once.decision == NOT_INTERESTED and "crouch" not in once.cues
    twice = run([0.9, 0.9, 0.9], [reading(posture="crouching")])
    assert twice.decision == INTERESTED and "crouch" in twice.cues
    far = run([1.2, 1.2], [reading(posture="crouching")])
    assert far.decision == INTERESTED and far.looks == 1  # from 1.2 m one look is enough


def test_someone_whose_interest_is_still_unknown_is_turned_to_again_soon():
    clock = Clock()
    memory = social_people(clock)
    [seen] = memory.saw(view(1.0), EXECUTOR)
    memory.react(EXECUTOR, seen.id)  # turned to them; they were lost from view: no check, nothing decided
    clock.t += 10
    assert memory.to_react(EXECUTOR, EXPLORE, [seen]) is None
    clock.t += 6
    assert memory.to_react(EXECUTOR, EXPLORE, [seen]) is seen  # 16 s later: again


def test_a_cue_model_of_its_own_does_not_inherit_the_executors_reasoning_effort():
    """Suite step 3d: a GPT-6 Luna brain (reasoning_effort none) with GPT-4.1 mini reading cues sent
    reasoning_effort to GPT-4.1 mini, which refused every call."""
    from gpt_pet.brain import cue_reader_model

    settings = load_settings("sim")
    luna = settings.model_copy(update={"model": settings.model.model_copy(update={"executor": "openai/gpt-6-luna", "reasoning_effort": "none"})})
    own = luna.model_copy(update={"people": luna.people.model_copy(update={"cue_model": "openai/gpt-4.1-mini"})})
    name, model = cue_reader_model(own)
    assert name == "openai/gpt-4.1-mini" and model.reasoning_effort == "" and model.image_detail == "low"
    name, model = cue_reader_model(luna)
    assert name == "openai/gpt-6-luna" and model.reasoning_effort == "none"


def test_someone_left_be_who_stays_right_beside_the_pet_gets_one_more_look():
    """Suite step 3d, GPT-4.1 mini: the pet left a wandering person be as they paused 0.7 m away;
    40 s later the same person crouched 0.6 m in front of it for half a minute, out of the level
    camera's frame. Staying right beside the pet earns one more look per decision."""
    clock = Clock()
    memory = social_people(clock)
    memory.saw(view(0.7), EXECUTOR)
    decided = engagement(NOT_INTERESTED, ("facing_away",))
    decided.details = [{"distance_m": 0.7}]
    memory.record_engagement(decided)
    for _ in range(5):  # still right there, but too soon after the decision
        clock.t += 1.0
        memory.saw(view(0.7), WATCH)
    assert memory.leaving_be("obj:person_0")
    clock.t += 20
    for _ in range(4):
        clock.t += 1.0
        memory.saw(view(0.6), WATCH)
    assert not memory.leaving_be("obj:person_0") and memory.social.persons["obj:person_0"].lingered
    again = engagement(NOT_INTERESTED, ("facing_away",))
    again.details = [{"distance_m": 0.6}]
    memory.record_engagement(again)  # not interested again: a new decision, a new one-time look
    assert not memory.social.persons["obj:person_0"].lingered
