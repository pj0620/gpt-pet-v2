"""Does this person want to interact with the pet? (`[people] engage`; gptpet-plans: People
Step 3 Interest and Body Language 10_7_2026.md).

When a person whose interest the pet does not know yet comes into view, the robot attends to them
(it turns, tilts its camera, walks up to 1.3 m if they stand still) and watches for up to
`engage_window_s`, looking every `engage_look_every_s`. It decides from what a robot with a camera
and depth can perceive, never from the simulator's own state:

- each look's frame goes to the cue reader, a small vision call that reports the person's facing,
  posture, gesture, what they are busy with and whether they look at the pet (on the robot a pose
  detector can stand in for it);
- the person's track (distance and bearing over the looks) gives approach, stop near, pass by and
  leave.

Cues add up to a score (CUE_WEIGHTS, each scaled by what the pet has learned about it): a wave,
beckon, clap or reach toward the pet, or crouching down facing it, decides at once; looking at the
pet while standing near it, or coming over and staying, counts less; being busy with something,
walking past or leaving count against. Interested at INTERESTED_SCORE; not interested when the
window ends below it, or earlier when the evidence is clearly against or the person has left.

`SocialMemory` keeps what came of it, per person and per cue: who to leave be and until when
(doubling for each disinterest in a row, reset by an interaction), and how often each cue was
followed by a real interaction. A decision is checked afterwards from what the robot sees: an
"interested" holds if the person is still near the pet CONFIRM_AFTER_S later; a "not interested"
holds if they do not come right up to it within REFUTE_WITHIN_S. With `[people] memory_file`, the
memory is kept across sessions.
"""
from __future__ import annotations

import asyncio
import json
import logging
import time
from collections import deque
from collections.abc import Awaitable, Callable, Iterable, Mapping
from dataclasses import asdict, dataclass, field
from pathlib import Path
from typing import TYPE_CHECKING, Any, Literal

from google.genai import types
from pydantic import BaseModel, Field

if TYPE_CHECKING:
    from gpt_pet.settings import ModelSettings, PeopleSettings
    from gpt_pet.stats import RunStats

log = logging.getLogger("gpt_pet.social")

INTERESTED, NOT_INTERESTED = "interested", "not_interested"
INTERESTED_SCORE = 3.0
NOT_INTERESTED_SCORE = -3.0
EARLY_NO_AFTER_S = 4.0
"""Clear evidence against decides "not interested" no sooner than this into the window."""
LOST_AFTER_S = 4.0
"""Out of view this long (and not coming closer): they have left."""
NEAR_M = 2.0
"""Standing within this of the pet counts as near it."""
LOOK_AT_ME_MAX = 2
"""Looks at the pet count at most this many times in one window."""
CUE_WEIGHTS = {
    # strong: decides on its own
    "wave": 3.0,
    "beckon": 3.0,
    "clap": 3.0,
    "reach": 2.0,  # the least reliable: someone working at a table or pointing reads as reaching too
    "crouch": 3.0,
    # mild
    "points": 1.0,
    "looks_at_me": 1.0,
    "approached": 1.0,
    "stays_near": 1.0,
    # against
    "busy": -2.0,
    "leaving": -2.0,
    "passing": -1.0,
    "facing_away": -1.0,
}
CUE_WORDS = {
    "wave": "waved at me",
    "beckon": "beckoned me over",
    "clap": "clapped",
    "reach": "reached out to me",
    "crouch": "crouched down to my level",
    "points": "pointed",
    "looks_at_me": "looked at me",
    "approached": "came over to me",
    "stays_near": "stayed near me, facing me",
    "busy": "was busy with something",
    "leaving": "walked off",
    "passing": "walked past",
    "facing_away": "faced away",
}
GESTURE_CUES = {"waving": "wave", "beckoning": "beckon", "clapping": "clap", "reaching_toward_me": "reach", "pointing": "points"}
WEIGHT_RANGE = (0.5, 1.5)
"""A learned cue weight scales its base weight by this much at most."""
CONFIRM_AFTER_S = 15.0
"""An "interested" holds if the person is still within CONFIRM_NEAR_M this long after."""
CONFIRM_NEAR_M = 2.5
REFUTE_WITHIN_S = 60.0
"""A "not interested" is wrong if the person comes within REOPEN_M and stays REOPEN_STAY_S within this long."""
REOPEN_M = 1.5
REOPEN_STAY_S = 2.0
LINGER_M = 1.3
LINGER_AFTER_S = 20.0
"""Someone left be who stays within LINGER_M of the pet (two looks REOPEN_STAY_S apart) at least this
long after the decision gets one more look (once per decision): a crouch right beside a 30 cm robot
is out of a level camera's frame, so no glance can catch it."""
REOPEN_CLOSER_M = 0.5
"""...and only when they came at least this much closer than when the pet decided (someone busy right
beside the pet does not keep reopening their own case)."""
APPROACH_STEP_M = 0.25
"""A look this much closer than the one before: they are still coming. That time does not count
toward the window (the pet waits for them to arrive), up to MAX_WINDOW_S in all."""
MAX_WINDOW_S = 25.0
SAME_LOOK_S = 0.3
CLOSE_CONFIRM_M = 1.0
"""A strong cue read from closer than this counts only when a second look shows a strong cue too:
from a 30 cm camera tilted up, someone standing that close can read as crouching or reaching."""
STRONG_CUES = frozenset({"wave", "beckon", "clap", "reach", "crouch"})
REST_AFTER_INTERACTION_S = 30.0
"""After an interaction the pet leaves the person be this long (they may well be saying goodbye)."""
INTEREST_FRESH_S = 90.0
"""A "wants to interact" stays news this long (the pet sets interaction goals for them meanwhile)."""

CUE_INSTRUCTION = """You read single frames from the camera of a small robot pet, about 30 cm tall, that looks
up at people. Report what the person in the frame does, only from what the image shows. Answer
"unclear" rather than guess. The frame may carry boxes, numbers and a caption: ignore them.

- facing: toward_me (their chest or face toward the camera), sideways, away (their back), unclear.
- posture: standing, walking (mid-stride), crouching (squatting or kneeling down low), sitting, unclear.
- gesture: waving (an arm raised: the hand at or above head height, or the arm lifted out to the
  side and up; from this low camera a wave looks like a raised arm), beckoning (an arm held out in
  front, the forearm bent up toward them: "come here"), pointing (an arm straight out, low),
  clapping (both hands together, in front or overhead), reaching_toward_me (an arm stretched down
  toward the camera), none (arms down by their sides or holding something), unclear.
- busy_with: what they are doing with an object or furniture (e.g. "working at the table",
  "watching TV", "sitting on the sofa"), or "" if nothing.
- looking_at_me: yes if their face is turned to the camera, no, or unclear.
- evidence: one short sentence on what you see."""


class CueReading(BaseModel):
    """What one frame shows about a person (the cue reader's answer)."""

    person_visible: bool = Field(description="Is a person visible in the frame?")
    facing: Literal["toward_me", "sideways", "away", "unclear"]
    posture: Literal["standing", "walking", "crouching", "sitting", "unclear"]
    gesture: Literal["waving", "beckoning", "pointing", "clapping", "reaching_toward_me", "none", "unclear"]
    busy_with: str = Field(description='What they are doing with an object, or "" if nothing.')
    looking_at_me: Literal["yes", "no", "unclear"]
    evidence: str = Field(description="One short sentence on what the image shows.")


# --- reading cues ---------------------------------------------------------------------------------


class CueReader:
    """The cue reader: one frame and a line of context in, a CueReading out (one vision call to the
    configured model; `[people] cue_model`, by default the executor's)."""

    def __init__(
        self,
        model_name: str,
        model_settings: "ModelSettings",
        stats: "RunStats | None" = None,
        timeout_s: float = 12.0,
        crop: bool = True,
    ) -> None:
        self.model_name = model_name
        self.model_settings = model_settings
        self.stats = stats
        self.timeout_s = timeout_s
        self.crop = crop
        """Read the person's box (with a margin, enlarged), not the whole frame: what a detector
        pipeline hands its classifier, and the model looks at the person rather than the room."""
        self._llm: Any = None

    def _model(self) -> Any:
        if self._llm is None:
            from gpt_pet.agents.models import agent_model

            llm = agent_model(self.model_name, self.model_settings)
            if isinstance(llm, str):
                from google.adk.models.registry import LLMRegistry

                llm = LLMRegistry.new_llm(llm)
            self._llm = llm
        return self._llm

    async def read(self, image: bytes, mime_type: str, context: str = "", bbox: Any = None) -> CueReading | None:
        """None when the call fails or times out (a missed reading is not worth stopping for).
        `bbox` (x0, y0, x1, y1 in the frame's pixels): the person's box, read cropped when `crop`."""
        from google.adk.models.llm_request import LlmRequest

        if self.crop and bbox is not None:
            image, mime_type = crop_to_person(image, bbox)

        config = types.GenerateContentConfig(
            system_instruction=CUE_INSTRUCTION,
            response_schema=CueReading,
            response_mime_type="application/json",
            temperature=0.0,
        )
        parts = [types.Part(text=f"Context: {context}" if context else "What does the person do?"), types.Part.from_bytes(data=image, mime_type=mime_type or "image/jpeg")]
        request = LlmRequest(model=self.model_name, contents=[types.Content(role="user", parts=parts)], config=config)
        started = time.monotonic()
        try:
            response = await asyncio.wait_for(self._first(self._model().generate_content_async(request, stream=False)), timeout=self.timeout_s)
        except Exception as exc:  # noqa: BLE001 - the look goes on without this reading
            log.warning("cue reader failed: %s", exc)
            if self.stats is not None:
                self.stats.record_llm_error(time.monotonic() - started)
            return None
        if self.stats is not None:
            self.stats.record_llm_call("cue_reader", time.monotonic() - started, getattr(response, "usage_metadata", None), model=self.model_name)
        text = "".join(part.text or "" for part in ((response.content.parts if response.content else None) or []) if getattr(part, "text", None))
        try:
            return CueReading.model_validate_json(_json_part(text))
        except Exception as exc:  # noqa: BLE001
            log.warning("cue reader answered something else: %s (%s)", text[:200], exc)
            return None

    @staticmethod
    async def _first(stream: Any) -> Any:
        async for response in stream:
            return response
        raise RuntimeError("no response")


CROP_MARGIN = 0.25
"""A crop keeps this much of the box's size around it on every side (arms out, a raised hand)."""
CROP_PX = 448


def crop_to_person(image: bytes, bbox: Any) -> tuple[bytes, str]:
    """The frame cut to the person's box plus CROP_MARGIN, enlarged to CROP_PX on its long side, as
    JPEG; the frame unchanged when it cannot be read or the box is degenerate."""
    import io

    try:
        from PIL import Image

        x0, y0, x1, y1 = (float(v) for v in bbox)
        with Image.open(io.BytesIO(image)) as frame:
            frame = frame.convert("RGB")
            w, h = frame.size
            bw, bh = x1 - x0, y1 - y0
            if bw < 4 or bh < 4:
                return image, "image/jpeg"
            side = max(bw, bh) * (1 + 2 * CROP_MARGIN)  # a square, so a raised arm beside the body stays in
            cx, cy = (x0 + x1) / 2, (y0 + y1) / 2
            box = (max(0, int(cx - side / 2)), max(0, int(cy - side / 2)), min(w, int(cx + side / 2)), min(h, int(cy + side / 2)))
            cut = frame.crop(box)
            scale = CROP_PX / max(cut.size)
            if abs(scale - 1) > 0.002:  # every crop the same size for the model
                cut = cut.resize((max(1, int(cut.size[0] * scale)), max(1, int(cut.size[1] * scale))), Image.LANCZOS)
            out = io.BytesIO()
            cut.save(out, format="JPEG", quality=90)
            return out.getvalue(), "image/jpeg"
    except Exception as exc:  # noqa: BLE001 - read the whole frame instead
        log.debug("crop failed: %s", exc)
        return image, "image/jpeg"


def _json_part(text: str) -> str:
    start, end = text.find("{"), text.rfind("}")
    return text[start : end + 1] if start >= 0 and end > start else text


# --- the person's track ---------------------------------------------------------------------------


@dataclass
class Track:
    """Where a person was relative to the robot, look by look: (time, distance, bearing)."""

    points: deque = field(default_factory=lambda: deque(maxlen=200))

    def add(self, t: float, distance_m: float | None, bearing_deg: float | None) -> None:
        """One look's distance; a second report of the same look (within SAME_LOOK_S: the people
        memory and the interest check both record each look) replaces the first."""
        if distance_m is None:
            return
        if self.points and abs(t - self.points[-1][0]) < SAME_LOOK_S:
            self.points[-1] = (t, float(distance_m), bearing_deg)
        else:
            self.points.append((t, float(distance_m), bearing_deg))

    def since(self, t0: float) -> list[tuple[float, float, Any]]:
        return [p for p in self.points if p[0] >= t0]

    def last_seen(self) -> float | None:
        return self.points[-1][0] if self.points else None

    def nearest(self, t0: float) -> float | None:
        recent = self.since(t0)
        return min((p[1] for p in recent), default=None)

    def approaching(self) -> bool:
        """The latest look was APPROACH_STEP_M closer than the one before."""
        return len(self.points) >= 2 and self.points[-2][1] - self.points[-1][1] >= APPROACH_STEP_M

    def receding_after_closest(self, t0: float) -> bool:
        """Since t0 they came closest earlier and are 0.3 m farther now (they went past)."""
        window = self.since(t0)
        if len(window) < 2:
            return False
        nearest = min(p[1] for p in window)
        return window[-1][1] - nearest >= 0.3 and window[-1][1] > window[-2][1]

    def stayed_within(self, near_m: float, for_s: float, now: float) -> bool:
        """Every look of the last for_s saw them within near_m, and those looks span most of it."""
        recent = [p for p in self.points if now - p[0] <= for_s]
        if len(recent) < 2 or recent[-1][0] - recent[0][0] < for_s * 0.8:
            return False
        return all(p[1] <= near_m for p in recent)

    def features(self, t0: float, now: float) -> dict[str, bool]:
        """approached (came 0.8 m closer and is now near), stopped_near (near and moved under 0.3 m
        over the last 3 s), leaving (1 m farther than their nearest, or out of view LOST_AFTER_S
        while not near)."""
        window = self.since(t0)
        out = {"approached": False, "stopped_near": False, "leaving": False}
        if not window:
            return out
        first, last = window[0][1], window[-1][1]
        nearest = min(p[1] for p in window)
        last_seen = window[-1][0]
        recent = [p for p in window if now - p[0] <= 4.5]  # looks come every 2 s
        stopped = len(recent) >= 2 and max(p[1] for p in recent) - min(p[1] for p in recent) < 0.3 and recent[-1][0] - recent[0][0] >= 2.5
        out["stopped_near"] = stopped and last <= NEAR_M
        out["approached"] = first - nearest >= 0.8 and last <= NEAR_M + 0.2
        out["leaving"] = last - nearest >= 1.0 or (now - last_seen >= LOST_AFTER_S and last > NEAR_M)
        return out


# --- one engagement -------------------------------------------------------------------------------


@dataclass
class Look:
    """One look of an engagement window."""

    t: float
    distance_m: float | None
    bearing_deg: float | None
    visible: bool
    reading: dict[str, Any] | None = None
    cues: list[str] = field(default_factory=list)


@dataclass
class Engagement:
    """The outcome of watching one person for a while."""

    person_id: str
    decision: str
    score: float
    cues: list[str]
    evidence: list[str]
    watched_s: float
    looks: int
    readings: int
    decided_by: str
    """"evidence" (the score crossed a threshold), "window" (it ran out), "left" (they left)."""
    details: list[dict[str, Any]] = field(default_factory=list)

    def words(self) -> str:
        """"waved at me (1.5 m), crouched down to my level" or "none of the signs ..."."""
        return ", ".join(self.evidence) if self.evidence else "showed no sign of wanting to interact"

    def to_dict(self) -> dict[str, Any]:
        out = asdict(self)
        out["score"] = round(self.score, 2)
        out["watched_s"] = round(self.watched_s, 1)
        return out


def reading_cues(reading: CueReading, distance_m: float | None, receding: bool = True) -> list[str]:
    """The cues one reading shows (see CUE_WEIGHTS). `receding`: the track shows them moving away
    after their closest point; walking past counts only then (one frame of someone walking at an
    angle cannot tell coming from going)."""
    if not reading.person_visible:
        return []
    cues: list[str] = []
    gesture = GESTURE_CUES.get(reading.gesture)
    if gesture and reading.facing != "away":
        cues.append(gesture)
    near = distance_m is None or distance_m <= NEAR_M + 0.5
    if reading.posture == "crouching" and reading.facing in ("toward_me", "sideways") and near:
        cues.append("crouch")
    if reading.looking_at_me == "yes" and reading.facing == "toward_me" and reading.posture in ("standing", "crouching", "sitting") and near:
        cues.append("looks_at_me")
    if reading.busy_with.strip() and reading.facing != "toward_me" and not gesture:
        cues.append("busy")
    if reading.posture == "walking" and reading.facing in ("sideways", "away") and receding:
        cues.append("passing")
    if reading.facing == "away" and reading.posture != "walking" and not reading.busy_with.strip():
        cues.append("facing_away")
    return cues


def score_cues(cue_counts: Mapping[str, int], weight: Callable[[str], float]) -> float:
    total = 0.0
    for cue, count in cue_counts.items():
        times = min(count, LOOK_AT_ME_MAX) if cue == "looks_at_me" else min(count, 1)
        total += CUE_WEIGHTS.get(cue, 0.0) * weight(cue) * times
    return total


LookFn = Callable[[], Awaitable[tuple[dict[str, Any] | None, bytes | None, str | None]]]


async def engage(
    person_id: str,
    look: LookFn,
    reader: CueReader | Any,
    track: Track,
    memory: "SocialMemory",
    *,
    window_s: float = 10.0,
    look_every_s: float = 2.0,
    clock: Callable[[], float] = time.monotonic,
    sleep: Callable[[float], Awaitable[None]] = asyncio.sleep,
    on_look: Callable[[dict[str, Any] | None, bytes | None, str | None], None] | None = None,
) -> Engagement:
    """Watch `person_id` for up to `window_s` (see the module docstring)."""
    started = clock()
    looks: list[Look] = []
    pending: list[tuple[Look, asyncio.Task]] = []
    counts: dict[str, int] = {}
    first_at: dict[str, float] = {}
    evidence: list[str] = []

    def note(cue: str, t: float, distance: float | None) -> None:
        counts[cue] = counts.get(cue, 0) + 1
        if cue not in first_at:
            first_at[cue] = t
            where = f" ({distance:.1f} m)" if isinstance(distance, (int, float)) else ""
            evidence.append(CUE_WORDS.get(cue, cue) + where)

    strong_looks: list[Look] = []
    held_back: list[tuple[str, Look]] = []
    """Strong cues read too close to count on their own (CLOSE_CONFIRM_M), until another look confirms."""

    def harvest() -> None:
        for item in list(pending):
            look_, task = item
            if task.done():
                pending.remove(item)
                reading = task.result() if not task.cancelled() and task.exception() is None else None
                if reading is not None:
                    look_.reading = reading.model_dump()
                    look_.cues = reading_cues(reading, look_.distance_m, receding=track.receding_after_closest(started))
                    if any(cue in STRONG_CUES for cue in look_.cues):
                        strong_looks.append(look_)
                    close = look_.distance_m is not None and look_.distance_m < CLOSE_CONFIRM_M
                    for cue in look_.cues:
                        if cue in STRONG_CUES and close and len(strong_looks) < 2:
                            held_back.append((cue, look_))
                        else:
                            note(cue, look_.t, look_.distance_m)
        if len(strong_looks) >= 2 and held_back:  # confirmed by a second look
            for cue, look_ in held_back:
                note(cue, look_.t, look_.distance_m)
            held_back.clear()

    def decide(now: float, final: bool) -> tuple[str, str] | None:
        feats = track.features(started, now)
        if feats["leaving"] and track.points and not track.receding_after_closest(started) and now - (track.last_seen() or now) < LOST_AFTER_S:
            feats["leaving"] = False  # still on their way over, just not nearer yet
        for name, on in (("approached", feats["approached"]), ("leaving", feats["leaving"])):
            if on and name not in counts:
                note(name, now, track.points[-1][1] if track.points else None)
        latest = next((lk.reading for lk in reversed(looks) if lk.reading), None)
        if feats["stopped_near"] and latest and latest.get("facing") == "toward_me" and "stays_near" not in counts:
            note("stays_near", now, track.points[-1][1] if track.points else None)
        score = score_cues(counts, memory.weight)
        elapsed = counted["s"]
        if score >= INTERESTED_SCORE:
            return INTERESTED, "evidence"
        if feats["leaving"] and score < INTERESTED_SCORE and now - started >= min(EARLY_NO_AFTER_S, window_s):
            return NOT_INTERESTED, "left"
        if score <= NOT_INTERESTED_SCORE and elapsed >= EARLY_NO_AFTER_S:
            return NOT_INTERESTED, "evidence"
        if final:
            return NOT_INTERESTED, "window"
        return None

    outcome: tuple[str, str] | None = None
    counted = {"s": 0.0, "at": started}
    """Window time that counts: not while they are still walking up to the pet (see APPROACH_STEP_M)."""

    def count_time(now: float) -> None:
        if not track.approaching() or not track.points or now - (track.last_seen() or 0.0) > look_every_s * 1.5:
            counted["s"] += now - counted["at"]
        counted["at"] = now

    while True:
        now = clock()
        try:
            payload, image, mime = await look()
        except Exception as exc:  # noqa: BLE001 - a missed look is a look without the person
            log.warning("engagement look failed: %s", exc)
            payload, image, mime = None, None, None
        if on_look is not None:
            try:
                on_look(payload, image, mime)
            except Exception:  # noqa: BLE001
                log.exception("engagement look listener failed")
        person = next((d for d in (payload or {}).get("destinations") or [] if isinstance(d, dict) and d.get("id") == person_id), None)
        distance = float(person["distance_m"]) if person and isinstance(person.get("distance_m"), (int, float)) else None
        from gpt_pet.trace import bearing_from_box

        bearing = bearing_from_box(person.get("bbox")) if person else None
        now = clock()
        lk = Look(t=now, distance_m=distance, bearing_deg=bearing, visible=person is not None)
        looks.append(lk)
        if person is not None:
            track.add(now, distance, bearing)
            if image:
                context = f"the person is about {distance:.1f} m away" if distance is not None else ""
                pending.append((lk, asyncio.ensure_future(reader.read(image, mime or "image/jpeg", context, person.get("bbox")))))
        harvest()
        count_time(now)
        outcome = decide(now, final=False)
        if outcome is not None:
            break
        if counted["s"] >= window_s or now - started >= MAX_WINDOW_S:
            break
        await sleep(max(0.0, min(look_every_s, window_s - counted["s"], MAX_WINDOW_S - (now - started))))
        harvest()
        count_time(clock())
        outcome = decide(clock(), final=False)
        if outcome is not None:
            break
    # the readings still on their way may settle it
    if outcome is None or outcome[0] != INTERESTED:
        for _, task in pending:
            try:
                await asyncio.wait_for(asyncio.shield(task), timeout=max(0.5, (reader.timeout_s if hasattr(reader, "timeout_s") else 5.0)))
            except Exception as exc:  # noqa: BLE001 - a reading that never came is a look without one
                log.debug("a late cue reading did not come: %s", exc)
        harvest()
        late = decide(clock(), final=True)
        if outcome is None or late[0] == INTERESTED:
            outcome = late
    for _, task in pending:
        task.cancel()
    score = score_cues(counts, memory.weight)
    readings = sum(1 for lk in looks if lk.reading)
    return Engagement(
        person_id=person_id,
        decision=outcome[0],
        score=score,
        cues=sorted(counts),
        evidence=list(evidence),
        watched_s=clock() - started,
        looks=len(looks),
        readings=readings,
        decided_by=outcome[1],
        details=[asdict(lk) for lk in looks],
    )


# --- what the pet keeps ---------------------------------------------------------------------------


@dataclass
class PersonRecord:
    """What the pet knows about one person."""

    id: str
    encounters: int = 0
    interested: int = 0
    not_interested: int = 0
    interactions: int = 0
    """Interested decisions that held (they were still near the pet afterwards)."""
    came_back: int = 0
    """Not-interested decisions they proved wrong (came right up to the pet)."""
    streak: int = 0
    """Not-interested decisions in a row (sets the leave-be time)."""
    status: str = "unknown"
    """unknown | interested | ignoring"""
    since: float = 0.0
    until: float = 0.0
    """ignoring: until when (wall clock); interested: news until when."""
    last_words: str = ""
    lingered: bool = False
    """The one "stayed right beside me" second look this decision allows has been used."""
    decided_m: float | None = None
    """How far they were when the pet last decided (someone left be is looked at again only when
    they come REOPEN_CLOSER_M closer than that)."""
    told: bool = False
    """The goal setter has heard about their interest."""


@dataclass
class Check:
    person_id: str
    decision: str
    cues: list[str]
    decided_at: float
    due_at: float


class SocialMemory:
    """Per person: status, leave-be time and history; per cue: how often it was right. Wall-clock
    times (`clock`), so a memory file stays valid across sessions."""

    def __init__(self, settings: "PeopleSettings | None" = None, *, path: str | Path | None = None, clock: Callable[[], float] = time.time) -> None:
        self.ignore_for_s = settings.ignore_for_s if settings is not None else 120.0
        self.ignore_max_s = settings.ignore_max_s if settings is not None else 900.0
        self.clock = clock
        self.path = Path(path) if path else None
        self.persons: dict[str, PersonRecord] = {}
        self.cue_stats: dict[str, list[int]] = {}
        """cue: [right, wrong]: how often the cue pointed the way the decision's check came out."""
        self.checks: list[Check] = []
        self.log: deque = deque(maxlen=50)
        if self.path is not None and self.path.is_file():
            self.load()

    # --- status -----------------------------------------------------------------------------

    def person(self, person_id: str) -> PersonRecord:
        if person_id not in self.persons:
            self.persons[person_id] = PersonRecord(id=person_id)
        return self.persons[person_id]

    def status(self, person_id: str) -> str:
        record = self.persons.get(person_id)
        if record is None:
            return "unknown"
        now = self.clock()
        if record.status in ("ignoring", "interested") and now >= record.until:
            record.status = "unknown"
        return record.status

    def ignoring(self, person_id: str) -> bool:
        return self.status(person_id) == "ignoring"

    def interested(self, person_id: str) -> bool:
        return self.status(person_id) == "interested"

    def leave_be_s(self, person_id: str) -> float:
        record = self.persons.get(person_id)
        return max(0.0, record.until - self.clock()) if record is not None and self.ignoring(person_id) else 0.0

    # --- learning ---------------------------------------------------------------------------

    def weight(self, cue: str) -> float:
        """The cue's learned multiplier: 0.5 + P(right), P from its record with a uniform prior."""
        right, wrong = self.cue_stats.get(cue, [0, 0])
        p = (right + 1) / (right + wrong + 2)
        return min(WEIGHT_RANGE[1], max(WEIGHT_RANGE[0], 0.5 + p))

    def _learn(self, cues: Iterable[str], interest_was_real: bool) -> None:
        for cue in cues:
            positive = CUE_WEIGHTS.get(cue, 0.0) > 0
            right = positive == interest_was_real
            stats = self.cue_stats.setdefault(cue, [0, 0])
            stats[0 if right else 1] += 1

    # --- decisions --------------------------------------------------------------------------

    def record(self, engagement: Engagement) -> PersonRecord:
        """Remember a decision: set the person's status (and leave-be time) and schedule its check."""
        now = self.clock()
        record = self.person(engagement.person_id)
        record.encounters += 1
        record.lingered = False
        record.last_words = engagement.words()
        record.decided_m = next((d["distance_m"] for d in reversed(engagement.details) if d.get("distance_m") is not None), None)
        record.since = now
        record.told = False
        if engagement.decision == INTERESTED:
            record.interested += 1
            record.status = "interested"
            record.until = now + INTEREST_FRESH_S
            self.checks.append(Check(engagement.person_id, INTERESTED, list(engagement.cues), now, now + CONFIRM_AFTER_S))
        else:
            record.not_interested += 1
            record.streak += 1
            record.status = "ignoring"
            record.until = now + min(self.ignore_max_s, self.ignore_for_s * 2 ** (record.streak - 1))
            self.checks.append(Check(engagement.person_id, NOT_INTERESTED, list(engagement.cues), now, now + REFUTE_WITHIN_S))
        self.log.append({"t": now, "person": engagement.person_id, "decision": engagement.decision, "cues": list(engagement.cues)})
        self.save()
        return record

    def reopen(self, person_id: str, why: str) -> None:
        """Stop leaving someone be (they came right up to the pet): their interest is unknown again."""
        record = self.person(person_id)
        if record.status == "ignoring":
            record.status = "unknown"
            record.until = self.clock()
            record.last_words = why
            log.info("%s %s: looking at them again", person_id, why)
            self.save()

    def interaction_over(self, person_id: str) -> None:
        """The interaction with them is over (the pet is off its goal about them): leave them be for
        REST_AFTER_INTERACTION_S, which is not disinterest (no streak), then their interest is
        unknown again."""
        record = self.persons.get(person_id)
        if record is not None and record.status == "interested":
            record.status = "ignoring"
            record.until = self.clock() + REST_AFTER_INTERACTION_S
            record.last_words = "just interacted with you"
            self.save()

    def resolve(self, last_near: Callable[[str, float, float], bool], came_up: Callable[[str, float], bool]) -> list[tuple[Check, bool]]:
        """Check due decisions: `last_near(person, since, within_m)` (seen that close since then),
        `came_up(person, since)` (came within REOPEN_M and stayed). Returns (check, was it right)."""
        now = self.clock()
        settled = []
        for check in list(self.checks):
            record = self.person(check.person_id)
            if check.decision == INTERESTED:
                if now < check.due_at:
                    continue
                real = last_near(check.person_id, check.due_at - 5.0, CONFIRM_NEAR_M)
                if real:
                    record.interactions += 1
                    record.streak = 0
                self._learn(check.cues, interest_was_real=real)
                settled.append((check, real))
            else:
                came = came_up(check.person_id, check.decided_at)
                if not came and now < check.due_at:
                    continue
                if came:
                    record.came_back += 1
                self._learn(check.cues, interest_was_real=came)
                settled.append((check, not came))
            self.checks.remove(check)
        if settled:
            self.save()
        return settled

    # --- what the goal setter reads ---------------------------------------------------------

    def person_line(self, person_id: str) -> str:
        record = self.persons.get(person_id)
        if record is None or record.encounters == 0:
            return ""
        status = self.status(person_id)
        history = f"met {record.encounters} time{'s' if record.encounters != 1 else ''}: wanted to interact {record.interested}, not interested {record.not_interested}"
        if status == "interested":
            return f"wants to interact with you ({record.last_words}); {history}"
        if status == "ignoring":
            return f"not interested ({record.last_words}); leave them be for another {self.leave_be_s(person_id):.0f} s; {history}"
        return history

    def lessons(self, limit: int = 4) -> list[str]:
        """What the pet has learned about cues: "waved at me: right 3 of 3 times"."""
        rows = sorted(self.cue_stats.items(), key=lambda kv: -(kv[1][0] + kv[1][1]))
        out = []
        for cue, (right, wrong) in rows[:limit]:
            if right + wrong == 0:
                continue
            meaning = "wanted to interact" if CUE_WEIGHTS.get(cue, 0) > 0 else "did not want to interact"
            out.append(f'"{CUE_WORDS.get(cue, cue)}" meant they {meaning} {right} of {right + wrong} times')
        return out

    # --- keeping ----------------------------------------------------------------------------

    def to_json(self) -> dict[str, Any]:
        return {
            "persons": {pid: asdict(rec) for pid, rec in self.persons.items()},
            "cue_stats": self.cue_stats,
            "checks": [asdict(c) for c in self.checks],
            "saved_at": self.clock(),
        }

    def save(self) -> None:
        if self.path is None:
            return
        try:
            self.path.parent.mkdir(parents=True, exist_ok=True)
            tmp = self.path.with_suffix(".tmp")
            tmp.write_text(json.dumps(self.to_json(), indent=1), encoding="utf-8")
            tmp.replace(self.path)
        except OSError as exc:
            log.warning("could not save the social memory to %s: %s", self.path, exc)

    def load(self) -> None:
        try:
            data = json.loads(self.path.read_text(encoding="utf-8"))  # type: ignore[union-attr]
        except (OSError, ValueError) as exc:
            log.warning("could not read the social memory %s: %s", self.path, exc)
            return
        self.persons = {pid: PersonRecord(**rec) for pid, rec in (data.get("persons") or {}).items()}
        self.cue_stats = {cue: list(v) for cue, v in (data.get("cue_stats") or {}).items()}
        self.checks = [Check(**c) for c in data.get("checks") or []]
        log.info("social memory: %d people, %d cues learned from %s", len(self.persons), len(self.cue_stats), self.path)
