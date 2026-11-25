"""The pet and people (`[people]`; gptpet-plans: People Interaction Design, step 2).

People are not places: they move, so the places memory leaves them out and the pet reacts to them
instead of exploring them. Everything here runs without a model call:

- `People`: who the pet has seen, where and when, and when it last reacted to each person.
- `PersonWatch`: looks for people every `watch_every_s` while the robot drives (a person coming
  into view stops a drive that is not about them) and while it rests between ticks (a person
  wakes it up), so a 2 s appearance is not missed between the brain's own pictures.
- the attention wait (`make_attention_wait`): `set_attention` returns once the robot has framed
  the person (or lost them), so the executor's next look shows them centred with the camera
  tilted for their distance.

What the pet then does is the models' business, helped by System 1's rules (`gpt_pet.s1`: attend
to a person who comes into view and hand the stretch back; on a social goal attend, look, and let
the model say what the person is doing) and by a People section in the goal setter's prompt.

Reactions are rate-limited per person and kind (`react_every_s`): after reacting to someone the
pet does not interrupt itself for them again for a while, unless its goal is about them.

With `[people] engage` (`gpt_pet.social`), turning to a person whose interest is unknown starts an
interest check: the attention wait goes on watching them for up to `engage_window_s` and decides
whether they want to interact. People who do become news for the goal setter ("Human wants to
interact with me!"); people who do not are left be for a while: no drive stops, wake-ups or turns
for them, unless they walk right up to the pet.
"""
from __future__ import annotations

import asyncio
import logging
import re
import time
from collections.abc import Callable, Iterable, Mapping
from dataclasses import dataclass
from typing import TYPE_CHECKING, Any

from gpt_pet.mcp import mcp_media_callback
from gpt_pet.social import (
    CUE_WORDS,
    INTERESTED,
    LINGER_AFTER_S,
    LINGER_M,
    REOPEN_CLOSER_M,
    REOPEN_M,
    REOPEN_STAY_S,
    Engagement,
    SocialMemory,
    Track,
    engage,
    reading_cues,
)
from gpt_pet.trace import bearing_from_box

if TYPE_CHECKING:
    from gpt_pet.frames import FrameStore
    from gpt_pet.goals import GoalInterrupt
    from gpt_pet.settings import PeopleSettings
    from gpt_pet.social import CueReader

log = logging.getLogger("gpt_pet.people")

VIEW_TOOL = "get_current_view"
ATTENTION_TOOL = "set_attention"
ATTENTION_STATUS_TOOL = "get_attention_status"
CANCEL_ATTENTION_TOOL = "cancel_attention"
ATTENTION_TOOLS = (ATTENTION_TOOL, ATTENTION_STATUS_TOOL, CANCEL_ATTENTION_TOOL)
ATTENTION_LIMITS = {ATTENTION_TOOL: 2, ATTENTION_STATUS_TOOL: 3, CANCEL_ATTENTION_TOOL: 1}
"""Per-tick caps for the attention tools unless `[tool_limits]` sets its own."""
ACTIVE_ATTENTION = ("aiming", "approaching", "framed", "searching")
SETTLED_ATTENTION = ("framed", "lost", "off")
ATTENTION_POLL_S = 0.5
FRESH_S = 20.0
"""A sighting this recent is news for the goal setter."""
PERSON_WORDS = re.compile(
    r"\b(person|people|human|someone|somebody|owner|man|woman|visitor|resident|guest|child|kid)\b", re.IGNORECASE
)
EXECUTOR, SETTER, WATCH = "executor", "setter", "watch"
UNKNOWN_RETRY_S = 15.0
GLANCE_M = 2.5
"""Someone the pet leaves be, seen this close: one cue reading (at most every GLANCE_EVERY_S) checks
whether they are trying to get its attention after all."""
GLANCE_EVERY_S = 15.0
GLANCE_TILT_DEG = 15.0
GLANCE_LEVEL_M = 1.8
"""With the camera level (tilted less than GLANCE_TILT_DEG), someone nearer than this shows only
their legs: no glance."""
STRONG_CUES = frozenset({"wave", "beckon", "clap", "crouch"})
"""Cues that end a leave-be on a glance ("reach" is too easily someone working at a table)."""
GLANCE_AFTER_DECISION_S = 15.0
"""No glance this soon after deciding: the check has just watched them."""


@dataclass
class Sighting:
    """One person in one view."""

    id: str
    label: str
    distance_m: float | None
    bearing_deg: float | None
    description: str
    seen_at: float
    source: str
    """"executor" (the brain's own look) or "watch"."""

    def words(self) -> str:
        """"Person (obj:person_0), 1.8 m ahead-left, walking" (the view's own description)."""
        text = self.description or (f"{self.label}, {self.distance_m} m" if self.distance_m is not None else self.label)
        return f"{self.label} ({self.id}): {text}"


def goal_key(goal: Mapping[str, Any] | None) -> Any:
    return goal.get("id") if isinstance(goal, Mapping) and goal.get("status") == "active" else None


class People:
    """Who the pet has seen and when it last reacted to each person (see the module docstring)."""

    def __init__(
        self,
        labels: Iterable[str] = ("Person",),
        *,
        react_every_s: float = 60.0,
        clock: Callable[[], float] = time.monotonic,
        social: SocialMemory | None = None,
    ) -> None:
        self.labels = {label.strip().lower() for label in labels if label.strip()}
        self.react_every_s = react_every_s
        self.clock = clock
        self.last: dict[str, Sighting] = {}
        self.reacted: dict[tuple[str, str], float] = {}
        self.attention: dict[str, Any] | None = None
        """The latest attention status the brain saw (after `set_attention`, or in a view)."""
        self.counts = {"sightings": 0, "watch_looks": 0, "drive_interrupts": 0, "rest_wakes": 0, "attended": 0, "escalations": 0}
        self.social = social
        """`[people] engage`: what the pet knows about people's interest (None: engage off)."""
        self.tracks: dict[str, Track] = {}
        """Each person's distance and bearing, look by look (wall-clock times when engage is on)."""
        self.on_event: Callable[[dict[str, Any]], None] | None = None
        """Where social notices go ("Human wants to interact with me!"): the runtime's notices."""
        self.reader: Any = None
        """The cue reader (engage on): also glances at people left be (see `glance`)."""
        self.glanced: dict[str, float] = {}
        if social is not None:
            self.counts.update(
                {"engagements": 0, "interested": 0, "not_interested": 0, "cue_reads": 0, "reopened": 0, "left_be": 0, "glances": 0}
            )

    # --- what is a person ---------------------------------------------------------------------

    def is_person(self, destination: Mapping[str, Any]) -> bool:
        return str(destination.get("label") or "").strip().lower() in self.labels

    def in_view(self, view: Mapping[str, Any] | None) -> list[dict[str, Any]]:
        return [d for d in (view or {}).get("destinations") or [] if isinstance(d, dict) and d.get("id") and self.is_person(d)]

    def without_people(self, view: Mapping[str, Any] | None) -> dict[str, Any] | None:
        """`view` with its people left out: what places, drives and exploring choose from."""
        if view is None:
            return None
        return {**view, "destinations": [d for d in view.get("destinations") or [] if not (isinstance(d, Mapping) and self.is_person(d))]}

    def is_social(self, goal: Mapping[str, Any] | None) -> bool:
        """The goal is about a person: kind "social" or a person as its target; a goal without a
        kind (the owner's, the default) by person words in the goal itself. Not by its success
        criteria: an exploration "identifies an interesting person, animal, room, or object"."""
        if not isinstance(goal, Mapping):
            return False
        if goal.get("kind") == "social" or str(goal.get("target_id") or "") in self.last:
            return True
        if goal.get("kind"):
            return False
        return bool(PERSON_WORDS.search(str(goal.get("goal") or "")))

    # --- remembering --------------------------------------------------------------------------

    def saw(self, view: Mapping[str, Any] | None, source: str) -> list[Sighting]:
        """Remember the people in a view (and the attention status it carries); returns them."""
        now = self.clock()
        if isinstance((view or {}).get("attention"), Mapping):
            self.attention = dict(view["attention"])  # type: ignore[index]
        found = []
        track_now = self.social.clock() if self.social is not None else now
        for destination in self.in_view(view):
            distance = destination.get("distance_m")
            sighting = Sighting(
                id=str(destination["id"]),
                label=str(destination.get("label") or "Person"),
                distance_m=float(distance) if isinstance(distance, (int, float)) else None,
                bearing_deg=bearing_from_box(destination.get("bbox")),
                description=str(destination.get("description") or ""),
                seen_at=now,
                source=source,
            )
            self.last[sighting.id] = sighting
            found.append(sighting)
            self.tracks.setdefault(sighting.id, Track()).add(track_now, sighting.distance_m, sighting.bearing_deg)
        self.counts["sightings"] += len(found)
        if self.social is not None:
            for sighting in found:
                self._maybe_reopen(sighting, track_now)
            self.settle()
        return found

    # --- interest (`[people] engage`) ----------------------------------------------------------

    def leaving_be(self, person_id: str) -> bool:
        """The pet decided this person is not interested and leaves them be for now."""
        return self.social is not None and self.social.ignoring(person_id)

    def wants_to_interact(self, person_id: str) -> bool:
        return self.social is not None and self.social.interested(person_id)

    def interest_known(self, person_id: str) -> bool:
        """Engage on and the pet already knows whether they want to interact (no check needed)."""
        return self.social is not None and self.social.status(person_id) != "unknown"

    def _maybe_reopen(self, sighting: Sighting, now: float) -> None:
        """A person the pet leaves be who walks right up to it (closer than when it decided) and
        stays gets looked at again; so does one who stays right beside it a while (once)."""
        if not self.social.ignoring(sighting.id) or sighting.distance_m is None or sighting.distance_m > REOPEN_M:
            return
        record = self.social.person(sighting.id)
        track = self.tracks[sighting.id]
        lingering = (
            not record.lingered
            and sighting.distance_m <= LINGER_M
            and now - record.since >= LINGER_AFTER_S
            and track.stayed_within(LINGER_M + 0.2, REOPEN_STAY_S, now)
        )
        if lingering:
            record.lingered = True
            self.social.reopen(sighting.id, "stayed right beside me")
            self.counts["reopened"] += 1
            for kind in (EXECUTOR, WATCH, SETTER):
                self.reacted.pop((kind, sighting.id), None)
            return
        decided = record.decided_m
        if decided is not None and decided - sighting.distance_m < REOPEN_CLOSER_M:
            return
        if track.stayed_within(REOPEN_M + 0.2, REOPEN_STAY_S, now):
            self.social.reopen(sighting.id, "came right up to me")
            self.counts["reopened"] += 1
            for kind in (EXECUTOR, WATCH, SETTER):
                self.reacted.pop((kind, sighting.id), None)

    async def glance(self, view: Mapping[str, Any] | None, image: bytes | None, mime_type: str | None) -> str | None:
        """Someone the pet leaves be, in view within GLANCE_M: one cue reading (at most every
        GLANCE_EVERY_S per person). A wave, beckon, clap, reach or crouch toward the pet ends the
        leave-be (they are trying to get its attention after all): their interest is unknown again,
        so the pet turns to them and checks. Returns the person's id when it did."""
        if self.social is None or self.reader is None or not image:
            return None
        now = self.clock()
        tilt = float(((view or {}).get("camera") or {}).get("tilt_deg") or 0.0)
        for destination in self.in_view(view):
            person_id = str(destination["id"])
            distance = destination.get("distance_m")
            if not self.leaving_be(person_id) or not isinstance(distance, (int, float)) or distance > GLANCE_M:
                continue
            if tilt < GLANCE_TILT_DEG and distance < GLANCE_LEVEL_M:
                continue  # camera level and them close: only their legs are in the frame
            if now - self.glanced.get(person_id, -1e9) < GLANCE_EVERY_S:
                continue
            if self.social.clock() - self.social.person(person_id).since < GLANCE_AFTER_DECISION_S:
                continue
            self.glanced[person_id] = now
            self.counts["glances"] += 1
            self.counts["cue_reads"] += 1
            reading = await self.reader.read(image, mime_type or "image/jpeg", f"the person is about {distance:.1f} m away", destination.get("bbox"))
            if reading is None:
                continue
            strong = [cue for cue in reading_cues(reading, float(distance)) if cue in STRONG_CUES]
            if not strong:
                continue
            words = CUE_WORDS.get(strong[0], strong[0])
            self.social.reopen(person_id, words)
            self.counts["reopened"] += 1
            for kind in (EXECUTOR, WATCH, SETTER):
                self.reacted.pop((kind, person_id), None)
            label = str(destination.get("label") or "Person")
            self.emit({"kind": "person", "text": f"{label} ({person_id}) {words} ({distance:.1f} m) while I was leaving them be; looking again."})
            return person_id
        return None

    def settle(self) -> None:
        """Check due interest decisions against what the robot has seen since (social.resolve)."""
        if self.social is None:
            return

        def last_near(person_id: str, since: float, within_m: float) -> bool:
            track = self.tracks.get(person_id)
            return track is not None and any(t >= since and d <= within_m for t, d, _ in track.points)

        def came_up(person_id: str, since: float) -> bool:
            track = self.tracks.get(person_id)
            if track is None:
                return False
            close = [t for t, d, _ in track.points if t >= since and d <= REOPEN_M + 0.3]
            return len(close) >= 2 and close[-1] - close[0] >= REOPEN_STAY_S * 0.8

        for check, right in self.social.resolve(last_near, came_up):
            log.info("interest check: %s was %s for %s", check.decision, "right" if right else "wrong", check.person_id)

    def emit(self, notice: dict[str, Any]) -> None:
        if self.on_event is not None:
            try:
                self.on_event(notice)
            except Exception:  # noqa: BLE001 - a broken listener must not stop the pet
                log.exception("social notice listener failed")

    def record_engagement(self, engagement: Engagement) -> None:
        """Remember an interest check's outcome and say so (notice kinds "social" and "ignore")."""
        assert self.social is not None
        self.counts["engagements"] += 1
        record = self.social.record(engagement)
        sighting = self.last.get(engagement.person_id)
        who = sighting.label if sighting is not None else "The person"
        if engagement.decision == INTERESTED:
            self.counts["interested"] += 1
            text = f"Human wants to interact with me! {who} ({engagement.person_id}) {engagement.words()}."
            self.emit({"kind": "social", "text": text, "engagement": engagement.to_dict()})
        else:
            self.counts["not_interested"] += 1
            left = max(0.0, record.until - self.social.clock())
            text = (
                f"{who} ({engagement.person_id}) does not seem interested ({engagement.words()}); "
                f"leaving them be for {left / 60:.0f} min."
                if left >= 90
                else f"{who} ({engagement.person_id}) does not seem interested ({engagement.words()}); leaving them be for {left:.0f} s."
            )
            self.emit({"kind": "ignore", "text": text, "engagement": engagement.to_dict()})

    def recent(self, within_s: float = FRESH_S) -> list[Sighting]:
        now = self.clock()
        return sorted((s for s in self.last.values() if now - s.seen_at <= within_s), key=lambda s: -s.seen_at)

    def attending(self, person_id: str | None = None) -> bool:
        attention = self.attention or {}
        return attention.get("state") in ACTIVE_ATTENTION and (person_id is None or attention.get("target") == person_id)

    # --- reacting -----------------------------------------------------------------------------

    def due(self, kind: str, person_id: str) -> bool:
        """Never reacted to this person this way, or long enough ago. With engage on, someone whose
        interest is still unknown (the last turn to them found nothing: they were lost from view)
        is turned to again after UNKNOWN_RETRY_S."""
        last = self.reacted.get((kind, person_id))
        if last is None:
            return True
        wait = self.react_every_s
        if self.social is not None and kind in (EXECUTOR, WATCH) and self.social.status(person_id) == "unknown":
            wait = min(wait, UNKNOWN_RETRY_S)
        return self.clock() - last >= wait

    def react(self, kind: str, person_id: str) -> None:
        self.reacted[(kind, person_id)] = self.clock()

    def to_react(self, kind: str, goal: Mapping[str, Any] | None, sightings: Iterable[Sighting]) -> Sighting | None:
        """The nearest person in `sightings` worth a reaction of this kind: never on a social goal
        (the pet is busy with people already), and not again soon for the same person. With engage
        on, never someone the pet leaves be, and for the goal setter only someone who wants to
        interact."""
        if self.is_social(goal):
            return None
        if self.social is not None:
            for s in sightings:  # the goal setter heard of their interest, and the pet is off it: over
                if self.social.interested(s.id) and self.social.person(s.id).told:
                    self.social.interaction_over(s.id)
        due = [s for s in sightings if self.due(kind, s.id)]
        if self.social is not None:
            skipped = [s for s in due if self.leaving_be(s.id)]
            if skipped:
                self.counts["left_be"] += 1
            due = [s for s in due if not self.leaving_be(s.id)]
            if kind == SETTER:
                due = [s for s in due if self.wants_to_interact(s.id)]
        return min(due, key=lambda s: s.distance_m if s.distance_m is not None else 99.0) if due else None

    def news_pending(self, goal: Mapping[str, Any] | None) -> Sighting | None:
        """A person seen lately whom the goal setter has not heard about yet (marks nothing): a
        resting pet does not keep the goal setter waiting for them."""
        return self.to_react(SETTER, goal, self.recent())

    def news_for_setter(self, goal: Mapping[str, Any] | None) -> Sighting | None:
        """A person seen lately whom the goal setter has not decided about yet (System 1 then
        leaves the goal decision to the model); marks them as heard."""
        sighting = self.to_react(SETTER, goal, self.recent())
        if sighting is not None:
            self.react(SETTER, sighting.id)
            self.counts["escalations"] += 1
            if self.social is not None:
                self.social.person(sighting.id).told = True
        return sighting

    # --- for the goal setter ------------------------------------------------------------------

    def summary(self) -> list[str]:
        """What the goal setter reads about people (empty when the pet has seen nobody)."""
        if not self.last:
            return []
        now = self.clock()
        lines = []
        for sighting in sorted(self.last.values(), key=lambda s: -s.seen_at)[:3]:
            ago = now - sighting.seen_at
            line = f"- {sighting.words()} (seen {ago:.0f} s ago)"
            interest = self.social.person_line(sighting.id) if self.social is not None else ""
            lines.append(line + (f": {interest}" if interest else ""))
        attention = self.attention or {}
        if attention.get("state") in ACTIVE_ATTENTION:
            tilt = attention.get("tilt_deg")
            lines.append(
                f"- The robot is keeping {attention.get('target')} in view by itself (attention {attention.get('state')}, "
                f"{attention.get('distance_m')} m{', camera tilted up' if tilt else ''})."
            )
        return lines


def view_from_tool_result(raw: Any) -> tuple[dict[str, Any] | None, bytes | None, str | None]:
    """(payload, image bytes, mime type) of a dumped get_current_view result; Nones on an error."""
    shaped = mcp_media_callback(None, {}, None, raw)
    if shaped is None or shaped["isError"] or not isinstance(shaped["result"], dict):
        return None, None, None
    media = shaped.get("media") or []
    blob = media[0].inline_data if media else None
    return shaped["result"], (bytes(blob.data) if blob is not None and blob.data else None), (blob.mime_type if blob is not None else None)


class PersonWatch:
    """Looks for people outside the tool flow: during drives (through the executor's MCP session)
    and between ticks (through its own). Each look updates the portal's camera frame and the trace."""

    def __init__(
        self,
        people: People,
        every_s: float,
        *,
        frames: "FrameStore | None" = None,
        on_view: Callable[..., Any] | None = None,
        url: str | None = None,
    ) -> None:
        self.people = people
        self.every_s = every_s
        self.frames = frames
        self.on_view = on_view
        self.url = url
        self._proxy: Any = None
        self.people_kind = WATCH

    def show(self, by: str, payload: Mapping[str, Any] | None, image: bytes | None, mime_type: str | None, tick: int | None) -> None:
        """Put a look on the portal's camera and in the trace."""
        if image and self.frames is not None:
            self.frames.put("camera", image, mime_type or "image/jpeg")
        if self.on_view is not None:
            try:
                self.on_view(by, image, mime_type, payload, tick)
            except Exception:  # noqa: BLE001 - a broken listener must not stop the watch
                log.exception("watch view listener failed")

    def note(self, payload: Mapping[str, Any], image: bytes | None, mime_type: str | None, tick: int | None, goal: Mapping[str, Any] | None) -> Sighting | None:
        """A look the watch took: show it, trace it, remember its people, and return a person
        worth interrupting for (see `People.to_react`)."""
        self.people.counts["watch_looks"] += 1
        self.show("watch", payload, image, mime_type, tick)
        sighting = self.people.to_react(WATCH, goal, self.people.saw(payload, WATCH))
        if sighting is not None:
            self.people.react(WATCH, sighting.id)
        return sighting

    async def look_between_ticks(self, tick: int | None, goal: Mapping[str, Any] | None) -> Sighting | None:
        """One look while the pet rests; a person worth waking up for, else None."""
        if self.url is None:
            return None
        if self._proxy is None:
            from gpt_pet.mcp_proxy import McpProxy

            self._proxy = McpProxy(self.url, timeout_s=20)
        try:
            result = await self._proxy.call(VIEW_TOOL, {})
        except Exception as exc:  # noqa: BLE001 - a missed look is not worth stopping for
            log.warning("person watch: look failed: %s", exc)
            return None
        if result.is_error or not isinstance(result.data, dict):
            return None
        sighting = self.note(result.data, result.image, result.mime_type, tick, goal)
        if sighting is None and await self.people.glance(result.data, result.image, result.mime_type):
            sighting = self.people.to_react(WATCH, goal, self.people.saw(result.data, WATCH))
        if sighting is not None:
            self.people.counts["rest_wakes"] += 1
        return sighting

    async def close(self) -> None:
        if self._proxy is not None:
            await self._proxy.close()
            self._proxy = None


def make_attention_wait(
    toolset: Any,
    settings: "PeopleSettings",
    people: People,
    interrupt: "GoalInterrupt | None" = None,
    reader: "CueReader | None" = None,
    watch: "PersonWatch | None" = None,
):
    """`after_tool_callback` part: `set_attention` returns once the robot has framed the person,
    lost them, or `attention_wait_s` passed, with that status as its result (no model calls
    meanwhile, like the drive wait). None for every other tool.

    With `[people] engage` (a `reader` and `people.social`), turning to someone whose interest is
    unknown, on a goal that is not about people, goes on into the interest check
    (`gpt_pet.social.engage`): the response then carries an "engagement" block with the decision,
    and attention ends when they are not interested."""

    async def wait_for_attention(tool: Any, args: Any, tool_context: Any, tool_response: Any) -> dict[str, Any] | None:
        if getattr(tool, "name", None) != ATTENTION_TOOL:
            return None
        response = mcp_media_callback(tool, args, tool_context, tool_response)
        if response is None or response["isError"] or not isinstance(response["result"], dict):
            return None  # refused or failed: nothing to wait for
        status_tool = next((t for t in await toolset.get_tools() if t.name == ATTENTION_STATUS_TOOL), None)
        status = dict(response["result"])
        started = time.monotonic()
        while status_tool is not None and status.get("state") not in SETTLED_ATTENTION:
            if time.monotonic() - started >= settings.attention_wait_s or (interrupt is not None and interrupt.requested):
                break
            await asyncio.sleep(ATTENTION_POLL_S)
            try:
                shaped = mcp_media_callback(None, {}, None, await status_tool.run_async(args={}, tool_context=tool_context))
            except Exception as exc:  # noqa: BLE001 - the model gets the last status it had
                log.warning("%s failed: %s", ATTENTION_STATUS_TOOL, exc)
                break
            if shaped is not None and not shaped["isError"] and isinstance(shaped["result"], dict):
                status = shaped["result"]
        people.attention = status
        people.counts["attended"] += 1
        waited = round(time.monotonic() - started, 1)
        person_id = str((args or {}).get("destination_id") or "")
        log.info("attention on %s: %s after %.1f s", person_id, status.get("state"), waited)
        out = {**response, "result": status, "waited_s": waited}
        if reader is None or people.social is None or not person_id or status.get("state") not in ACTIVE_ATTENTION:
            return out
        from gpt_pet.goals import STATE_CURRENT_GOAL, STATE_TICK

        state = getattr(tool_context, "state", None) or {}
        if people.is_social(state.get(STATE_CURRENT_GOAL)) or people.interest_known(person_id):
            return out
        engagement = await check_interest(toolset, settings, people, reader, watch, person_id, tool_context, int(state.get(STATE_TICK) or 0), interrupt)
        out["engagement"] = {k: v for k, v in engagement.to_dict().items() if k != "details"}
        if engagement.decision != INTERESTED:
            cancel_tool = next((t for t in await toolset.get_tools() if t.name == CANCEL_ATTENTION_TOOL), None)
            if cancel_tool is not None:
                try:
                    shaped = mcp_media_callback(None, {}, None, await cancel_tool.run_async(args={}, tool_context=tool_context))
                    if shaped is not None and not shaped["isError"] and isinstance(shaped["result"], dict):
                        people.attention = shaped["result"]
                        out["result"] = shaped["result"]
                except Exception as exc:  # noqa: BLE001
                    log.warning("%s failed: %s", CANCEL_ATTENTION_TOOL, exc)
        return out

    return wait_for_attention


async def check_interest(
    toolset: Any,
    settings: "PeopleSettings",
    people: People,
    reader: "CueReader",
    watch: "PersonWatch | None",
    person_id: str,
    tool_context: Any,
    tick: int,
    interrupt: "GoalInterrupt | None" = None,
) -> Engagement:
    """Watch `person_id` and decide whether they want to interact (`gpt_pet.social.engage`), with
    the executor's own MCP session for the looks; record and announce the decision."""
    view_tool = next((t for t in await toolset.get_tools() if t.name == VIEW_TOOL), None)

    async def look() -> tuple[dict[str, Any] | None, bytes | None, str | None]:
        if view_tool is None or (interrupt is not None and interrupt.requested):
            return None, None, None
        payload, image, mime = view_from_tool_result(await view_tool.run_async(args={}, tool_context=tool_context))
        people.saw(payload, "engage")
        if watch is not None:
            watch.show("engage", payload, image, mime, tick)
        return payload, image, mime

    class Counted:  # counts the cue reader's calls
        timeout_s = getattr(reader, "timeout_s", 10.0)

        async def read(self, image: bytes, mime: str, context: str = "", bbox: Any = None) -> Any:
            people.counts["cue_reads"] += 1
            return await reader.read(image, mime, context, bbox)

    track = people.tracks.setdefault(person_id, Track())
    engagement = await engage(
        person_id,
        look,
        Counted(),
        track,
        people.social,
        window_s=settings.engage_window_s,
        look_every_s=settings.engage_look_every_s,
        clock=people.social.clock,
    )
    log.info(
        "interest check on %s: %s after %.1f s (%s; score %.1f, %d looks, %d readings)",
        person_id, engagement.decision, engagement.watched_s, engagement.words(), engagement.score, engagement.looks, engagement.readings,
    )
    people.record_engagement(engagement)
    return engagement
