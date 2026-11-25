"""What the pet remembers about places, across ticks.

Everything comes from tool results the robot contract already has: each `get_current_view` gives
the robot's pose and, per destination, its kind, label and `distance_m`; each `set_nav_goal`
response names its destination and, once the drive is waited out (`gpt_pet.navwait`), its
outcome, planned length and final pose. From those `Places` keeps the robot's pose, a coarse
trail, the spots it looked around from, and per destination: how often it was seen, drives to it,
failed drives, and where a drive reached it.

It marks every view the executor reads, so Gemini and System 1 see the same thing:

- `here`: within the destination's reach, or reached by a drive that ended where the robot
  stands now. A drive there goes nowhere. The reach is `here_m` (`[brain] here_m`; a passage:
  PASSAGE_HERE_M) until a drive teaches better: a drive that succeeded from `distance_m` away with a planned path of
  `length_m` shows the destination counts as reached from `distance_m - length_m`. `distance_m`
  is measured to the object's centre, so a sofa is reached from 2.5 m and a cup from 1.5 m; the
  longest reach seen per destination and per label is kept (`reach_m`, `label_reach`). A doorway
  a metre away is still worth driving through, hence the passages' short radius.
- `visited`: a drive reached it before, or the robot was within its reach before.
- `failed`: how many drives to it failed.
- `failed_here` (`[brain] failed_guard`): a drive to it already failed from where the robot stands
  now; the executor refuses another one from here.

It also counts drives that failed one after another without the robot moving (`stuck_drives`), for
System 1's rule to back up and turn around (`[s1] unstuck_rule`).

Kept in memory with the brain, like the frames and the stats: session state would copy it into
every event of a long run.
"""
from __future__ import annotations

import copy
import math
import time
from collections.abc import Mapping
from dataclasses import dataclass, field
from typing import Any

from gpt_pet.navwait import NULL_DRIVE_M

LOOK = "get_current_view"
DRIVE = "set_nav_goal"
HERE_M = 1.5
"""By default an object this close is within reach until a drive shows its reach is longer
(`[brain] here_m` sets it per profile)."""
PASSAGE_HERE_M = 0.6
"""A passage this close is where the robot stands; farther, a drive takes it into the opening."""
MAX_REACH_M = 3.0
"""Learned reaches are capped: a longer one would be a stale distance, not a big object."""
SAME_SPOT_M = 0.1
"""A destination's distance counts for a drive only when it was seen from where the drive started."""
ARRIVAL_M = 0.4
"""The robot is still where an earlier drive ended when it is this close to that spot."""
TRAIL_STEP_M = 0.5
TRAIL_CAP = 300
LOOK_CELL_M = 0.5
PLACES_CAP = 300
MAX_FAILED_DRIVES = 2
"""A place drives failed to reach this often is no longer offered as somewhere to go."""
SUMMARY_PLACES = 8
STUCK_RADIUS_M = 0.3
"""Failed drives that start within this of each other, without the robot moving, count as stuck."""
MOVED_M = 0.1
"""A drive that took the robot less than this far went nowhere."""
FAILED_HERE_M = 0.5
"""A failed drive counts against a destination from spots this close to where it started."""
FAILED_SPOTS_CAP = 5
MARKS_NOTE = (
    "Marks from the pet's memory: here = already within reach, a drive there goes nowhere; "
    "visited = reached on an earlier drive; failed = drives there failed that many times."
)
FAILED_HERE_NOTE = " failed_here = a drive there already failed from this spot: back up or turn, or go elsewhere."
ALREADY_THERE_NOTE = "Already there: the robot did not move. Pick a destination farther away, or turn to see more."


@dataclass
class Place:
    id: str
    kind: str
    label: str
    description: str = ""
    distance_m: float | None = None
    seen: int = 0
    first_tick: int = 0
    last_tick: int = 0
    seen_from: tuple[float, float] | None = None
    """The robot's (x, z) when it last saw this place."""
    drives: int = 0
    failed: int = 0
    reached_tick: int | None = None
    reached_at: tuple[float, float] | None = None
    """Where the robot stood when a drive here ended, or when it last saw this place within reach."""
    reach_m: float | None = None
    """The longest distance a drive showed this destination counts as reached from."""
    failed_from: list[tuple[float, float]] = field(default_factory=list)
    """Where the robot stood when drives here failed (the latest FAILED_SPOTS_CAP)."""


@dataclass
class Places:
    here_m: float = HERE_M
    """An object at most this far away counts as within reach (until drives teach a longer reach)."""
    pose: dict[str, float] | None = None
    """The robot's latest {x, z, yaw}."""
    trail: list[tuple[float, float]] = field(default_factory=list)
    looks: dict[tuple[int, int], set[int]] = field(default_factory=dict)
    """Spots the robot looked from (0.5 m cells) -> headings seen (90-degree buckets)."""
    destinations: dict[str, Place] = field(default_factory=dict)
    label_reach: dict[str, float] = field(default_factory=dict)
    """Per label, the longest reach a drive showed: what counts as here for its other instances."""
    ticks: set[int] = field(default_factory=set)
    drive_outcomes: dict[str, int] = field(default_factory=dict)
    """Waited-out drives by outcome; `already there` counts the ones that went nowhere."""
    driven_m: float = 0.0
    started: float = field(default_factory=time.monotonic)
    failed_guard: bool = False
    """`[brain] failed_guard`: views mark `failed_here`, and the summary notes failed places."""
    stuck_drives: int = 0
    """Drives that failed in a row without taking the robot anywhere, from about `stuck_at`."""
    stuck_at: tuple[float, float] | None = None
    people_labels: frozenset[str] = frozenset()
    """`[people] labels` (lower case): people move, so they are never places (no marks, never
    unvisited, never a reach to learn). Empty: people are kept like any object (`[people]` off)."""

    def is_person(self, destination: Mapping[str, Any]) -> bool:
        return bool(self.people_labels) and str(destination.get("label") or "").strip().lower() in self.people_labels

    def blocked_by_person(self, reason: Any) -> bool:
        """A drive that failed because a person stood in the way ("blocked by Person near ..."):
        the person moves on, so it says nothing about the place or the robot being stuck."""
        text = str(reason or "").lower()
        return bool(self.people_labels) and any(f"blocked by {label}" in text for label in self.people_labels)

    # --- recording ---------------------------------------------------------------------------

    def at(self, pose: Mapping[str, Any] | None, tick: int) -> None:
        """Note the robot's pose; extend the trail once it moved TRAIL_STEP_M."""
        if not isinstance(pose, Mapping) or not _is_number(pose.get("x")) or not _is_number(pose.get("z")):
            return
        x, z = float(pose["x"]), float(pose["z"])
        self.pose = {"x": x, "z": z, "yaw": float(pose.get("yaw") or 0.0)}
        if self.stuck_at is not None and math.dist(self.stuck_at, (x, z)) > STUCK_RADIUS_M:
            self.stuck_drives, self.stuck_at = 0, None  # it got away
        self.ticks.add(tick)
        if not self.trail or math.dist(self.trail[-1], (x, z)) >= TRAIL_STEP_M:
            self.trail.append((x, z))
            del self.trail[:-TRAIL_CAP]

    def saw(self, view: Mapping[str, Any], tick: int) -> None:
        """Remember a view: the pose it was taken from and every destination in it."""
        self.at(view.get("agent"), tick)
        here = self._xz()
        if self.pose is not None and here is not None:
            cell = (round(here[0] / LOOK_CELL_M), round(here[1] / LOOK_CELL_M))
            self.looks.setdefault(cell, set()).add(int(round(self.pose["yaw"] / 90.0)) % 4)
        for raw in view.get("destinations") or []:
            if not isinstance(raw, Mapping) or not raw.get("id") or self.is_person(raw):
                continue
            place = self.destinations.get(str(raw["id"]))
            if place is None:
                if len(self.destinations) >= PLACES_CAP:
                    continue
                place = Place(id=str(raw["id"]), kind=str(raw.get("kind") or _kind(str(raw["id"]))), label=str(raw.get("label") or raw["id"]), first_tick=tick)
                self.destinations[place.id] = place
            place.seen += 1
            place.last_tick = tick
            place.description = str(raw.get("description") or place.description)
            place.distance_m = float(raw["distance_m"]) if _is_number(raw.get("distance_m")) else None
            place.seen_from = here
            if place.distance_m is not None and here is not None and place.distance_m <= self.reach(raw, place):
                # within reach is as good as a drive there: the pet has been at this place
                place.reached_tick = place.reached_tick if place.reached_tick is not None else tick
                place.reached_at = here

    def drove(self, destination_id: str | None, drive: Mapping[str, Any], tick: int, planned_m: float | None = None) -> None:
        """Remember a waited-out drive: its outcome, where the robot ended up, and what it shows
        about the destination's reach."""
        start = self._xz()
        final = drive.get("final_status") if isinstance(drive.get("final_status"), Mapping) else {}
        self.at(final.get("agent"), tick)
        outcome_name = str(drive.get("outcome"))
        if outcome_name == "succeeded" and planned_m is not None and planned_m < NULL_DRIVE_M:
            outcome_name = "already there"
        self.drive_outcomes[outcome_name] = self.drive_outcomes.get(outcome_name, 0) + 1
        traveled = (final.get("progress") or {}).get("distance_traveled_m") if isinstance(final.get("progress"), Mapping) else None
        self.driven_m += float(traveled) if _is_number(traveled) else 0.0
        outcome = drive.get("outcome")
        if outcome in ("failed", "error") and self.blocked_by_person(final.get("reason")):
            return  # a person in the way: not the place's fault, nor the robot stuck
        self._count_stuck(outcome, float(traveled) if _is_number(traveled) else None)
        place = self.destinations.get(str(destination_id)) if destination_id else None
        if place is None:
            return
        place.drives += 1
        if outcome == "succeeded":
            place.reached_tick = tick
            place.reached_at = self._xz()
            self._learn_reach(place, start, planned_m)
        elif outcome in ("failed", "error"):
            place.failed += 1
            if start is not None:
                place.failed_from = [*place.failed_from, start][-FAILED_SPOTS_CAP:]

    def _count_stuck(self, outcome: Any, traveled: float | None) -> None:
        """Count a drive that failed without taking the robot anywhere; any other drive ends a count."""
        here = self._xz()
        if outcome in ("failed", "error") and (traveled or 0.0) < MOVED_M and here is not None:
            if self.stuck_at is not None and math.dist(self.stuck_at, here) <= STUCK_RADIUS_M:
                self.stuck_drives += 1
            else:
                self.stuck_drives, self.stuck_at = 1, here
        else:
            self.stuck_drives, self.stuck_at = 0, None

    def _learn_reach(self, place: Place, start: tuple[float, float] | None, planned_m: float | None) -> None:
        seen_here = place.seen_from is not None and start is not None and math.dist(place.seen_from, start) <= SAME_SPOT_M
        if place.kind == "passage" or not seen_here or place.distance_m is None or planned_m is None:
            return
        reach = place.distance_m - planned_m
        if not 0 < reach <= MAX_REACH_M:
            return
        place.reach_m = max(place.reach_m or 0.0, reach)
        self.label_reach[place.label] = max(self.label_reach.get(place.label, 0.0), reach)

    def reach(self, destination: Mapping[str, Any], place: Place | None = None) -> float:
        """How close a destination must be to count as here."""
        destination_id = str(destination.get("id"))
        if str(destination.get("kind") or _kind(destination_id)) == "passage":
            return PASSAGE_HERE_M
        label = str(destination.get("label") or (place.label if place else ""))
        learned = max((place.reach_m or 0.0) if place else 0.0, self.label_reach.get(label, 0.0))
        return max(self.here_m, learned)

    # --- reading -----------------------------------------------------------------------------

    def marks(self, destination: Mapping[str, Any]) -> dict[str, Any]:
        """The marks for one destination of a view (empty when there is nothing to say)."""
        out: dict[str, Any] = {}
        if self.is_person(destination):
            return out
        place = self.destinations.get(str(destination.get("id")))
        distance = destination.get("distance_m")
        here = self._xz()
        reached_here = (
            place is not None and place.reached_at is not None and here is not None and math.dist(place.reached_at, here) <= ARRIVAL_M
        )
        if (_is_number(distance) and float(distance) <= self.reach(destination, place)) or reached_here:
            out["here"] = True
        elif place is not None and place.reached_tick is not None:
            out["visited"] = True
        if place is not None and place.failed:
            out["failed"] = place.failed
            if self.failed_guard and self.failed_here(place.id):
                out["failed_here"] = True
        return out

    def mark_view(self, view: Mapping[str, Any]) -> dict[str, Any]:
        """A copy of `view` with every destination's marks, and a note on what they mean."""
        marked = copy.deepcopy(dict(view))
        any_marks = False
        for destination in marked.get("destinations") or []:
            if isinstance(destination, dict):
                marks = self.marks(destination)
                destination.update(marks)
                any_marks = any_marks or bool(marks)
        if any_marks:
            marked["marks"] = MARKS_NOTE + (FAILED_HERE_NOTE if self.failed_guard else "")
        return marked

    def _xz(self) -> tuple[float, float] | None:
        return None if self.pose is None else (self.pose["x"], self.pose["z"])

    def is_here(self, destination_id: str) -> bool:
        """Whether a drive to this destination would go nowhere, judged from where the robot
        stands now: it was reached from this spot, or last seen from here within its reach."""
        place = self.destinations.get(destination_id)
        here = self._xz()
        if place is None or here is None:
            return False
        if place.reached_at is not None and math.dist(place.reached_at, here) <= ARRIVAL_M:
            return True
        seen_here = place.seen_from is not None and math.dist(place.seen_from, here) <= SAME_SPOT_M
        destination = {"id": place.id, "kind": place.kind, "label": place.label}
        return seen_here and place.distance_m is not None and place.distance_m <= self.reach(destination, place)

    def failed_here(self, destination_id: str) -> bool:
        """Whether a drive to this destination already failed from about where the robot stands."""
        place = self.destinations.get(destination_id)
        here = self._xz()
        if place is None or here is None:
            return False
        return any(math.dist(spot, here) <= FAILED_HERE_M for spot in place.failed_from)

    def unvisited(self) -> list[Place]:
        """Places seen but never reached, failed too often excluded: passages first, then objects,
        each most recently seen first."""
        open_ = [p for p in self.destinations.values() if p.reached_tick is None and p.failed < MAX_FAILED_DRIVES]
        if self.failed_guard:  # the ones drives already failed to reach go last
            return sorted(open_, key=lambda p: (p.failed > 0, p.kind != "passage", -p.last_tick, p.distance_m or 0.0))
        return sorted(open_, key=lambda p: (p.kind != "passage", -p.last_tick, p.distance_m or 0.0))

    def summary(self, now: float | None = None, limit: int = SUMMARY_PLACES) -> list[str]:
        """What the pet has done so far, in a few lines for the goal setter."""
        minutes = ((time.monotonic() if now is None else now) - self.started) / 60
        drives = sum(self.drive_outcomes.values())
        outcomes = ", ".join(f"{count} {name}" for name, count in sorted(self.drive_outcomes.items(), key=lambda kv: -kv[1]))
        lines = [
            f"Time: {minutes:.0f} min over {len(self.ticks)} ticks. Drives: {drives}"
            + (f" ({outcomes}), {self.driven_m:.1f} m driven." if drives else "."),
            f"Looked around from {len(self.looks)} spots.",
        ]
        visited = sorted((p for p in self.destinations.values() if p.reached_tick is not None), key=lambda p: p.reached_tick or 0)
        if visited:
            lines.append("Visited: " + ", ".join(f"{p.label} ({p.id})" for p in visited[-limit:]) + ".")
        fresh = self.unvisited()
        if fresh:
            lines.append("Seen but not visited yet, passages first (id: what, distance when last seen):")
            lines += [f"- {p.id}: {p.description or p.label}" + _distance_note(p) + _failed_note(p, self.failed_guard) for p in fresh[:limit]]
            if len(fresh) > limit:
                lines.append(f"- and {len(fresh) - limit} more.")
        else:
            lines.append("Seen but not visited yet: nothing; turn or go somewhere new to discover more.")
        return lines

    # --- the executor's after-tool hook ------------------------------------------------------

    def remember(self, tool_name: str, args: Mapping[str, Any] | None, response: dict[str, Any], tick: int) -> dict[str, Any]:
        """Learn from one tool response (as the media and drive-wait callbacks shaped it) and
        return what the model should read instead: a view with marks, or a drive with a note
        when it went nowhere."""
        if response.get("isError"):
            return response
        result = response.get("result")
        if tool_name == LOOK and isinstance(result, Mapping):
            self.saw(result, tick)
            return {**response, "result": self.mark_view(result)}
        drive = response.get("drive")
        if tool_name == DRIVE and isinstance(result, Mapping) and isinstance(drive, Mapping):
            destination_id = result.get("destination_id") or (args or {}).get("destination_id")
            path = result.get("path") if isinstance(result.get("path"), Mapping) else {}
            planned = float(path["length_m"]) if _is_number(path.get("length_m")) else None
            self.drove(str(destination_id) if destination_id else None, drive, tick, planned)
            if drive.get("outcome") == "succeeded" and planned is not None and planned < NULL_DRIVE_M and not drive.get("note"):
                return {**response, "drive": {**drive, "note": ALREADY_THERE_NOTE}}
        return response


def _is_number(value: Any) -> bool:
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)


def _distance_note(place: Place) -> str:
    return "" if place.distance_m is None else f" ({place.distance_m:g} m)"


def _failed_note(place: Place, failed_guard: bool) -> str:
    """With `[brain] failed_guard`, the summary says which places drives already failed to reach."""
    if not failed_guard or not place.failed:
        return ""
    return f", a drive there failed {place.failed}x"


def _kind(destination_id: str) -> str:
    return "passage" if destination_id.startswith("psg:") else "object"
