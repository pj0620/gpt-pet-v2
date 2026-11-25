"""Offline: the pet's memory of places, on the shapes the executor's after-tool callback hands it
(ai2thor-mcp's real `get_current_view` and waited-out `set_nav_goal` responses). The callback
itself runs live (tests/live/test_places_live.py)."""
from __future__ import annotations

from gpt_pet.places import ALREADY_THERE_NOTE, ARRIVAL_M, HERE_M, MARKS_NOTE, PASSAGE_HERE_M, TRAIL_STEP_M, Places

TABLE = {"marker": 1, "id": "obj:DiningTable|1", "kind": "object", "label": "DiningTable", "description": "DiningTable, 0.9 m ahead", "distance_m": 0.9}
SOFA = {"marker": 2, "id": "obj:Sofa|2", "kind": "object", "label": "Sofa", "description": "Sofa, 2.6 m ahead-left", "distance_m": 2.6}
DOOR = {"marker": 3, "id": "psg:4", "kind": "passage", "label": "doorway", "description": "passage: doorway ahead-right, ~0.9 m wide", "distance_m": 1.1}
ARCH = {"marker": 4, "id": "psg:5", "kind": "passage", "label": "opening", "description": "passage: opening to the left, ~1.4 m wide", "distance_m": 0.4}


def view(x: float, z: float, yaw: float, *destinations: dict) -> dict:
    """A `get_current_view` response after the media callback: the JSON result, image parts aside."""
    result = {"agent": {"x": x, "z": z, "yaw": yaw}, "destinations": [dict(d) for d in destinations], "note": "Pass a destination id to set_nav_goal to drive there."}
    return {"result": result, "isError": False}


def drive(destination_id: str, outcome: str, planned_m: float, at: tuple[float, float], note: str | None = None) -> dict:
    """A `set_nav_goal` response once `gpt_pet.navwait` waited the drive out."""
    final = {"state": outcome, "agent": {"x": at[0], "z": at[1], "yaw": 90.0}, "progress": {"distance_traveled_m": planned_m}}
    block = {"outcome": outcome, "waited_s": 5.2, "final_status": final}
    if note:
        block["note"] = note
    result = {"goal_id": "g1", "destination_id": destination_id, "label": "x", "state": "following", "path": {"waypoints": [], "length_m": planned_m}}
    return {"result": result, "isError": False, "drive": block}


def marks_of(response: dict) -> dict[str, dict]:
    return {d["id"]: {k: d[k] for k in ("here", "visited", "failed") if k in d} for d in response["result"]["destinations"]}


def test_a_view_marks_what_is_within_reach_and_says_what_marks_mean() -> None:
    places = Places()
    marked = places.remember("get_current_view", {}, view(1.0, 2.0, 90.0, TABLE, SOFA, DOOR, ARCH), tick=1)
    assert TABLE["distance_m"] <= HERE_M < SOFA["distance_m"] and ARCH["distance_m"] <= PASSAGE_HERE_M < DOOR["distance_m"]
    assert marks_of(marked) == {TABLE["id"]: {"here": True}, SOFA["id"]: {}, DOOR["id"]: {}, ARCH["id"]: {"here": True}}
    assert marked["result"]["marks"] == MARKS_NOTE and marked["result"]["note"].startswith("Pass a destination id")
    assert places.pose == {"x": 1.0, "z": 2.0, "yaw": 90.0} and places.trail == [(1.0, 2.0)]
    assert set(places.destinations) == {TABLE["id"], SOFA["id"], DOOR["id"], ARCH["id"]}
    assert places.destinations[SOFA["id"]].seen == 1 and places.destinations[DOOR["id"]].kind == "passage"
    nothing_close = places.remember("get_current_view", {}, view(1.0, 2.0, 180.0, SOFA), tick=1)
    assert "marks" not in nothing_close["result"]  # no mark, no note


def test_a_reached_place_is_here_while_the_robot_stays_and_visited_once_it_left() -> None:
    places = Places()
    places.remember("get_current_view", {}, view(1.0, 2.0, 90.0, SOFA), tick=1)
    arrived = places.remember("set_nav_goal", {"destination_id": SOFA["id"]}, drive(SOFA["id"], "succeeded", 2.4, (3.0, 2.5)), tick=1)
    assert "note" not in arrived["drive"]
    sofa = places.destinations[SOFA["id"]]
    assert (sofa.drives, sofa.reached_tick, sofa.reached_at) == (1, 1, (3.0, 2.5))
    # a big sofa's centre can stay out of HERE_M even from its closest reachable point
    from_there = places.remember("get_current_view", {}, view(3.0 + ARRIVAL_M / 2, 2.5, 0.0, {**SOFA, "distance_m": 1.6}), tick=2)
    assert marks_of(from_there)[SOFA["id"]] == {"here": True}
    elsewhere = places.remember("get_current_view", {}, view(6.0, 2.5, 270.0, {**SOFA, "distance_m": 3.0}), tick=3)
    assert marks_of(elsewhere)[SOFA["id"]] == {"visited": True}
    assert places.trail == [(1.0, 2.0), (3.0, 2.5), (6.0, 2.5)]


def test_a_drive_that_went_nowhere_says_so_and_failures_are_counted() -> None:
    places = Places()
    places.remember("get_current_view", {}, view(1.0, 2.0, 90.0, TABLE, SOFA), tick=1)
    nowhere = places.remember("set_nav_goal", {"destination_id": TABLE["id"]}, drive(TABLE["id"], "succeeded", 0.12, (1.0, 2.0)), tick=1)
    assert nowhere["drive"]["note"] == ALREADY_THERE_NOTE
    timed_out = drive(SOFA["id"], "timeout", 0.1, (1.0, 2.0), note="Still driving after 120 s; stopped waiting.")
    assert places.remember("set_nav_goal", {}, timed_out, tick=1)["drive"]["note"].startswith("Still driving")
    for tick in (2, 3):
        places.remember("set_nav_goal", {}, drive(SOFA["id"], "failed", 2.4, (1.0, 2.0)), tick=tick)
    again = places.remember("get_current_view", {}, view(1.0, 2.0, 90.0, SOFA), tick=4)
    assert marks_of(again)[SOFA["id"]] == {"failed": 2}
    assert places.destinations[SOFA["id"]].drives == 3


def test_drives_teach_how_far_away_a_destination_and_its_kind_count_as_reached() -> None:
    # baseline run 1: a drive to the sofa from 2.5 m planned 0.0 m: a sofa is reached from its edge
    places = Places()
    big_sofa = {**SOFA, "distance_m": 2.5}
    places.remember("get_current_view", {}, view(1.0, 2.0, 90.0, big_sofa), tick=1)
    assert marks_of(places.remember("get_current_view", {}, view(1.0, 2.0, 90.0, big_sofa), tick=1))[SOFA["id"]] == {}
    places.remember("set_nav_goal", {}, drive(SOFA["id"], "succeeded", 0.0, (1.0, 2.0)), tick=1)
    assert places.destinations[SOFA["id"]].reach_m == 2.5 and places.label_reach == {"Sofa": 2.5}
    elsewhere = places.remember("get_current_view", {}, view(4.0, 2.0, 270.0, {**SOFA, "distance_m": 2.4}), tick=2)
    assert marks_of(elsewhere)[SOFA["id"]] == {"here": True}  # within its learned reach, though not where the drive ended
    other_sofa = {**SOFA, "id": "obj:Sofa|9", "marker": 5, "distance_m": 2.3}
    far_sofa = {**SOFA, "id": "obj:Sofa|10", "marker": 6, "distance_m": 2.7}
    both = places.remember("get_current_view", {}, view(4.0, 2.0, 270.0, other_sofa, far_sofa), tick=2)
    assert marks_of(both) == {other_sofa["id"]: {"here": True}, far_sofa["id"]: {}}  # the label's reach carries over


def test_a_distance_seen_from_elsewhere_teaches_nothing() -> None:
    places = Places()
    places.remember("get_current_view", {}, view(1.0, 2.0, 90.0, {**SOFA, "distance_m": 2.5}), tick=1)
    places.remember("get_current_view", {}, view(2.0, 2.0, 0.0), tick=1)  # moved on: the sofa's distance is stale
    places.remember("set_nav_goal", {}, drive(SOFA["id"], "succeeded", 0.3, (2.0, 2.0)), tick=1)
    assert places.destinations[SOFA["id"]].reach_m is None and not places.label_reach
    door = {**DOOR, "distance_m": 2.0}
    places.remember("get_current_view", {}, view(2.0, 2.0, 0.0, door), tick=2)
    places.remember("set_nav_goal", {}, drive(DOOR["id"], "succeeded", 0.1, (2.0, 2.0)), tick=2)
    assert places.destinations[DOOR["id"]].reach_m is None  # passages keep their short radius


def test_errors_and_other_tools_pass_through_untouched() -> None:
    places = Places()
    error = {"result": "Unknown destination id 'obj:Ghost|1'.", "isError": True}
    assert places.remember("set_nav_goal", {"destination_id": "obj:Ghost|1"}, error, tick=1) is error
    rotated = {"result": "Successfully rotated", "isError": False}
    assert places.remember("do_rotate", {"action": "RotateLeft"}, rotated, tick=1) is rotated
    assert places.pose is None and not places.destinations


def test_the_trail_grows_only_with_real_moves_and_looks_record_headings() -> None:
    places = Places()
    for tick, (x, z, yaw) in enumerate([(0.0, 0.0, 0.0), (0.1, 0.0, 90.0), (TRAIL_STEP_M, 0.0, 180.0), (TRAIL_STEP_M, 0.0, 270.0)], 1):
        places.remember("get_current_view", {}, view(x, z, yaw), tick=tick)
    assert places.trail == [(0.0, 0.0), (TRAIL_STEP_M, 0.0)]
    assert places.looks[(0, 0)] == {0, 1} and places.looks[(1, 0)] == {2, 3}
    assert places.ticks == {1, 2, 3, 4}


def test_the_summary_tells_the_goal_setter_what_was_done_and_where_it_has_not_been() -> None:
    places = Places(started=0.0)
    places.remember("get_current_view", {}, view(1.0, 2.0, 90.0, TABLE, SOFA, DOOR), tick=1)
    places.remember("set_nav_goal", {}, drive(TABLE["id"], "succeeded", 0.1, (1.0, 2.0)), tick=1)
    places.remember("set_nav_goal", {}, drive(SOFA["id"], "succeeded", 2.4, (3.0, 2.5)), tick=2)
    places.remember("get_current_view", {}, view(3.0, 2.5, 0.0, ARCH), tick=2)
    lines = places.summary(now=180.0)
    assert lines[0] == "Time: 3 min over 2 ticks. Drives: 2 (1 already there, 1 succeeded), 2.5 m driven."
    assert lines[1] == "Looked around from 2 spots."
    # the arch was seen within reach (0.4 m): the pet has been there, so it is no target
    assert lines[2] == "Visited: DiningTable (obj:DiningTable|1), Sofa (obj:Sofa|2), opening (psg:5)."
    assert lines[3].startswith("Seen but not visited yet, passages first")
    assert lines[4:] == ["- psg:4: passage: doorway ahead-right, ~0.9 m wide (1.1 m)"]
    assert Places(started=0.0).summary(now=0.0)[-1].startswith("Seen but not visited yet: nothing")


def test_a_place_seen_within_reach_counts_as_visited_and_is_never_offered_as_a_target() -> None:
    # exploration run s1_off_p2-1: Gemini targeted a baseball bat 1.8 m away, the pet "drove" 0.25 m
    places = Places(here_m=2.0)
    bat = {"marker": 1, "id": "obj:BaseballBat|1", "kind": "object", "label": "BaseballBat", "description": "BaseballBat, 1.8 m right", "distance_m": 1.8}
    places.remember("get_current_view", {}, view(1.0, 2.0, 90.0, bat, SOFA), tick=1)
    assert places.destinations[bat["id"]].reached_tick == 1 and places.destinations[bat["id"]].reached_at == (1.0, 2.0)
    assert [p.id for p in places.unvisited()] == [SOFA["id"]]
    assert not any(bat["id"] in line for line in places.summary(now=0.0) if line.startswith("- "))
    later = places.remember("get_current_view", {}, view(5.0, 2.0, 270.0, {**bat, "distance_m": 4.0}), tick=2)
    assert marks_of(later)[bat["id"]] == {"visited": True}


def test_is_here_judges_from_where_the_robot_stands_now() -> None:
    places = Places(here_m=2.0)
    plate = {"marker": 1, "id": "obj:Plate|1", "kind": "object", "label": "Plate", "description": "Plate, 1.6 m ahead", "distance_m": 1.6}
    places.remember("get_current_view", {}, view(1.0, 2.0, 90.0, plate, SOFA), tick=1)
    assert places.is_here(plate["id"]) and not places.is_here(SOFA["id"]) and not places.is_here("obj:Ghost|9")
    places.at({"x": 4.0, "z": 2.0, "yaw": 0.0}, tick=2)  # drove off: the plate's distance is stale now
    assert not places.is_here(plate["id"])
    places.at({"x": 1.1, "z": 2.0, "yaw": 0.0}, tick=3)  # back where it was within reach
    assert places.is_here(plate["id"])


def test_with_people_on_a_drive_a_person_blocked_does_not_count_against_the_place() -> None:
    reason = "blocked by Person near (6.43, -3.25): they stayed in the way for 10 s and there was no way around them"

    def blocked() -> dict:
        response = drive(SOFA["id"], "failed", 0.0, (1.0, 2.0))
        response["drive"]["final_status"]["reason"] = reason
        return response

    with_people = Places(people_labels=frozenset({"person"}))
    without = Places()
    for places in (with_people, without):
        places.remember("get_current_view", {}, view(1.0, 2.0, 90.0, SOFA), tick=1)
        for tick in (2, 3):
            places.remember("set_nav_goal", {}, blocked(), tick=tick)
    assert with_people.destinations[SOFA["id"]].failed == 0 and with_people.stuck_drives == 0  # the person moves on
    assert with_people.drive_outcomes["failed"] == 2  # still counted as drives that failed
    assert without.destinations[SOFA["id"]].failed == 2 and without.stuck_drives == 2  # [people] off: as before
