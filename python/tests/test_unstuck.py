"""Offline: what the brain does about drives that keep failing (2026-10-02 session analysis: the
robot got wedged and drive after drive failed without it moving). The places memory counts such
drives and remembers where each failed; with `[brain] failed_guard` views mark `failed_here` and
the executor refuses those drives; with `[s1] unstuck_rule` System 1 backs up and turns around;
`[s1] explore_ticks` keeps an explore goal by rule. On ai2thor-mcp's real response shapes; the
ticks themselves run live (tests/live/test_unstuck_live.py)."""
from __future__ import annotations

from gpt_pet.agents.executor import FAILED_MARKER, build_executor, failed_refusal, is_budget_refusal
from gpt_pet.agents.goal_setter import FAILURE_SECTION, render_goal_setter_prompt
from gpt_pet.mcp import build_toolset
from gpt_pet.places import FAILED_HERE_NOTE, MARKS_NOTE, Places
from gpt_pet.s1 import BACK_UP_M, MOVE, STUCK_DRIVES, TURN, Step, SystemOne, fresh, policy_turn, rule_turn, unstuck_turn
from gpt_pet.settings import Settings, load_settings
from gpt_pet.stats import RunStats

SOFA = {"marker": 1, "id": "obj:Sofa|2", "kind": "object", "label": "Sofa", "description": "Sofa, 2.6 m ahead-left", "distance_m": 2.6}
DOOR = {"marker": 2, "id": "psg:4", "kind": "passage", "label": "doorway", "description": "passage: doorway ahead-right", "distance_m": 2.1}
LIMITS = {"get_current_view": 4, "set_nav_goal": 2, "do_rotate": 2, "do_move": 3}


def view(x: float, z: float, *destinations: dict) -> dict:
    return {"result": {"agent": {"x": x, "z": z, "yaw": 90.0}, "destinations": [dict(d) for d in destinations]}, "isError": False}


def drive(destination_id: str, outcome: str, traveled: float, at: tuple[float, float]) -> dict:
    final = {"state": outcome, "agent": {"x": at[0], "z": at[1], "yaw": 90.0}, "progress": {"distance_traveled_m": traveled}}
    result = {"goal_id": "g", "destination_id": destination_id, "label": "x", "state": "following", "path": {"waypoints": [], "length_m": 2.0}}
    return {"result": result, "isError": False, "drive": {"outcome": outcome, "waited_s": 1.0, "final_status": final}}


def stuck_places(*, guard: bool = True) -> Places:
    """Two drives failed from (1.0, 2.0) without the robot moving."""
    places = Places(failed_guard=guard)
    places.remember("get_current_view", {}, view(1.0, 2.0, SOFA, DOOR), tick=1)
    places.remember("set_nav_goal", {"destination_id": SOFA["id"]}, drive(SOFA["id"], "failed", 0.0, (1.0, 2.0)), tick=1)
    places.remember("set_nav_goal", {"destination_id": DOOR["id"]}, drive(DOOR["id"], "failed", 0.02, (1.02, 2.0)), tick=2)
    return places


def test_drives_failing_without_moving_count_as_stuck_until_the_robot_gets_away() -> None:
    places = stuck_places()
    assert places.stuck_drives == STUCK_DRIVES and places.stuck_at == (1.0, 2.0)
    places.remember("get_current_view", {}, view(1.1, 2.0, SOFA), tick=2)  # 10 cm: still stuck
    assert places.stuck_drives == 2
    places.remember("get_current_view", {}, view(1.0, 1.5, SOFA), tick=3)  # backed up half a metre
    assert (places.stuck_drives, places.stuck_at) == (0, None)
    again = stuck_places()
    again.remember("set_nav_goal", {}, drive(SOFA["id"], "failed", 1.4, (2.4, 2.0)), tick=3)  # failed, but it moved
    assert again.stuck_drives == 0
    again.remember("set_nav_goal", {}, drive(SOFA["id"], "failed", 0.0, (2.4, 2.0)), tick=4)
    assert again.stuck_drives == 1  # a new count, from the new spot


def test_a_view_marks_failed_here_only_with_the_guard_and_only_near_where_it_failed() -> None:
    places = stuck_places()
    marked = places.remember("get_current_view", {}, view(1.0, 2.0, SOFA, DOOR), tick=3)["result"]
    sofa = next(d for d in marked["destinations"] if d["id"] == SOFA["id"])
    assert sofa["failed"] == 1 and sofa["failed_here"] is True and marked["marks"] == MARKS_NOTE + FAILED_HERE_NOTE
    assert places.destinations[SOFA["id"]].failed_from == [(1.0, 2.0)]
    elsewhere = places.remember("get_current_view", {}, view(3.0, 2.0, SOFA), tick=4)["result"]
    assert "failed_here" not in elsewhere["destinations"][0] and elsewhere["destinations"][0]["failed"] == 1
    unguarded = stuck_places(guard=False)
    plain = unguarded.remember("get_current_view", {}, view(1.0, 2.0, SOFA), tick=3)["result"]
    assert "failed_here" not in plain["destinations"][0] and plain["marks"] == MARKS_NOTE  # as before


def test_the_executor_refuses_a_drive_that_failed_from_here_and_counts_it_as_a_refusal() -> None:
    places = stuck_places()
    refusal = failed_refusal(places, SOFA["id"])
    assert refusal.startswith(FAILED_MARKER) and "Sofa" in refusal
    assert is_budget_refusal({"error": refusal})
    places.remember("get_current_view", {}, view(3.0, 2.0, SOFA), tick=3)  # moved away: allowed again
    assert failed_refusal(places, SOFA["id"]) is None
    assert failed_refusal(stuck_places(guard=False), SOFA["id"]) is None


def test_the_goal_setter_hears_which_places_failed_and_sees_them_last() -> None:
    arch = {"marker": 3, "id": "psg:5", "kind": "passage", "label": "opening", "description": "passage: opening to the left", "distance_m": 3.0}
    places = stuck_places()
    places.remember("get_current_view", {}, view(1.0, 2.0, arch), tick=3)
    summary = "\n".join(places.summary())
    assert "- psg:4: passage: doorway ahead-right (2.1 m), a drive there failed 1x" in summary
    assert "- psg:5: passage: opening to the left (3 m)\n" in summary + "\n"
    assert [p.id for p in places.unvisited()] == [arch["id"], DOOR["id"], SOFA["id"]]  # the failed ones last
    plain = stuck_places(guard=False)
    plain.remember("get_current_view", {}, view(1.0, 2.0, arch), tick=3)
    assert "a drive there failed" not in "\n".join(plain.summary())
    assert [p.id for p in plain.unvisited()] == [arch["id"], DOOR["id"], SOFA["id"]]  # passages first, newest first


def test_system_1_skips_places_that_failed_from_here_and_their_targets() -> None:
    assert not fresh({**SOFA, "failed": 1, "failed_here": True})
    assert fresh({**SOFA, "failed": 1})
    places = stuck_places()
    goal = {"goal": "explore", "kind": "explore", "target_id": SOFA["id"]}
    marked = places.mark_view(view(1.0, 2.0, SOFA, DOOR)["result"])
    steps = [Step("get_current_view", {}, {"result": marked})]
    assert policy_turn(goal, steps, marked, LIMITS, places) is None  # both failed from here: nothing to drive to
    places.remember("get_current_view", {}, view(3.0, 2.0, SOFA, DOOR), tick=3)  # from elsewhere they are fine
    elsewhere = places.mark_view(view(3.0, 2.0, SOFA, DOOR)["result"])
    turn = policy_turn(goal, [Step("get_current_view", {}, {"result": elsewhere})], elsewhere, LIMITS, places)
    assert turn.args == {"destination_id": SOFA["id"]}  # the goal's target first


def test_when_stuck_system_1_backs_up_then_turns_around_then_goes_on() -> None:
    looked = Step("get_current_view", {}, {"result": {}})
    first = unstuck_turn([looked], LIMITS)
    assert (first.kind, first.tool, first.args) == ("call", MOVE, {"action": "MoveBack", "moveMagnitude": BACK_UP_M})
    backed = [looked, Step(MOVE, first.args, {"result": "Successfully moved 0.50 m back"})]
    assert rule_turn({"goal": "x"}, backed, LIMITS).tool == "get_current_view"  # looks again after moving
    second = unstuck_turn([*backed, looked], LIMITS)
    assert (second.tool, second.args["degrees"]) == (TURN, 180)
    turned = [*backed, looked, Step(TURN, second.args, {"result": "Successfully rotated"}), looked]
    assert unstuck_turn(turned, LIMITS) is None  # both done: back to the usual rules


def test_an_explore_goal_goes_on_by_rule_while_places_remain_and_the_robot_is_free() -> None:
    settings = load_settings("sim")
    s1 = settings.s1.model_copy(update={"enabled": True, "explore_ticks": 3})
    places = Places()
    places.remember("get_current_view", {}, view(1.0, 2.0, SOFA, DOOR), tick=1)
    system = SystemOne(settings.model_copy(update={"s1": s1}), RunStats(), places)
    explore = {"goal": "explore the hallway", "kind": "explore", "attempts": 1, "status": "active"}
    assert system._keeps_exploring(explore)
    assert not system._keeps_exploring({**explore, "attempts": 3})  # had its stretches
    assert not system._keeps_exploring({**explore, "kind": "visit"})
    stuck = stuck_places(guard=False)
    assert not SystemOne(settings.model_copy(update={"s1": s1}), RunStats(), stuck)._keeps_exploring(explore)
    off = settings.model_copy(update={"s1": settings.s1.model_copy(update={"explore_ticks": 0})})
    assert not SystemOne(off, RunStats(), places)._keeps_exploring(explore)  # 0: off


def test_the_prompts_carry_the_failure_guidance_only_when_switched_on(sim_settings: Settings) -> None:
    on = sim_settings.model_copy(update={"brain": sim_settings.brain.model_copy(update={"failed_guard": True, "failure_guidance": True})})
    toolset = build_toolset(on.mcp)
    places = Places(failed_guard=True)
    prompt = build_executor(on, toolset, places=places).instruction(None)
    assert "failed_here" in prompt and "do_move MoveBack 0.5" in prompt
    off = sim_settings.model_copy(update={"brain": sim_settings.brain.model_copy(update={"failed_guard": False, "failure_guidance": False})})
    plain = build_executor(off, toolset, places=Places()).instruction(None)
    assert "failed_here" not in plain and "MoveBack" not in plain
    assert FAILURE_SECTION in render_goal_setter_prompt({}, on, places)
    assert FAILURE_SECTION not in render_goal_setter_prompt({}, off, places)
