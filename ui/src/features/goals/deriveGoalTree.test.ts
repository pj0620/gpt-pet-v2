import { describe, expect, it } from "vitest";

import { deriveGoalTree, MARKER_TEXT } from "@/features/goals/deriveGoalTree";
import type { PetState } from "@/types/api";

/** Shape copied from a real `gpt-pet run` session (python/src/gpt_pet/goals.py output). */
const state: PetState = {
  tick: 2,
  next_goal_id: 3,
  current_goal: {
    id: 2,
    goal: "go through the opening to explore the next room",
    success_criteria: "I have successfully navigated through the opening and scanned a new room.",
    sub_goals: [
      "navigate to the opening",
      "move through the opening into the next room",
      "spin in a circle to scan the new room",
    ],
    status: "active",
    created_tick: 2,
    attempts: 0,
  },
  goal_history: [
    {
      id: 1,
      goal: "get a close look at the fridge",
      success_criteria: "The fridge fills most of the view.",
      sub_goals: [],
      status: "done",
      created_tick: 1,
      attempts: 0,
      finished_tick: 2,
    },
  ],
  pending_goals: [{ id: 7, goal: "find the owner", sub_goals: ["look in the living room"], source: "portal" }],
  last_report: "Reached the fridge.",
  goal_decision: {
    previous_goal_status: "done",
    goal: "go through the opening to explore the next room",
    reasoning: "fridge seen",
  },
} as PetState;

describe("deriveGoalTree", () => {
  it("returns nothing for missing state", () => {
    expect(deriveGoalTree(undefined)).toEqual([]);
  });

  it("orders active, pending, then finished goals with the mockup markers", () => {
    const tree = deriveGoalTree(state);
    expect(tree.map((node) => [node.marker, node.label])).toEqual([
      ["active", "go through the opening to explore the next room"],
      ["pending", "find the owner"],
      ["done", "get a close look at the fridge"],
    ]);
    expect(tree.map((node) => (node.marker ? MARKER_TEXT[node.marker] : ""))).toEqual(["[*]", "[...]", "[done]"]);
  });

  it("nests sub-goals under their goal without a status marker", () => {
    const [active, pending] = deriveGoalTree(state);
    expect(active?.children.map((child) => child.label)).toEqual([
      "navigate to the opening",
      "move through the opening into the next room",
      "spin in a circle to scan the new room",
    ]);
    expect(active?.children.every((child) => child.marker === null)).toBe(true);
    expect(pending?.children).toHaveLength(1);
    expect(new Set(deriveGoalTree(state).flatMap((n) => [n.key, ...n.children.map((c) => c.key)])).size).toBe(7);
  });

  it("marks abandoned history entries and copies goal metadata", () => {
    const abandoned: PetState = {
      ...state,
      current_goal: null,
      pending_goals: [],
      goal_history: state.goal_history.map((goal) => ({ ...goal, status: "abandoned" as const, attempts: 3 })),
    };
    const [node] = deriveGoalTree(abandoned);
    expect(node?.marker).toBe("abandoned");
    expect(node?.meta).toMatchObject({ id: 1, attempts: 3, createdTick: 1, finishedTick: 2 });
  });
});
