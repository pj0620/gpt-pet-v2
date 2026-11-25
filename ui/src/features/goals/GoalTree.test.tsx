import { render, screen } from "@testing-library/react";
import { describe, expect, it } from "vitest";
import type { GoalNode } from "@/features/goals/deriveGoalTree";
import { GoalTree } from "@/features/goals/GoalTree";

const nodes: GoalNode[] = [
  {
    key: "goal:2",
    label: "get a close look at the sofa",
    marker: "active",
    children: [{ key: "goal:2/sub:0", label: "navigate to the sofa", marker: null, children: [] }],
  },
  { key: "pending:3", label: "find the owner", marker: "pending", children: [] },
  { key: "goal:1", label: "explore", marker: "done", children: [] },
];

describe("GoalTree", () => {
  it("spins on the active goal while the pet works", () => {
    render(<GoalTree nodes={nodes} activeMode="spinning" />);
    expect(screen.getByTestId("goal-spinner")).toHaveAttribute("aria-label", "in progress");
    expect(screen.queryByTestId("goal-paused-marker")).toBeNull();
    expect(screen.getByTestId("goal-pending").textContent).toContain("[...]");
    expect(screen.getByTestId("goal-done").textContent).toContain("[done]");
    expect(screen.getByTestId("goal-step").textContent).toContain("navigate to the sofa");
  });

  it("shows [paused] on the goal that was running once the pet is paused", () => {
    render(<GoalTree nodes={nodes} activeMode="paused" />);
    const row = screen.getByTestId("goal-active");
    expect(row.textContent).toContain("[paused]");
    expect(row.textContent).toContain("get a close look at the sofa");
    expect(screen.queryByTestId("goal-spinner")).toBeNull();
  });
});
