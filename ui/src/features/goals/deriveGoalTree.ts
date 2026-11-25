import type { Goal, PendingGoal, PetState } from "@/types/api";

export type GoalMarker = "active" | "pending" | "done" | "abandoned";

export interface GoalNode {
  key: string;
  label: string;
  /** null for sub-goals: the backend keeps no per-step status yet. */
  marker: GoalMarker | null;
  children: GoalNode[];
  meta?: {
    id?: number;
    attempts?: number;
    createdTick?: number;
    finishedTick?: number;
    successCriteria?: string;
  };
}

export const MARKER_TEXT: Record<GoalMarker, string> = {
  active: "[*]",
  pending: "[...]",
  done: "[done]",
  abandoned: "[abandoned]",
};

function subGoals(parentKey: string, steps: string[] | undefined): GoalNode[] {
  return (steps ?? []).map((step, i) => ({ key: `${parentKey}/sub:${i}`, label: step, marker: null, children: [] }));
}

function goalNode(goal: Goal, marker: GoalMarker): GoalNode {
  const key = `goal:${goal.id}`;
  return {
    key,
    label: goal.goal,
    marker,
    children: subGoals(key, goal.sub_goals),
    meta: {
      id: goal.id,
      attempts: goal.attempts,
      createdTick: goal.created_tick,
      finishedTick: goal.finished_tick,
      successCriteria: goal.success_criteria,
    },
  };
}

function pendingNode(goal: PendingGoal, i: number): GoalNode {
  const key = `pending:${goal.id ?? i}`;
  return { key, label: goal.goal, marker: "pending", children: subGoals(key, goal.sub_goals) };
}

/**
 * Pure: the Goals Queue as a tree. Order is the active goal, then pending goals submitted from
 * the portal, then finished goals newest first (the backend already stores history that way).
 */
export function deriveGoalTree(state: PetState | undefined): GoalNode[] {
  if (!state) return [];
  const nodes: GoalNode[] = [];
  if (state.current_goal) nodes.push(goalNode(state.current_goal, "active"));
  (state.pending_goals ?? []).forEach((goal, i) => {
    nodes.push(pendingNode(goal, i));
  });
  for (const goal of state.goal_history) nodes.push(goalNode(goal, goal.status === "done" ? "done" : "abandoned"));
  return nodes;
}
