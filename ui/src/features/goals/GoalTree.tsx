import { type GoalNode, MARKER_TEXT } from "@/features/goals/deriveGoalTree";
import { SpinnerMarker } from "@/features/goals/SpinnerMarker";
import { cn } from "@/lib/utils";

const MARKER_CLASS: Record<NonNullable<GoalNode["marker"]>, string> = {
  active: "text-emerald-400",
  pending: "text-sky-400",
  done: "text-muted-foreground",
  abandoned: "text-amber-500",
};

/** How the active goal's marker is drawn: spinning while the pet works, `[paused]` while it waits. */
export type ActiveMarkerMode = "spinning" | "still" | "paused";

function ActiveMarker({ mode }: { mode: ActiveMarkerMode }) {
  if (mode === "paused") {
    return (
      <span data-testid="goal-paused-marker" className="shrink-0 text-yellow-400">
        [paused]
      </span>
    );
  }
  return <SpinnerMarker running={mode === "spinning"} className={cn("shrink-0", MARKER_CLASS.active)} />;
}

function GoalRow({ node, depth, activeMode }: { node: GoalNode; depth: number; activeMode: ActiveMarkerMode }) {
  const marker = node.marker;
  return (
    <li>
      <div
        className={cn("flex items-baseline gap-2 py-0.5 font-mono text-xs", depth > 0 && "text-muted-foreground")}
        style={{ paddingLeft: `${depth * 1.25}rem` }}
        data-testid={marker ? `goal-${marker}` : "goal-step"}
        title={node.meta?.successCriteria}
      >
        {marker === "active" ? (
          <ActiveMarker mode={activeMode} />
        ) : (
          <span className={cn("shrink-0", marker ? MARKER_CLASS[marker] : "text-muted-foreground/60")}>
            {marker ? MARKER_TEXT[marker] : "-"}
          </span>
        )}
        <span className={cn("break-words", marker === "done" && "line-through decoration-muted-foreground/60")}>
          {node.label}
        </span>
        {node.meta && node.meta.attempts !== undefined && node.meta.attempts > 0 ? (
          <span className="shrink-0 text-[10px] text-muted-foreground">attempt {node.meta.attempts + 1}</span>
        ) : null}
      </div>
      {node.children.length > 0 ? (
        <ul>
          {node.children.map((child) => (
            <GoalRow key={child.key} node={child} depth={depth + 1} activeMode={activeMode} />
          ))}
        </ul>
      ) : null}
    </li>
  );
}

/** The goals tree: a spinning `[|]` (or `[paused]`) for the goal in progress, `[...]` pending,
 * `[done]`, with nested sub-goals. */
export function GoalTree({ nodes, activeMode = "spinning" }: { nodes: GoalNode[]; activeMode?: ActiveMarkerMode }) {
  return (
    <ul className="flex flex-col gap-0.5">
      {nodes.map((node) => (
        <GoalRow key={node.key} node={node} depth={0} activeMode={activeMode} />
      ))}
    </ul>
  );
}
