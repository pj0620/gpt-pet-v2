import { ListTodo } from "lucide-react";

import { useStateQuery, useStatusQuery } from "@/api/queries";
import { PanelCard } from "@/components/PanelCard";
import { ScrollArea } from "@/components/ui/scroll-area";
import { CustomGoalForm } from "@/features/goals/CustomGoalForm";
import { deriveGoalTree } from "@/features/goals/deriveGoalTree";
import { GoalBudget } from "@/features/goals/GoalBudget";
import { type ActiveMarkerMode, GoalTree } from "@/features/goals/GoalTree";
import { usePortalStore } from "@/stores/portalStore";

export function GoalsQueue() {
  const connection = usePortalStore((store) => store.connection);
  const state = useStateQuery();
  const status = useStatusQuery();
  const tree = deriveGoalTree(state.data);
  const activeMode: ActiveMarkerMode =
    connection !== "open"
      ? "still"
      : status.data?.running_tick
        ? "spinning"
        : status.data?.paused
          ? "paused"
          : "spinning";

  return (
    <PanelCard title="Goals Queue" icon={<ListTodo className="size-4" />} actions={<GoalBudget />}>
      <div className="flex h-full flex-col">
        <ScrollArea className="min-h-0 flex-1">
          <div className="p-3">
            {tree.length > 0 ? (
              <GoalTree nodes={tree} activeMode={activeMode} />
            ) : (
              <p className="text-xs text-muted-foreground">
                {connection === "open"
                  ? "No goals yet. The pet decides one on its first tick."
                  : "Waiting for the pet server."}
              </p>
            )}
            {state.data?.last_report ? (
              <div className="mt-4 rounded-md bg-muted/40 p-2 text-xs text-muted-foreground">
                <div className="mb-1 font-medium text-foreground">Last report</div>
                <p className="whitespace-pre-wrap">{state.data.last_report}</p>
              </div>
            ) : null}
          </div>
        </ScrollArea>
        <div className="shrink-0 border-t border-border p-3">
          <CustomGoalForm />
        </div>
      </div>
    </PanelCard>
  );
}
