import { useExtendGoals, useStatusQuery } from "@/api/queries";
import { Button } from "@/components/ui/button";
import { usePortalStore } from "@/stores/portalStore";

const STEPS = [1, 2, 5, 10];

/** "goals used/limit" plus buttons that allow N more goals this run (and un-pause a limited loop). */
export function GoalBudget() {
  const connection = usePortalStore((store) => store.connection);
  const status = useStatusQuery();
  const extend = useExtendGoals();
  const limit = status.data?.goal_limit ?? null;
  const used = status.data?.goals_used ?? 0;
  const disabled = connection !== "open" || !status.data || limit === null || extend.isPending;

  return (
    <div className="flex items-center gap-1 text-xs text-muted-foreground">
      <span
        data-testid="goal-budget"
        className="mr-1 font-mono"
        title="goals started this run / goals allowed for the pet's own goals (custom goals always run)"
      >
        {status.data ? (limit === null ? `goals ${used}` : `goals ${used}/${limit}`) : "goals -/-"}
      </span>
      <span className="sr-only">Add more goals</span>
      {STEPS.map((step) => (
        <Button
          key={step}
          size="xs"
          variant="outline"
          disabled={disabled}
          aria-label={`Add ${step} more goal${step === 1 ? "" : "s"}`}
          title={
            limit === null ? "This run has no goal limit" : `Allow ${step} more goal${step === 1 ? "" : "s"} this run`
          }
          onClick={() => extend.mutate(step)}
        >
          +{step}
        </Button>
      ))}
    </div>
  );
}
