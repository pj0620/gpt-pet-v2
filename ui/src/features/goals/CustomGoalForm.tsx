import { type FormEvent, useState } from "react";

import { useSubmitGoal } from "@/api/queries";
import { Button } from "@/components/ui/button";
import { Textarea } from "@/components/ui/textarea";
import { usePortalStore } from "@/stores/portalStore";

/** "Enter Custom Goal": queues a goal the goal setter adopts on its next tick. */
export function CustomGoalForm() {
  const connection = usePortalStore((store) => store.connection);
  const [goal, setGoal] = useState("");
  const submit = useSubmitGoal();
  const canSubmit = connection === "open" && goal.trim().length > 0 && !submit.isPending;

  const onSubmit = (event: FormEvent<HTMLFormElement>) => {
    event.preventDefault();
    if (!canSubmit) return;
    submit.mutate({ goal: goal.trim() }, { onSuccess: () => setGoal("") });
  };

  return (
    <form onSubmit={onSubmit} className="flex flex-col gap-2">
      <label htmlFor="custom-goal" className="text-xs font-medium">
        Enter Custom Goal
      </label>
      <Textarea
        id="custom-goal"
        aria-label="Custom goal"
        placeholder={
          connection === "open" ? "e.g. find the owner and say hi" : "Available when the pet server is online"
        }
        value={goal}
        onChange={(event) => setGoal(event.target.value)}
        disabled={connection !== "open"}
        rows={2}
        className="resize-none text-xs"
      />
      <div className="flex items-center justify-between gap-2">
        <span className="text-xs text-muted-foreground">
          {submit.isPending ? "Submitting…" : submit.error ? `Failed: ${submit.error.message}` : ""}
        </span>
        <Button type="submit" size="sm" disabled={!canSubmit}>
          Submit
        </Button>
      </div>
    </form>
  );
}
