import { Pause, Play, StepForward } from "lucide-react";

import { useControl, useSetProfile, useStatusQuery } from "@/api/queries";
import { Button } from "@/components/ui/button";
import { usePortalStore } from "@/stores/portalStore";
import type { ProfileName } from "@/types/api";

const PROFILES: ProfileName[] = ["sim", "real"];

/** Pause/resume the tick loop, step one tick, and switch the MCP profile. */
export function Controls() {
  const connection = usePortalStore((store) => store.connection);
  const status = useStatusQuery();
  const control = useControl();
  const setProfile = useSetProfile();
  const disabled = connection !== "open" || !status.data || control.isPending || setProfile.isPending;
  const paused = status.data?.paused ?? false;
  const limitReached = status.data?.goal_limit_reached ?? false;

  return (
    <div className="flex flex-col gap-3">
      <h3 className="text-sm font-medium">Controls</h3>
      <div className="flex flex-wrap gap-2">
        <Button
          variant="outline"
          size="sm"
          disabled={disabled || (paused && limitReached)}
          title={paused && limitReached ? "Goal limit reached: add more goals in the Goals Queue header" : undefined}
          onClick={() => control.mutate(paused ? "resume" : "pause")}
        >
          {paused ? <Play /> : <Pause />}
          {paused ? "Resume" : "Pause"}
        </Button>
        <Button variant="outline" size="sm" disabled={disabled || !paused} onClick={() => control.mutate("tick")}>
          <StepForward />
          Run one tick
        </Button>
      </div>
      <div className="flex items-center gap-2">
        <span className="text-xs text-muted-foreground">Profile</span>
        {PROFILES.map((name) => (
          <Button
            key={name}
            size="sm"
            variant={status.data?.profile === name ? "default" : "outline"}
            disabled={disabled || status.data?.profile === name}
            onClick={() => setProfile.mutate(name)}
          >
            {name}
          </Button>
        ))}
      </div>
      {control.error || setProfile.error ? (
        <p className="text-xs text-destructive">{String((control.error ?? setProfile.error)?.message)}</p>
      ) : null}
      {limitReached ? (
        <p className="text-xs text-muted-foreground">
          Goal limit reached. Use the +1 / +2 / +5 / +10 buttons in the Goals Queue header to allow more, or submit a
          custom goal: custom goals run regardless of the limit.
        </p>
      ) : null}
      {connection !== "open" ? <p className="text-xs text-muted-foreground">Controls need the pet server.</p> : null}
    </div>
  );
}
