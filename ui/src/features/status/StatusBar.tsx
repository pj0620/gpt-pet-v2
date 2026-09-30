import { Wifi, WifiOff } from "lucide-react";

import { useStatusQuery } from "@/api/queries";
import { Badge } from "@/components/ui/badge";
import { type ConnectionState, usePortalStore } from "@/stores/portalStore";

const CONNECTION_LABEL: Record<ConnectionState, string> = {
  open: "Live",
  connecting: "Connecting",
  closed: "Offline",
};

export function StatusBar() {
  const connection = usePortalStore((store) => store.connection);
  const lastTick = usePortalStore((store) => store.lastTick);
  const status = useStatusQuery();
  const online = connection === "open";

  return (
    <div className="flex items-center gap-2 text-xs text-muted-foreground">
      {status.data ? (
        <>
          <Badge variant="outline">{status.data.profile}</Badge>
          <span>tick {status.data.tick}</span>
          {status.data.goal_limit_reached ? (
            <Badge variant="secondary" title="max_goals_per_run reached; Resume grants another window">
              goal limit reached
            </Badge>
          ) : status.data.paused ? (
            <Badge variant="secondary">paused</Badge>
          ) : null}
          {status.data.running_tick ? <span className="animate-pulse">thinking…</span> : null}
          {!status.data.running_tick && status.data.waiting_s ? (
            <span>next tick in {Math.ceil(status.data.waiting_s)} s</span>
          ) : null}
        </>
      ) : null}
      {lastTick ? <span>last tick {lastTick.seconds.toFixed(1)} s</span> : null}
      <Badge
        variant={online ? "default" : "outline"}
        data-testid="connection-state"
        className="gap-1"
        title={online ? "Connected to the pet server" : "No pet server on /api (start `gpt-pet serve`)"}
      >
        {online ? <Wifi /> : <WifiOff />}
        {CONNECTION_LABEL[connection]}
      </Badge>
    </div>
  );
}
