import { Gauge, RotateCcw } from "lucide-react";

import { useResetStats, useStatsQuery } from "@/api/queries";
import { PanelCard } from "@/components/PanelCard";
import { Button } from "@/components/ui/button";
import { formatSeconds } from "@/features/stats/format";
import { StatsGrid } from "@/features/stats/StatsGrid";
import { usePortalStore } from "@/stores/portalStore";

/** Live run counters: the server pushes a `stats` frame whenever they move (and every 5 s). */
export function StatsPanel() {
  const stats = useStatsQuery().data ?? null;
  const reset = useResetStats();
  const connection = usePortalStore((store) => store.connection);

  return (
    <PanelCard
      title="Stats"
      icon={<Gauge className="size-4" />}
      actions={
        <>
          {stats ? (
            <span
              className="font-mono text-xs text-muted-foreground"
              data-testid="stats-uptime"
              title="Time since the counters started"
            >
              up {formatSeconds(stats.uptime_s)}
            </span>
          ) : null}
          <Button
            variant="ghost"
            size="icon-sm"
            aria-label="Reset stats"
            title="Start the counters over"
            onClick={() => reset.mutate()}
            disabled={!stats || connection !== "open" || reset.isPending}
          >
            <RotateCcw />
          </Button>
        </>
      }
    >
      {stats ? (
        <div className="h-full overflow-auto">
          <StatsGrid stats={stats} />
        </div>
      ) : (
        <p className="p-3 text-xs text-muted-foreground">
          {connection === "open"
            ? "Waiting for the first counters."
            : "Live stats appear here once the pet server runs."}
        </p>
      )}
    </PanelCard>
  );
}
