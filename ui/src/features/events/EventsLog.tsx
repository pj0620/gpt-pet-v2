import { useVirtualizer } from "@tanstack/react-virtual";
import { Activity, Eraser } from "lucide-react";
import { useEffect, useMemo, useRef } from "react";

import { PanelCard } from "@/components/PanelCard";
import { Button } from "@/components/ui/button";
import { EventRow } from "@/features/events/EventRow";
import { usePortalStore } from "@/stores/portalStore";

/** MCP tool calls (and agent text) as they happen; rows expand to the raw input/output. */
export function EventsLog() {
  const rows = usePortalStore((store) => store.events.rows);
  const selectedKey = usePortalStore((store) => store.selectedKey);
  const select = usePortalStore((store) => store.select);
  const toolFilter = usePortalStore((store) => store.toolFilter);
  const setToolFilter = usePortalStore((store) => store.setToolFilter);
  const clearEvents = usePortalStore((store) => store.clearEvents);
  const connection = usePortalStore((store) => store.connection);

  const toolNames = useMemo(() => {
    const names = new Set<string>();
    for (const row of rows) if (row.kind === "call") names.add(row.name);
    return [...names].sort();
  }, [rows]);

  const visible = useMemo(
    () => (toolFilter ? rows.filter((row) => row.kind !== "call" || row.name === toolFilter) : rows),
    [rows, toolFilter],
  );

  const parentRef = useRef<HTMLDivElement>(null);
  const virtualizer = useVirtualizer({
    count: visible.length,
    getScrollElement: () => parentRef.current,
    estimateSize: () => 32,
    overscan: 12,
    getItemKey: (index) => visible[index]?.key ?? index,
  });

  // Follow the log unless the user scrolled up or is inspecting a row.
  const lastCount = useRef(0);
  useEffect(() => {
    if (visible.length > lastCount.current && selectedKey === null) {
      virtualizer.scrollToIndex(visible.length - 1, { align: "end" });
    }
    lastCount.current = visible.length;
  }, [visible.length, selectedKey, virtualizer]);

  return (
    <PanelCard
      title="Events Log"
      icon={<Activity className="size-4" />}
      actions={
        <>
          <select
            aria-label="Filter by tool"
            className="h-7 rounded-md border border-input bg-background px-2 text-xs"
            value={toolFilter ?? ""}
            onChange={(event) => setToolFilter(event.target.value || null)}
          >
            <option value="">all tools</option>
            {toolNames.map((name) => (
              <option key={name} value={name}>
                {name}
              </option>
            ))}
          </select>
          <Button
            variant="ghost"
            size="icon-sm"
            aria-label="Clear events"
            onClick={clearEvents}
            disabled={rows.length === 0}
          >
            <Eraser />
          </Button>
        </>
      }
    >
      {visible.length === 0 ? (
        <p className="p-3 text-xs text-muted-foreground">
          {connection === "open" ? "Tool calls appear here as the pet acts." : "Waiting for the pet server."}
        </p>
      ) : (
        <div ref={parentRef} className="h-full overflow-auto">
          <div style={{ height: virtualizer.getTotalSize(), position: "relative", width: "100%" }}>
            {virtualizer.getVirtualItems().map((item) => {
              const row = visible[item.index];
              if (!row) return null;
              return (
                <div
                  key={row.key}
                  data-index={item.index}
                  ref={virtualizer.measureElement}
                  style={{
                    position: "absolute",
                    top: 0,
                    left: 0,
                    width: "100%",
                    transform: `translateY(${item.start}px)`,
                  }}
                >
                  <EventRow
                    row={row}
                    selected={selectedKey === row.key}
                    onToggle={() => select(selectedKey === row.key ? null : row.key)}
                  />
                </div>
              );
            })}
          </div>
        </div>
      )}
    </PanelCard>
  );
}
