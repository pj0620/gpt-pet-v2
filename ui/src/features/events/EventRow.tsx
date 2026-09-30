import { CirclePause, Hourglass } from "lucide-react";

import { Badge } from "@/components/ui/badge";
import { EventDetail } from "@/features/events/EventDetail";
import { cn } from "@/lib/utils";
import type { EventRow as EventRowData } from "@/stores/eventsReducer";

interface EventRowProps {
  row: EventRowData;
  selected: boolean;
  onToggle: () => void;
}

function formatTime(timestamp: number): string {
  if (!timestamp) return "";
  return new Date(timestamp * 1000).toLocaleTimeString([], { hour12: false });
}

function s1Title(row: EventRowData): string {
  const s1 = row.s1;
  if (!s1) return "";
  if (s1.by === "rules") return "System 1 took this turn by rule; no model was called";
  const certainty = s1.certainty === undefined ? "" : ` (certainty ${s1.certainty.toFixed(2)})`;
  return `System 1 took this turn: ${s1.model ?? s1.by} decided${certainty} instead of Gemini`;
}

/** One log line, as in the mockup (`get_current_view [raw]`); click to expand the raw payload. */
export function EventRow({ row, selected, onToggle }: EventRowProps) {
  if (row.kind === "notice") {
    const Icon = row.name === "limit" ? CirclePause : Hourglass;
    return (
      <div
        className="flex items-center gap-2 border-b border-border/60 px-3 py-1.5 text-xs text-muted-foreground"
        data-testid="event-row"
        data-kind="notice"
        data-notice={row.name}
      >
        <span className="w-16 shrink-0 font-mono text-[10px]">{formatTime(row.timestamp)}</span>
        <Icon className="size-3.5 shrink-0" aria-hidden />
        <span className="min-w-0 flex-1 truncate" title={row.text}>
          {row.text}
        </span>
        <Badge variant="ghost" className="shrink-0 text-[10px]">
          tick {row.tick}
        </Badge>
      </div>
    );
  }

  if (row.kind === "tick") {
    return (
      <div
        className="flex items-center gap-2 px-3 py-1 text-[11px] text-muted-foreground"
        data-testid="event-row"
        data-kind="tick"
      >
        <span className="h-px flex-1 bg-border" />
        <span>
          {row.name} · {String((row.response as { seconds?: number } | undefined)?.seconds ?? "?")} s
        </span>
        <span className="h-px flex-1 bg-border" />
      </div>
    );
  }

  return (
    <div className="border-b border-border/60" data-testid="event-row" data-kind={row.kind}>
      <button
        type="button"
        onClick={onToggle}
        aria-expanded={selected}
        className={cn(
          "flex w-full items-center gap-2 px-3 py-1.5 text-left font-mono text-xs hover:bg-muted/50",
          selected && "bg-muted/60",
        )}
      >
        <span className="w-16 shrink-0 text-[10px] text-muted-foreground">{formatTime(row.timestamp)}</span>
        {row.kind === "text" ? (
          <span className="min-w-0 flex-1 truncate">
            <span className="text-muted-foreground">{row.author}: </span>
            {row.text}
          </span>
        ) : (
          <span className="min-w-0 flex-1 truncate">
            {row.name} <span className="text-muted-foreground">[raw]</span>
          </span>
        )}
        {row.s1 ? (
          <Badge variant="secondary" className="shrink-0 text-[10px]" title={s1Title(row)} data-testid="s1-badge">
            S1
          </Badge>
        ) : null}
        {row.kind === "call" && row.response === undefined ? <Badge variant="outline">pending</Badge> : null}
        {row.isError ? <Badge variant="destructive">error</Badge> : null}
        {row.durationMs !== undefined ? (
          <span className="shrink-0 text-[10px] text-muted-foreground">{row.durationMs} ms</span>
        ) : null}
        <Badge variant="ghost" className="shrink-0 text-[10px]">
          tick {row.tick}
        </Badge>
      </button>
      {selected ? <EventDetail row={row} /> : null}
    </div>
  );
}
