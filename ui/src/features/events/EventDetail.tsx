import { darkStyles, JsonView } from "react-json-view-lite";

import type { EventRow } from "@/stores/eventsReducer";

const expandTwoLevels = (level: number) => level < 2;

/** The raw MCP tool call input and output, or an agent's text. */
export function EventDetail({ row }: { row: EventRow }) {
  const data =
    row.kind === "text"
      ? { author: row.author, text: row.text }
      : { name: row.name, args: row.args ?? {}, response: row.response ?? null };
  return (
    <div className="border-t border-border/60 bg-black/30 p-2 text-xs" data-testid="event-detail">
      <JsonView data={data} style={darkStyles} shouldExpandNode={expandTwoLevels} />
    </div>
  );
}
