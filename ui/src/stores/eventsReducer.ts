import type { AdkEvent, NoticeEvent, S1Tag, TickEvent } from "@/types/api";

export type EventRowKind = "call" | "text" | "tick" | "notice";

/** One row of the Events Log. Calls and their responses are paired by the function call id. */
export interface EventRow {
  key: string;
  kind: EventRowKind;
  callId: string | null;
  tick: number;
  author: string;
  timestamp: number;
  name: string;
  args?: Record<string, unknown>;
  response?: unknown;
  responseAt?: number;
  durationMs?: number;
  isError?: boolean;
  text?: string;
  /** System 1 made this call or wrote this text instead of Gemini. */
  s1?: S1Tag;
}

export interface EventsState {
  rows: EventRow[];
  indexByCallId: Record<string, number>;
}

export const EVENT_CAP = 2000;

export function emptyEvents(): EventsState {
  return { rows: [], indexByCallId: {} };
}

function isRecord(value: unknown): value is Record<string, unknown> {
  return typeof value === "object" && value !== null && !Array.isArray(value);
}

/** True for MCP errors and executor budget refusals (both carry an `error`/`isError`). */
export function isErrorResponse(response: unknown): boolean {
  if (!isRecord(response)) return false;
  return response.isError === true || response.is_error === true || "error" in response;
}

function reindex(rows: EventRow[]): Record<string, number> {
  const index: Record<string, number> = {};
  rows.forEach((row, i) => {
    if (row.callId !== null) index[row.callId] = i;
  });
  return index;
}

function append(state: EventsState, row: EventRow, cap: number): EventsState {
  let rows = [...state.rows, row];
  if (rows.length > cap) rows = rows.slice(rows.length - cap);
  const indexByCallId =
    rows.length === state.rows.length + 1 && rows.length <= cap
      ? row.callId === null
        ? state.indexByCallId
        : { ...state.indexByCallId, [row.callId]: rows.length - 1 }
      : reindex(rows);
  return { rows, indexByCallId };
}

/** Pure: folds one stream event into the log. Never mutates its input. */
export function ingestEvent(
  state: EventsState,
  event: AdkEvent | TickEvent | NoticeEvent,
  cap = EVENT_CAP,
): EventsState {
  if (event.type === "notice") {
    return append(
      state,
      {
        key: `notice:${event.id}`,
        kind: "notice",
        callId: null,
        tick: event.tick,
        author: "runtime",
        timestamp: event.timestamp,
        name: event.kind,
        text: event.text,
      },
      cap,
    );
  }
  if (event.type === "tick") {
    return append(
      state,
      {
        key: `tick:${event.number}:${event.id}`,
        kind: "tick",
        callId: null,
        tick: event.number,
        author: "runtime",
        timestamp: 0,
        name: `tick ${event.number}`,
        response: {
          seconds: event.seconds,
          tools: event.tools,
          errors: event.errors,
          refusals: event.refusals,
          llm_calls: event.llm_calls,
          truncated: event.truncated,
        },
      },
      cap,
    );
  }

  let next = state;
  if (event.functionCall) {
    const call = event.functionCall;
    next = append(
      next,
      {
        key: `call:${call.id || event.id}`,
        kind: "call",
        callId: call.id || null,
        tick: event.tick,
        author: event.author,
        timestamp: event.timestamp,
        name: call.name,
        args: call.args,
        ...(event.s1 ? { s1: event.s1 } : {}),
      },
      cap,
    );
  }
  if (event.functionResponse) {
    const response = event.functionResponse;
    const index = response.id ? next.indexByCallId[response.id] : undefined;
    if (index !== undefined && next.rows[index] !== undefined) {
      const rows = [...next.rows];
      const row = rows[index] as EventRow;
      rows[index] = {
        ...row,
        response: response.response,
        responseAt: event.timestamp,
        durationMs: Math.max(0, Math.round((event.timestamp - row.timestamp) * 1000)),
        isError: isErrorResponse(response.response),
      };
      next = { rows, indexByCallId: next.indexByCallId };
    } else {
      next = append(
        next,
        {
          key: `response:${response.id || event.id}`,
          kind: "call",
          callId: null,
          tick: event.tick,
          author: event.author,
          timestamp: event.timestamp,
          name: response.name,
          response: response.response,
          responseAt: event.timestamp,
          isError: isErrorResponse(response.response),
        },
        cap,
      );
    }
  }
  if (event.text && !event.functionCall && !event.functionResponse) {
    next = append(
      next,
      {
        key: `text:${event.id}`,
        kind: "text",
        callId: null,
        tick: event.tick,
        author: event.author,
        timestamp: event.timestamp,
        name: event.author,
        text: event.text,
        ...(event.s1 ? { s1: event.s1 } : {}),
      },
      cap,
    );
  }
  return next;
}
