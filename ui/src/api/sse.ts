import type {
  AdkEvent,
  ImageEvent,
  ImageName,
  NoticeEvent,
  PortalEvent,
  RunStats,
  StateEvent,
  StatusEvent,
  TickEvent,
} from "@/types/api";

export const PORTAL_EVENT_NAMES = ["adk", "state", "status", "tick", "image", "stats", "notice"] as const;
export type PortalEventName = (typeof PORTAL_EVENT_NAMES)[number];

const IMAGE_NAMES: readonly ImageName[] = ["camera", "map", "depth", "free"];

export function isPortalEventName(name: string): name is PortalEventName {
  return (PORTAL_EVENT_NAMES as readonly string[]).includes(name);
}

function isRecord(value: unknown): value is Record<string, unknown> {
  return typeof value === "object" && value !== null && !Array.isArray(value);
}

/** True for a real stats snapshot; the server answers `{}` while no pet is running. */
export function isRunStats(value: unknown): value is RunStats {
  return isRecord(value) && typeof value.uptime_s === "number" && isRecord(value.llm) && isRecord(value.tokens);
}

function parse(data: string): Record<string, unknown> | null {
  try {
    const value: unknown = JSON.parse(data);
    return isRecord(value) ? value : null;
  } catch {
    return null;
  }
}

/**
 * Turns one SSE frame (`event:` name, `data:` JSON, `id:`) into a typed PortalEvent.
 * Returns null for unknown event names, malformed JSON, or payloads missing required fields,
 * so a bad frame never takes the stream down.
 */
export function decodeEvent(name: string, data: string, id: string): PortalEvent | null {
  if (!isPortalEventName(name)) return null;
  const payload = parse(data);
  if (payload === null) return null;

  switch (name) {
    case "adk": {
      if (typeof payload.author !== "string" || typeof payload.timestamp !== "number") return null;
      const event: AdkEvent = {
        type: "adk",
        id,
        tick: typeof payload.tick === "number" ? payload.tick : 0,
        author: payload.author,
        timestamp: payload.timestamp,
      };
      if (isRecord(payload.functionCall) && typeof payload.functionCall.name === "string") {
        event.functionCall = {
          id: String(payload.functionCall.id ?? ""),
          name: payload.functionCall.name,
          args: isRecord(payload.functionCall.args) ? payload.functionCall.args : {},
        };
      }
      if (isRecord(payload.functionResponse) && typeof payload.functionResponse.name === "string") {
        event.functionResponse = {
          id: String(payload.functionResponse.id ?? ""),
          name: payload.functionResponse.name,
          response: payload.functionResponse.response,
        };
      }
      if (typeof payload.text === "string" && payload.text.length > 0) event.text = payload.text;
      if (isRecord(payload.s1) && typeof payload.s1.by === "string") {
        const { by, action, model, certainty } = payload.s1;
        event.s1 = { by, action: typeof action === "string" ? action : "" };
        if (typeof model === "string") event.s1.model = model;
        if (typeof certainty === "number") event.s1.certainty = certainty;
      }
      return event;
    }
    case "state": {
      const state = isRecord(payload.state) ? payload.state : payload;
      if (typeof state.tick !== "number" || !Array.isArray(state.goal_history)) return null;
      return { type: "state", id, state: state as unknown as StateEvent["state"] };
    }
    case "status": {
      const status = isRecord(payload.status) ? payload.status : payload;
      if (typeof status.paused !== "boolean" || typeof status.profile !== "string") return null;
      return { type: "status", id, status: status as unknown as StatusEvent["status"] };
    }
    case "tick": {
      if (typeof payload.number !== "number") return null;
      const event: TickEvent = {
        type: "tick",
        id,
        number: payload.number,
        seconds: typeof payload.seconds === "number" ? payload.seconds : 0,
        tools: Array.isArray(payload.tools) ? payload.tools.map(String) : [],
        errors: typeof payload.errors === "number" ? payload.errors : 0,
        refusals: typeof payload.refusals === "number" ? payload.refusals : 0,
        llm_calls: typeof payload.llm_calls === "number" ? payload.llm_calls : 0,
        truncated: payload.truncated === true,
      };
      return event;
    }
    case "image": {
      if (typeof payload.name !== "string" || typeof payload.version !== "number") return null;
      if (!IMAGE_NAMES.includes(payload.name as ImageName)) return null;
      const event: ImageEvent = { type: "image", id, name: payload.name as ImageName, version: payload.version };
      return event;
    }
    case "stats": {
      if (!isRunStats(payload)) return null;
      return { type: "stats", id, stats: payload };
    }
    case "notice": {
      if (typeof payload.kind !== "string" || typeof payload.text !== "string") return null;
      const event: NoticeEvent = {
        type: "notice",
        id,
        kind: payload.kind,
        tick: typeof payload.tick === "number" ? payload.tick : 0,
        text: payload.text,
        timestamp: typeof payload.timestamp === "number" ? payload.timestamp : 0,
      };
      if (typeof payload.seconds === "number") event.seconds = payload.seconds;
      return event;
    }
    default:
      return null;
  }
}
