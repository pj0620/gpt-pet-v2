import { describe, expect, it } from "vitest";

import { decodeEvent, isPortalEventName, isRunStats } from "@/api/sse";
import { STATS_SNAPSHOT as SNAPSHOT } from "@/test/fixtures";

describe("decodeEvent", () => {
  it("decodes each typed frame", () => {
    expect(
      decodeEvent(
        "adk",
        JSON.stringify({
          tick: 1,
          author: "executor",
          timestamp: 5,
          functionCall: { id: "a", name: "get_map", args: {} },
        }),
        "1",
      ),
    ).toMatchObject({
      type: "adk",
      id: "1",
      functionCall: { id: "a", name: "get_map", args: {} },
    });
    expect(
      decodeEvent("state", JSON.stringify({ tick: 3, current_goal: null, goal_history: [], last_report: "" }), "2"),
    ).toMatchObject({
      type: "state",
      state: { tick: 3 },
    });
    expect(
      decodeEvent(
        "status",
        JSON.stringify({
          profile: "sim",
          paused: false,
          tick: 3,
          running_tick: true,
          mcp_url: "http://localhost:8000/mcp",
          last_tick_seconds: 20,
        }),
        "3",
      ),
    ).toMatchObject({
      type: "status",
      status: { paused: false },
    });
    expect(
      decodeEvent(
        "tick",
        JSON.stringify({
          number: 4,
          seconds: 19.3,
          tools: ["do_rotate"],
          errors: 0,
          refusals: 0,
          llm_calls: 14,
          truncated: false,
        }),
        "4",
      ),
    ).toMatchObject({
      type: "tick",
      number: 4,
      tools: ["do_rotate"],
    });
    expect(decodeEvent("image", JSON.stringify({ name: "camera", version: 12 }), "5")).toEqual({
      type: "image",
      id: "5",
      name: "camera",
      version: 12,
    });
  });

  it("keeps the System 1 tag on ADK events", () => {
    const frame = {
      tick: 2,
      author: "executor",
      timestamp: 7,
      functionCall: { id: "b", name: "get_current_view", args: {} },
      s1: { by: "rules", action: "get_current_view" },
    };
    expect(decodeEvent("adk", JSON.stringify(frame), "30")).toMatchObject({
      s1: { by: "rules", action: "get_current_view" },
    });
    const noTag = decodeEvent("adk", JSON.stringify({ ...frame, s1: "yes" }), "31");
    expect(noTag).not.toBeNull();
    expect(noTag && "s1" in noTag).toBe(false);
  });

  it("decodes live stats snapshots and runtime notices", () => {
    expect(decodeEvent("stats", JSON.stringify(SNAPSHOT), "20")).toEqual({ type: "stats", id: "20", stats: SNAPSHOT });
    expect(
      decodeEvent(
        "notice",
        JSON.stringify({
          kind: "rest",
          tick: 2,
          text: "Resting 60 s before the next tick.",
          timestamp: 9,
          seconds: 60,
        }),
        "21",
      ),
    ).toEqual({
      type: "notice",
      id: "21",
      kind: "rest",
      tick: 2,
      text: "Resting 60 s before the next tick.",
      timestamp: 9,
      seconds: 60,
    });
    expect(decodeEvent("stats", "{}", "22")).toBeNull(); // no pet running
    expect(decodeEvent("notice", JSON.stringify({ kind: "rest" }), "23")).toBeNull();
    expect(isRunStats(SNAPSHOT)).toBe(true);
    expect(isRunStats({ uptime_s: 3 })).toBe(false);
  });

  it("rejects unknown names, malformed JSON, and wrong shapes", () => {
    expect(isPortalEventName("ping")).toBe(false);
    expect(decodeEvent("ping", "{}", "6")).toBeNull();
    expect(decodeEvent("adk", "{not json", "7")).toBeNull();
    expect(decodeEvent("adk", "[]", "8")).toBeNull();
    expect(decodeEvent("image", JSON.stringify({ name: "lidar", version: 1 }), "9")).toBeNull();
    expect(decodeEvent("status", JSON.stringify({ paused: "no" }), "10")).toBeNull();
    expect(decodeEvent("state", JSON.stringify({ tick: "x" }), "11")).toBeNull();
  });
});
