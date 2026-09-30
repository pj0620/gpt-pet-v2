import { describe, expect, it } from "vitest";

import { decodeEvent, isPortalEventName } from "@/api/sse";

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
