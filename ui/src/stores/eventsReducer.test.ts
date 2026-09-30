import { describe, expect, it } from "vitest";

import { emptyEvents, ingestEvent, isErrorResponse } from "@/stores/eventsReducer";
import type { AdkEvent, TickEvent } from "@/types/api";

const call: AdkEvent = {
  type: "adk",
  id: "10",
  tick: 1,
  author: "executor",
  timestamp: 1000.0,
  functionCall: { id: "fc-1", name: "get_current_view", args: {} },
};

const response: AdkEvent = {
  type: "adk",
  id: "11",
  tick: 1,
  author: "executor",
  timestamp: 1001.25,
  functionResponse: { id: "fc-1", name: "get_current_view", response: { result: { agent: { x: 1 } }, isError: false } },
};

describe("ingestEvent", () => {
  it("pairs a function response with its call by id and computes the duration", () => {
    let state = ingestEvent(emptyEvents(), call);
    expect(state.rows).toHaveLength(1);
    expect(state.rows[0]).toMatchObject({ kind: "call", name: "get_current_view", callId: "fc-1" });
    expect(state.rows[0]?.response).toBeUndefined();

    state = ingestEvent(state, response);
    expect(state.rows).toHaveLength(1);
    expect(state.rows[0]).toMatchObject({ durationMs: 1250, isError: false });
    expect(state.rows[0]?.response).toEqual(response.functionResponse?.response);
  });

  it("keeps an unmatched response as its own row and flags errors", () => {
    const refused: AdkEvent = {
      ...response,
      id: "12",
      functionResponse: { id: "fc-unknown", name: "do_rotate", response: { error: "budget exhausted: do_rotate" } },
    };
    const state = ingestEvent(emptyEvents(), refused);
    expect(state.rows).toHaveLength(1);
    expect(state.rows[0]).toMatchObject({ kind: "call", callId: null, name: "do_rotate", isError: true });
  });

  it("records agent text and tick markers as rows", () => {
    const text: AdkEvent = {
      type: "adk",
      id: "13",
      tick: 1,
      author: "goal_setter",
      timestamp: 999,
      text: '{"goal":"x"}',
    };
    const tick: TickEvent = {
      type: "tick",
      id: "14",
      number: 1,
      seconds: 26.6,
      tools: ["get_current_view"],
      errors: 0,
      refusals: 1,
      llm_calls: 17,
      truncated: false,
    };
    let state = ingestEvent(emptyEvents(), text);
    state = ingestEvent(state, tick);
    expect(state.rows.map((row) => row.kind)).toEqual(["text", "tick"]);
    expect(state.rows[1]).toMatchObject({ name: "tick 1", response: { refusals: 1, llm_calls: 17 } });
  });

  it("does not mutate its input and enforces the cap with a valid index", () => {
    const initial = emptyEvents();
    let state = initial;
    for (let i = 0; i < 5; i += 1) {
      state = ingestEvent(
        state,
        { ...call, id: `c${i}`, functionCall: { id: `fc-${i}`, name: "get_nav_status", args: {} } },
        3,
      );
    }
    expect(initial.rows).toHaveLength(0);
    expect(state.rows.map((row) => row.callId)).toEqual(["fc-2", "fc-3", "fc-4"]);
    expect(state.indexByCallId).toEqual({ "fc-2": 0, "fc-3": 1, "fc-4": 2 });
    const paired = ingestEvent(
      state,
      {
        ...response,
        functionResponse: { id: "fc-4", name: "get_nav_status", response: { result: { state: "succeeded" } } },
      },
      3,
    );
    expect(paired.rows[2]?.response).toEqual({ result: { state: "succeeded" } });
  });
});

describe("isErrorResponse", () => {
  it("recognises MCP errors and budget refusals only", () => {
    expect(isErrorResponse({ result: "Successfully moved", isError: false })).toBe(false);
    expect(isErrorResponse({ result: "Move blocked", isError: true })).toBe(true);
    expect(isErrorResponse({ error: "budget exhausted: get_nav_status" })).toBe(true);
    expect(isErrorResponse("text")).toBe(false);
  });
});
