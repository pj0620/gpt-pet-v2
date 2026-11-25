import type { AttentionStatus, LatencySummary, RunStats } from "@/types/api";

/** An attention status as `get_attention_status` returns it while a person is framed. */
export const ATTENTION_FRAMED: AttentionStatus = {
  state: "framed",
  target: "obj:person_0",
  label: "Person",
  distance_m: 1.4,
  bearing_deg: -3.0,
  seen_s: 12.0,
  unseen_s: 0,
  moving: false,
  tilt_deg: 27.0,
  standoff_m: 1.3,
  since_s: 14.0,
  target_xz: [1.0, 2.0],
  reason: null,
};

const NO_SAMPLES: LatencySummary = { count: 0, avg_ms: null, p50_ms: null, p95_ms: null, max_ms: null, last_ms: null };

/** A stats snapshot shaped exactly like `RunStats.snapshot()` in python/src/gpt_pet/stats.py. */
export const STATS_SNAPSHOT: RunStats = {
  started_at: 1_790_000_000,
  uptime_s: 125.4,
  llm: {
    calls: 12,
    errors: 0,
    fallbacks: 0,
    maps_shown: 0,
    per_min: 5.74,
    latency: { count: 12, avg_ms: 2140, p50_ms: 1800, p95_ms: 5200, max_ms: 7400, last_ms: 1210 },
    failed_latency: NO_SAMPLES,
    by_agent: {
      executor: { calls: 9, tokens: 41_200, avg_ms: 2300 },
      goal_setter: { calls: 3, tokens: 4_900, avg_ms: 1660 },
    },
  },
  tokens: {
    input: 43_100,
    output: 1_250,
    thinking: 1_750,
    cached: 0,
    total: 46_100,
    per_min: 22_057.4,
    last_call: 5_123,
  },
  tools: {
    calls: 14,
    errors: 1,
    refusals: 1,
    latency: { count: 13, avg_ms: 640, p50_ms: 420, p95_ms: 2100, max_ms: 2900, last_ms: 380 },
  },
  ticks: {
    count: 3,
    truncated: 0,
    failed: 0,
    latency: { count: 3, avg_ms: 33_300, p50_ms: 30_100, p95_ms: 41_000, max_ms: 41_000, last_ms: 20_700 },
  },
  goals: { started: 2, done: 1, abandoned: 0, started_per_min: 0.96, done_per_min: 0.48 },
  drives: {
    count: 2,
    outcomes: { succeeded: 2 },
    driving_s: 30.6,
    driving_pct: 24.4,
    checks: 60,
    latency: { count: 2, avg_ms: 15_300, p50_ms: 15_300, p95_ms: 15_300, max_ms: 15_300, last_ms: 15_300 },
    null: 1,
    to_passages: 1,
    distance_m: 4.2,
    destinations_reached: 2,
    calls_by: { gemini: 2 },
  },
  s1: {
    enabled: false,
    model: null,
    turns: 0,
    share_pct: 0,
    by: {},
    by_agent: {},
    escalations: {},
    decider: { calls: 0, latency: NO_SAMPLES },
  },
  first_action_s: 12.3,
};
