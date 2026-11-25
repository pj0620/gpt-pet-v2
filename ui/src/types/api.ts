/** Wire types for the pet server's `/api/*` surface (see the plan; hand-written until the
 *  backend exists, then optionally replaced by openapi-typescript output). */

/** "canceled": the owner submitted a new goal while this one was in progress. */
export type GoalStatus = "active" | "done" | "abandoned" | "canceled";

export interface Goal {
  id: number;
  goal: string;
  success_criteria: string;
  sub_goals: string[];
  status: GoalStatus;
  created_tick: number;
  attempts: number;
  finished_tick?: number;
}

/** A goal submitted from the portal, waiting for the goal setter to adopt it. */
export interface PendingGoal {
  id?: number;
  goal: string;
  sub_goals?: string[];
  source?: string;
}

export interface GoalDecision {
  previous_goal_status: "none" | "continue" | "done" | "abandoned";
  goal?: string | null;
  success_criteria?: string | null;
  sub_goals?: string[];
  reasoning: string;
}

/** Mirrors the ADK session state written by python/src/gpt_pet/goals.py. */
export interface PetState {
  tick: number;
  current_goal: Goal | null;
  goal_history: Goal[];
  pending_goals?: PendingGoal[];
  last_report: string;
  goal_decision?: GoalDecision | null;
}

export type ProfileName = "sim" | "real";

export interface PetStatus {
  profile: ProfileName;
  paused: boolean;
  tick: number;
  running_tick: boolean;
  mcp_url: string;
  last_tick_seconds: number | null;
  /** Seconds left in the rest between ticks (goal_delay_s / tick_delay_s), when resting. */
  waiting_s?: number | null;
  /** The run's goal limit was reached; the loop paused itself until more goals are added. */
  goal_limit_reached?: boolean;
  /** Goals this run may start (null = unlimited) and goals started so far. */
  goal_limit?: number | null;
  goals_used?: number;
  /** Portal features the active profile offers; the free camera exists only in the simulator,
   *  attention once the robot's server has the attention tools. */
  features?: { free_camera?: boolean; attention?: boolean };
}

export interface CameraPose {
  position: { x: number; y: number; z: number };
  yaw: number;
  pitch: number;
  fov: number;
}

export interface CameraMoveBody {
  forward?: number;
  right?: number;
  up?: number;
  yaw?: number;
  pitch?: number;
  zoom?: number;
}

export type CameraPreset = "chase" | "front" | "top";

export interface CameraResponse {
  pose: CameraPose | null;
  version: number;
}

/** `Settings.model_dump(mode="json")` from python/src/gpt_pet/settings.py. */
export type Settings = Record<string, unknown>;

export interface SubmitGoalBody {
  goal: string;
  sub_goals?: string[];
}

export type ControlAction = "pause" | "resume" | "tick";

export type ImageName = "camera" | "map" | "depth" | "free";

export interface FunctionCall {
  id: string;
  name: string;
  args: Record<string, unknown>;
}

export interface FunctionResponse {
  id: string;
  name: string;
  response: unknown;
}

/** Present when System 1 took the turn instead of Gemini (python/src/gpt_pet/s1.py). */
export interface S1Tag {
  /** "rules" (no model at all) or "nimble" (the local decision model). */
  by: string;
  action: string;
  model?: string;
  certainty?: number;
}

/** One ADK event, image bytes stripped by the server. */
export interface AdkEvent {
  type: "adk";
  id: string;
  tick: number;
  author: string;
  timestamp: number;
  functionCall?: FunctionCall;
  functionResponse?: FunctionResponse;
  text?: string;
  s1?: S1Tag;
}

export interface StateEvent {
  type: "state";
  id: string;
  state: PetState;
}

export interface StatusEvent {
  type: "status";
  id: string;
  status: PetStatus;
}

/** One line per tick; mirrors TickResult.summary() in python/src/gpt_pet/runtime.py. */
export interface TickEvent {
  type: "tick";
  id: string;
  number: number;
  seconds: number;
  tools: string[];
  errors: number;
  refusals: number;
  llm_calls: number;
  truncated: boolean;
  /** Failed Gemini calls a fallback answered this tick. */
  fallbacks?: number;
  /** Why the tick failed (a transient error such as a Gemini timeout); null when it ran through. */
  error?: string | null;
}

export interface ImageEvent {
  type: "image";
  id: string;
  name: ImageName;
  version: number;
}

/** Latency summary; percentiles cover the most recent 200 samples. Null before the first one. */
export interface LatencySummary {
  count: number;
  avg_ms: number | null;
  p50_ms: number | null;
  p95_ms: number | null;
  /** The longest over the whole run. */
  max_ms: number | null;
  last_ms: number | null;
}

/** `RunStats.snapshot()` in python/src/gpt_pet/stats.py; rates are per minute of uptime. */
export interface RunStats {
  /** Unix seconds when the counters started (server start, or the last reset). */
  started_at: number;
  uptime_s: number;
  llm: {
    /** Gemini calls that answered; `errors` raised instead (a timeout, say). */
    calls: number;
    errors: number;
    /** Failed calls an agent's fallback answered in Gemini's place. */
    fallbacks: number;
    /** Goal decisions Gemini made with the robot's map attached (`[brain] map_for_goals`). */
    maps_shown: number;
    per_min: number;
    latency: LatencySummary;
    /** How long the failed calls took to fail. */
    failed_latency: LatencySummary;
    by_agent: Record<string, { calls: number; tokens: number; avg_ms: number }>;
  };
  tokens: {
    input: number;
    output: number;
    thinking: number;
    cached: number;
    total: number;
    per_min: number;
    last_call: number;
  };
  /** Every tool call the model made; refused calls (tool budget) never ran, so have no latency. */
  tools: { calls: number; errors: number; refusals: number; latency: LatencySummary };
  /** `failed`: ticks a transient error ended early; the loop retries them after a backoff. */
  ticks: { count: number; truncated: number; failed: number; latency: LatencySummary };
  /** `canceled`: goals the owner replaced by submitting a new one (absent from older servers). */
  goals: {
    started: number;
    done: number;
    abandoned: number;
    canceled?: number;
    started_per_min: number;
    done_per_min: number;
  };
  /** Drives waited out by the brain itself (`checks` = get_nav_status calls, no LLM involved). */
  drives: {
    count: number;
    outcomes: Record<string, number>;
    driving_s: number;
    driving_pct: number;
    checks: number;
    latency: LatencySummary;
    /** Drives that went nowhere: the destination was already within reach. */
    null: number;
    /** Drives to passages (doorways, openings). */
    to_passages: number;
    distance_m: number;
    destinations_reached: number;
    /** set_nav_goal calls by who decided them: "s1" or "gemini". */
    calls_by: Record<string, number>;
  };
  /** System 1: turns it took instead of Gemini, hand-offs to Gemini (by reason), model calls. */
  s1: {
    enabled: boolean;
    model: string | null;
    turns: number;
    /** S1 turns as a share of all model turns (S1 + Gemini). */
    share_pct: number;
    by: Record<string, number>;
    by_agent: Record<string, number>;
    /** "agent: reason" -> count, most frequent first. */
    escalations: Record<string, number>;
    decider: { calls: number; latency: LatencySummary };
  };
  first_action_s: number | null;
}

export interface StatsEvent {
  type: "stats";
  id: string;
  stats: RunStats;
}

/** A runtime notice for the Events Log, such as the rest between ticks or the goal-limit pause. */
export interface NoticeEvent {
  type: "notice";
  id: string;
  kind: string;
  tick: number;
  text: string;
  timestamp: number;
  seconds?: number;
}

export const ATTENTION_STATES = ["off", "aiming", "approaching", "framed", "searching", "lost"] as const;
export type AttentionState = (typeof ATTENTION_STATES)[number];

/** The robot keeping a person framed on its own (ai2thor-mcp's attention tools): what it is
 *  doing and to whom, as polled by the server from `get_attention_status`. */
export interface AttentionStatus {
  state: AttentionState;
  /** The person's destination id, e.g. "obj:person_0". */
  target: string | null;
  label: string | null;
  distance_m: number | null;
  /** Degrees off the view's centre, positive to the right. */
  bearing_deg: number | null;
  /** Seconds the person has been continuously in view (0 when not in view). */
  seen_s: number;
  /** Seconds since the person was last in view (0 while in view). */
  unseen_s: number;
  moving: boolean;
  /** The robot camera's tilt above level, degrees. */
  tilt_deg: number;
  standoff_m: number;
  since_s: number;
  target_xz: [number, number] | null;
  /** Why attention ended (lost, a drive started, manual control, canceled...). */
  reason: string | null;
}

export interface AttentionEvent {
  type: "attention";
  id: string;
  attention: AttentionStatus;
}

export type PortalEvent =
  | AdkEvent
  | StateEvent
  | StatusEvent
  | TickEvent
  | ImageEvent
  | StatsEvent
  | NoticeEvent
  | AttentionEvent;
