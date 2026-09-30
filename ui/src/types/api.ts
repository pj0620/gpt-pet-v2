/** Wire types for the pet server's `/api/*` surface (see the plan; hand-written until the
 *  backend exists, then optionally replaced by openapi-typescript output). */

export type GoalStatus = "active" | "done" | "abandoned";

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
  /** Portal features the active profile offers; the free camera exists only in the simulator. */
  features?: { free_camera?: boolean };
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
}

export interface ImageEvent {
  type: "image";
  id: string;
  name: ImageName;
  version: number;
}

export type PortalEvent = AdkEvent | StateEvent | StatusEvent | TickEvent | ImageEvent;
