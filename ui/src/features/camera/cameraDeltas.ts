import type { CameraMoveBody } from "@/types/api";

export type CameraDelta = Required<CameraMoveBody>;

export const ZERO_DELTA: CameraDelta = { forward: 0, right: 0, up: 0, yaw: 0, pitch: 0, zoom: 0 };

export const MOVE_STEP_M = 0.25;
export const LOOK_STEP_DEG = 10;
export const ZOOM_STEP_DEG = 5;
export const DRAG_DEG_PER_PX = 0.25;
export const WHEEL_DEG_PER_UNIT = 0.05;

/** Pure: fold one input into the pending delta (inputs between requests are coalesced). */
export function addDelta(pending: CameraDelta, delta: CameraMoveBody): CameraDelta {
  return {
    forward: pending.forward + (delta.forward ?? 0),
    right: pending.right + (delta.right ?? 0),
    up: pending.up + (delta.up ?? 0),
    yaw: pending.yaw + (delta.yaw ?? 0),
    pitch: pending.pitch + (delta.pitch ?? 0),
    zoom: pending.zoom + (delta.zoom ?? 0),
  };
}

export function isZeroDelta(delta: CameraDelta): boolean {
  return Object.values(delta).every((value) => Math.abs(value) < 1e-6);
}

/** Round for the wire and drop the zero fields. */
export function compactDelta(delta: CameraDelta): CameraMoveBody {
  const body: CameraMoveBody = {};
  for (const [key, value] of Object.entries(delta) as [keyof CameraDelta, number][]) {
    if (Math.abs(value) >= 1e-6) body[key] = Math.round(value * 1000) / 1000;
  }
  return body;
}

/** Keyboard mapping: WASD move, Q/E down/up, arrows look, +/- zoom. Null for other keys. */
export function deltaForKey(key: string): CameraMoveBody | null {
  switch (key.toLowerCase()) {
    case "w":
      return { forward: MOVE_STEP_M };
    case "s":
      return { forward: -MOVE_STEP_M };
    case "a":
      return { right: -MOVE_STEP_M };
    case "d":
      return { right: MOVE_STEP_M };
    case "e":
      return { up: MOVE_STEP_M };
    case "q":
      return { up: -MOVE_STEP_M };
    case "arrowleft":
      return { yaw: -LOOK_STEP_DEG };
    case "arrowright":
      return { yaw: LOOK_STEP_DEG };
    case "arrowup":
      return { pitch: -LOOK_STEP_DEG };
    case "arrowdown":
      return { pitch: LOOK_STEP_DEG };
    case "+":
    case "=":
      return { zoom: ZOOM_STEP_DEG };
    case "-":
    case "_":
      return { zoom: -ZOOM_STEP_DEG };
    default:
      return null;
  }
}
