import { describe, expect, it } from "vitest";

import { addDelta, compactDelta, deltaForKey, isZeroDelta, ZERO_DELTA } from "@/features/camera/cameraDeltas";

describe("camera deltas", () => {
  it("coalesces inputs and compacts them for the wire", () => {
    let pending = addDelta(ZERO_DELTA, { yaw: 3.3333 });
    pending = addDelta(pending, { yaw: 1.1111, forward: 0.25 });
    pending = addDelta(pending, { forward: -0.25 });
    expect(pending.yaw).toBeCloseTo(4.4444);
    expect(compactDelta(pending)).toEqual({ yaw: 4.444 });
    expect(isZeroDelta(ZERO_DELTA)).toBe(true);
    expect(isZeroDelta(pending)).toBe(false);
  });

  it("maps keys to camera-relative moves", () => {
    expect(deltaForKey("w")).toEqual({ forward: 0.25 });
    expect(deltaForKey("ArrowLeft")).toEqual({ yaw: -10 });
    expect(deltaForKey("+")).toEqual({ zoom: 5 });
    expect(deltaForKey("x")).toBeNull();
  });
});
