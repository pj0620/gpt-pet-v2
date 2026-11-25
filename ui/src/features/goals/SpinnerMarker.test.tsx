import { act, render, screen } from "@testing-library/react";
import { afterEach, beforeEach, describe, expect, it, vi } from "vitest";

import { SPINNER_FRAMES, SPINNER_INTERVAL_MS, SpinnerMarker } from "@/features/goals/SpinnerMarker";

describe("SpinnerMarker", () => {
  beforeEach(() => {
    vi.useFakeTimers();
  });
  afterEach(() => {
    vi.useRealTimers();
  });

  it("cycles through the line frames while running", () => {
    render(<SpinnerMarker running />);
    const marker = screen.getByTestId("goal-spinner");
    expect(marker.textContent).toBe(`[${SPINNER_FRAMES[0]}]`);
    for (let i = 1; i <= SPINNER_FRAMES.length; i += 1) {
      act(() => {
        vi.advanceTimersByTime(SPINNER_INTERVAL_MS);
      });
      expect(marker.textContent).toBe(`[${SPINNER_FRAMES[i % SPINNER_FRAMES.length]}]`);
    }
  });

  it("holds still when the pet is not running", () => {
    render(<SpinnerMarker running={false} />);
    const marker = screen.getByTestId("goal-spinner");
    act(() => {
      vi.advanceTimersByTime(SPINNER_INTERVAL_MS * 5);
    });
    expect(marker.textContent).toBe(`[${SPINNER_FRAMES[0]}]`);
    expect(marker).toHaveAttribute("aria-label", "in progress, paused");
  });
});
