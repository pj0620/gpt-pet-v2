import { render, screen } from "@testing-library/react";
import { afterEach, describe, expect, it } from "vitest";

import { AttentionStrip, describeAttention } from "@/features/attention/AttentionStrip";
import { usePortalStore } from "@/stores/portalStore";
import { ATTENTION_FRAMED } from "@/test/fixtures";

afterEach(() => {
  usePortalStore.setState({ attention: null });
});

describe("describeAttention", () => {
  it("says what the robot is doing and where the person is, per state", () => {
    expect(describeAttention(ATTENTION_FRAMED)).toBe("Watching Person · 1.4 m · 3° left · in view 12 s");
    expect(describeAttention({ ...ATTENTION_FRAMED, state: "aiming", distance_m: 3.0, bearing_deg: 40 })).toBe(
      "Turning toward Person · 3.0 m · 40° right",
    );
    expect(describeAttention({ ...ATTENTION_FRAMED, state: "approaching", distance_m: 3.1, bearing_deg: 0.4 })).toBe(
      "Walking up to Person · 3.1 m · straight ahead · stops 1.3 m away",
    );
    expect(describeAttention({ ...ATTENTION_FRAMED, state: "searching", unseen_s: 2.4 })).toBe(
      "Looking for Person · out of sight 2 s",
    );
    expect(describeAttention({ ...ATTENTION_FRAMED, state: "lost", unseen_s: 5.1 })).toBe(
      "Lost sight of Person after 5 s",
    );
    expect(describeAttention({ ...ATTENTION_FRAMED, state: "off", reason: "a drive started" })).toBe(
      "Attention ended: a drive started",
    );
    expect(
      describeAttention({ ...ATTENTION_FRAMED, state: "aiming", label: null, distance_m: null, bearing_deg: null }),
    ).toBe("Turning toward the person");
  });
});

describe("AttentionStrip", () => {
  it("renders nothing while attention is off", () => {
    render(<AttentionStrip />);
    expect(screen.queryByTestId("attention-strip")).toBeNull();
    usePortalStore.setState({ attention: { ...ATTENTION_FRAMED, state: "off" } });
    render(<AttentionStrip />);
    expect(screen.queryByTestId("attention-strip")).toBeNull();
  });

  it("shows the framed person, the camera tilt and an explanation", () => {
    usePortalStore.setState({ attention: ATTENTION_FRAMED });
    render(<AttentionStrip />);
    const strip = screen.getByTestId("attention-strip");
    expect(strip).toHaveAttribute("data-state", "framed");
    expect(strip).toHaveTextContent("Watching Person · 1.4 m · 3° left · in view 12 s");
    expect(strip).toHaveTextContent("camera up 27°");
    expect(strip.getAttribute("title")).toContain("centred with the camera tilted");
    expect(strip).not.toHaveTextContent("moving");
  });

  it("flags a moving person while the robot is still turning", () => {
    usePortalStore.setState({ attention: { ...ATTENTION_FRAMED, state: "aiming", moving: true, tilt_deg: 0 } });
    render(<AttentionStrip />);
    const strip = screen.getByTestId("attention-strip");
    expect(strip).toHaveAttribute("data-state", "aiming");
    expect(strip).toHaveTextContent("moving");
    expect(strip).not.toHaveTextContent("camera up");
  });
});
