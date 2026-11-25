import { render, screen, within } from "@testing-library/react";
import { describe, expect, it } from "vitest";

import { StatsGrid } from "@/features/stats/StatsGrid";
import { STATS_SNAPSHOT } from "@/test/fixtures";

function tile(testId: string) {
  const element = screen.getByTestId(testId);
  return { value: within(element).getByTestId("stat-value").textContent, text: element.textContent ?? "" };
}

describe("StatsGrid", () => {
  it("shows LLM calls, tokens, latency, goals and driving from one snapshot", () => {
    render(<StatsGrid stats={STATS_SNAPSHOT} />);
    expect(tile("stat-llm-calls").value).toBe("12");
    expect(tile("stat-llm-calls").text).toContain("setter 3 · executor 9");
    expect(tile("stat-tokens").value).toBe("46.1k");
    expect(tile("stat-tokens").text).toContain("in 43.1k · out 1.3k · think 1.8k");
    expect(tile("stat-token-rate").value).toBe("22.1k");
    expect(tile("stat-llm-latency").value).toBe("2.1 s");
    expect(tile("stat-llm-latency").text).toContain("p95 5.2 s");
    expect(tile("stat-goals").value).toBe("0.48");
    expect(tile("stat-goals").text).toContain("2 started · 1 done · 0 abandoned");
    expect(screen.getByTestId("stat-goals")).toHaveAttribute("title", "Goals started: 0.96 per minute");
    expect(tile("stat-ticks").text).toContain("3 ticks · last 21 s");
    expect(tile("stat-driving").value).toBe("24.4%");
    expect(tile("stat-driving").text).toContain("2 drives · 2 reached · 60 S1 checks");
    expect(tile("stat-tools").text).toContain("1 error · 1 refused");
    expect(screen.getByTestId("stats-since").textContent).toContain("first action after 12 s");
  });

  it("reads off for System 1 while it is disabled", () => {
    render(<StatsGrid stats={STATS_SNAPSHOT} />);
    expect(tile("stat-s1-share").value).toBe("off");
    expect(tile("stat-s1-share").text).toContain("enable [s1] in the profile");
    expect(tile("stat-s1-decider").value).toBe("off");
  });

  it("shows System 1's share of the turns, its hand-offs, and the decision model's latency", () => {
    const s1 = {
      enabled: true,
      model: "nimble",
      turns: 13,
      share_pct: 52,
      by: { nimble: 5, rules: 8 },
      by_agent: { executor: 11, goal_setter: 2 },
      escalations: { "executor: unsure": 3, "goal_setter: no active goal": 1 },
      decider: { calls: 7, latency: { count: 7, avg_ms: 1310, p50_ms: 1290, p95_ms: 2480, last_ms: 1300 } },
    };
    render(<StatsGrid stats={{ ...STATS_SNAPSHOT, s1 }} />);
    expect(tile("stat-s1-share").value).toBe("52%");
    expect(tile("stat-s1-share").text).toContain("rules 8 · nimble 5 · 4 to Gemini");
    expect(screen.getByTestId("stat-s1-share").getAttribute("title")).toContain("executor: unsure × 3");
    expect(tile("stat-s1-decider").value).toBe("7");
    expect(tile("stat-s1-decider").text).toContain("nimble · avg 1.3 s · p95 2.5 s");
  });
});
