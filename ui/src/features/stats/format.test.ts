import { describe, expect, it } from "vitest";

import { formatCount, formatMs, formatRate, formatSeconds } from "@/features/stats/format";

describe("stats formatting", () => {
  it("compacts counts", () => {
    expect(formatCount(0)).toBe("0");
    expect(formatCount(999)).toBe("999");
    expect(formatCount(1_000)).toBe("1k");
    expect(formatCount(46_100)).toBe("46.1k");
    expect(formatCount(123_456)).toBe("123k");
    expect(formatCount(1_250_000)).toBe("1.3M");
  });

  it("shows latencies in the unit that reads best, and a dash before any sample", () => {
    expect(formatMs(null)).toBe("—");
    expect(formatMs(850)).toBe("850 ms");
    expect(formatMs(4_400)).toBe("4.4 s");
    expect(formatMs(33_300)).toBe("33 s");
    expect(formatMs(65_000)).toBe("1m 05s");
  });

  it("shows durations up to hours", () => {
    expect(formatSeconds(4)).toBe("4 s");
    expect(formatSeconds(119.6)).toBe("1m 59s");
    expect(formatSeconds(7_380)).toBe("2h 03m");
  });

  it("keeps small rates precise and large ones compact", () => {
    expect(formatRate(0)).toBe("0");
    expect(formatRate(0.48)).toBe("0.48");
    expect(formatRate(12.54)).toBe("12.5");
    expect(formatRate(22_057.4)).toBe("22.1k");
  });
});
