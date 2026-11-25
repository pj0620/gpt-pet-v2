import type { ReactNode } from "react";

import { formatCount, formatMs, formatRate, formatSeconds } from "@/features/stats/format";
import type { RunStats } from "@/types/api";

interface StatProps {
  label: string;
  value: ReactNode;
  detail?: string;
  title?: string;
  testId: string;
}

function Stat({ label, value, detail, title, testId }: StatProps) {
  return (
    <div
      className="min-w-0 rounded-lg bg-muted/40 px-2.5 py-1.5 ring-1 ring-foreground/5"
      data-testid={testId}
      title={title}
    >
      <div className="truncate text-[11px] leading-tight text-muted-foreground">{label}</div>
      <div className="truncate font-heading text-base leading-snug font-medium tabular-nums" data-testid="stat-value">
        {value}
      </div>
      {detail ? (
        <div className="truncate text-[11px] leading-tight text-muted-foreground tabular-nums">{detail}</div>
      ) : null}
    </div>
  );
}

function clock(unixSeconds: number): string {
  return new Date(unixSeconds * 1000).toLocaleTimeString([], { hour12: false });
}

function plural(count: number, noun: string): string {
  return `${count} ${noun}${count === 1 ? "" : "s"}`;
}

/** The run's counters as tiles, all from one `RunStats` snapshot. */
export function StatsGrid({ stats }: { stats: RunStats }) {
  const { llm, tokens, tools, ticks, goals, drives, s1 } = stats;
  const agentCalls = (name: string) => llm.by_agent[name]?.calls ?? 0;
  const handoffs = Object.values(s1.escalations).reduce((sum, count) => sum + count, 0);
  const handoffReasons = Object.entries(s1.escalations)
    .map(([reason, count]) => `${reason} × ${count}`)
    .join("\n");

  return (
    <div className="grid grid-cols-2 gap-1.5 p-2.5">
      <Stat
        testId="stat-llm-calls"
        label="LLM calls"
        value={llm.calls}
        detail={`${formatRate(llm.per_min)}/min · setter ${agentCalls("goal_setter")} · executor ${agentCalls("executor")}${llm.errors ? ` · ${plural(llm.errors, "error")}` : ""}`}
      />
      <Stat
        testId="stat-tokens"
        label="Tokens used"
        value={formatCount(tokens.total)}
        detail={`in ${formatCount(tokens.input)} · out ${formatCount(tokens.output)} · think ${formatCount(tokens.thinking)}`}
        title={`${tokens.cached.toLocaleString()} of the input tokens were cached`}
      />
      <Stat
        testId="stat-token-rate"
        label="Tokens / min"
        value={formatRate(tokens.per_min)}
        detail={`last call ${formatCount(tokens.last_call)}`}
      />
      <Stat
        testId="stat-llm-latency"
        label="LLM latency"
        value={formatMs(llm.latency.avg_ms)}
        detail={`p50 ${formatMs(llm.latency.p50_ms)} · p95 ${formatMs(llm.latency.p95_ms)} · last ${formatMs(llm.latency.last_ms)}`}
        title="Average over the run; percentiles over the last 200 calls"
      />
      <Stat
        testId="stat-s1-share"
        label="S1 share of turns"
        value={s1.enabled ? `${s1.share_pct}%` : "off"}
        detail={
          s1.enabled
            ? `rules ${s1.by.rules ?? 0} · nimble ${s1.by.nimble ?? 0} · ${handoffs} to Gemini`
            : "enable [s1] in the profile"
        }
        title={
          s1.enabled
            ? `Model turns System 1 took instead of Gemini. Handed to Gemini:\n${handoffReasons || "none yet"}`
            : "System 1 is off: Gemini takes every turn"
        }
      />
      <Stat
        testId="stat-s1-decider"
        label="S1 decider calls"
        value={s1.enabled ? s1.decider.calls : "off"}
        detail={
          s1.enabled
            ? `${s1.model} · avg ${formatMs(s1.decider.latency.avg_ms)} · p95 ${formatMs(s1.decider.latency.p95_ms)}`
            : "no decision model"
        }
      />
      <Stat
        testId="stat-goals"
        label="Goals done / min"
        value={formatRate(goals.done_per_min)}
        detail={`${goals.started} started · ${goals.done} done · ${goals.abandoned} abandoned`}
        title={`Goals started: ${formatRate(goals.started_per_min)} per minute`}
      />
      <Stat
        testId="stat-ticks"
        label="Tick time"
        value={formatMs(ticks.latency.avg_ms)}
        detail={`${plural(ticks.count, "tick")} · last ${formatMs(ticks.latency.last_ms)}${ticks.truncated ? ` · ${ticks.truncated} truncated` : ""}`}
        title="Average wall time per tick, driving included"
      />
      <Stat
        testId="stat-driving"
        label="Time driving"
        value={`${drives.driving_pct}%`}
        detail={`${plural(drives.count, "drive")} · ${drives.outcomes.succeeded ?? 0} reached · ${drives.checks} S1 checks`}
        title={`Average drive ${formatMs(drives.latency.avg_ms)}; ${formatSeconds(drives.driving_s)} driven. S1 checks: get_nav_status calls the brain made itself while it waited out drives, no LLM involved.`}
      />
      <Stat
        testId="stat-tools"
        label="Tool calls"
        value={tools.calls}
        detail={`avg ${formatMs(tools.latency.avg_ms)} · ${plural(tools.errors, "error")} · ${tools.refusals} refused`}
      />
      <p className="col-span-2 px-0.5 text-[11px] leading-tight text-muted-foreground" data-testid="stats-since">
        Counting since {clock(stats.started_at)} · first action{" "}
        {stats.first_action_s === null ? "not yet" : `after ${formatSeconds(stats.first_action_s)}`}
      </p>
    </div>
  );
}
