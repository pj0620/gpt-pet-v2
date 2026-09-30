/** Compact counts: 999, 1.2k, 34.5k, 123k, 1.2M. */
export function formatCount(value: number): string {
  const abs = Math.abs(value);
  if (abs < 1_000) return String(Math.round(value));
  if (abs < 1_000_000) return `${shorten(value / 1_000)}k`;
  return `${shorten(value / 1_000_000)}M`;
}

function shorten(value: number): string {
  return Math.abs(value) < 100 ? String(Number(value.toFixed(1))) : String(Math.round(value));
}

/** Milliseconds as "850 ms", "4.4 s", "1m 05s"; "—" before the first sample. */
export function formatMs(ms: number | null | undefined): string {
  if (ms === null || ms === undefined) return "—";
  if (ms < 1_000) return `${Math.round(ms)} ms`;
  return formatSeconds(ms / 1_000);
}

/** Seconds as "4.4 s", "42 s", "1m 05s", "2h 03m". */
export function formatSeconds(seconds: number): string {
  if (seconds < 10) return `${Number(seconds.toFixed(1))} s`;
  if (seconds < 60) return `${Math.round(seconds)} s`;
  const minutes = Math.floor(seconds / 60);
  if (minutes < 60) return `${minutes}m ${String(Math.floor(seconds % 60)).padStart(2, "0")}s`;
  return `${Math.floor(minutes / 60)}h ${String(minutes % 60).padStart(2, "0")}m`;
}

/** Per-minute rates: 0.38, 12.5, 3.3k. */
export function formatRate(value: number): string {
  if (value >= 1_000) return formatCount(value);
  return String(Number(value.toFixed(value < 10 ? 2 : 1)));
}
