import { useEffect, useState } from "react";

import { cn } from "@/lib/utils";

export const SPINNER_FRAMES = ["|", "/", "-", "\\"] as const;
export const SPINNER_INTERVAL_MS = 125;

interface SpinnerMarkerProps {
  /** Animate while true; a paused or offline pet shows a still frame. */
  running: boolean;
  className?: string;
}

/** The in-progress goal marker: a spinning line inside brackets, `[|] [/] [-] [\]`. */
export function SpinnerMarker({ running, className }: SpinnerMarkerProps) {
  const [frame, setFrame] = useState(0);

  useEffect(() => {
    if (!running) return;
    const id = setInterval(() => setFrame((current) => (current + 1) % SPINNER_FRAMES.length), SPINNER_INTERVAL_MS);
    return () => clearInterval(id);
  }, [running]);

  return (
    <span
      role="img"
      aria-label={running ? "in progress" : "in progress, paused"}
      data-testid="goal-spinner"
      className={cn("inline-block w-[3ch] text-center", className)}
    >
      [{SPINNER_FRAMES[frame]}]
    </span>
  );
}
