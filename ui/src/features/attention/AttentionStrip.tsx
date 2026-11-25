import { Eye, EyeOff, Footprints, Search } from "lucide-react";

import { Badge } from "@/components/ui/badge";
import { cn } from "@/lib/utils";
import { usePortalStore } from "@/stores/portalStore";
import type { AttentionState, AttentionStatus } from "@/types/api";

export const ACTIVE_ATTENTION: readonly AttentionState[] = ["aiming", "approaching", "framed", "searching"];

function where(status: AttentionStatus): string[] {
  const parts: string[] = [];
  if (status.distance_m !== null) parts.push(`${status.distance_m.toFixed(1)} m`);
  if (status.bearing_deg !== null) {
    parts.push(
      Math.abs(status.bearing_deg) < 1
        ? "straight ahead"
        : `${Math.abs(status.bearing_deg).toFixed(0)}° ${status.bearing_deg > 0 ? "right" : "left"}`,
    );
  }
  return parts;
}

/** One line a viewer can read: what the robot is doing about the person and where they are. */
export function describeAttention(status: AttentionStatus): string {
  const label = status.label ?? "the person";
  const place = where(status);
  switch (status.state) {
    case "framed":
      return [`Watching ${label}`, ...place, `in view ${status.seen_s.toFixed(0)} s`].join(" · ");
    case "aiming":
      return [`Turning toward ${label}`, ...place].join(" · ");
    case "approaching":
      return [`Walking up to ${label}`, ...place, `stops ${status.standoff_m} m away`].join(" · ");
    case "searching":
      return `Looking for ${label} · out of sight ${status.unseen_s.toFixed(0)} s`;
    case "lost":
      return `Lost sight of ${label} after ${status.unseen_s.toFixed(0)} s`;
    default:
      return status.reason ? `Attention ended: ${status.reason}` : "Attention off";
  }
}

const TITLE: Record<AttentionState, string> = {
  off: "The robot is not attending to anyone.",
  aiming: "The robot is turning on its own to centre the person in its camera.",
  approaching: "The person stood still, so the robot is walking up to them (and levels its camera to drive).",
  framed: "The person is centred with the camera tilted for their distance: the brain sees them properly.",
  searching: "The person left the frame; the robot turns toward where they were for a few seconds.",
  lost: "The person stayed out of sight for 5 s, so the robot stopped attending.",
};

/** A strip over the camera view saying what the robot's attention is doing; hidden while off. */
export function AttentionStrip() {
  const attention = usePortalStore((store) => store.attention);
  if (attention === null || attention.state === "off") return null;
  const framed = attention.state === "framed";
  const ended = attention.state === "lost";
  const Icon =
    attention.state === "approaching"
      ? Footprints
      : attention.state === "searching"
        ? Search
        : attention.state === "lost"
          ? EyeOff
          : Eye;
  return (
    <div
      className={cn(
        "flex shrink-0 items-center gap-2 px-3 py-1 text-xs",
        framed
          ? "bg-fuchsia-600 text-white"
          : ended
            ? "bg-muted text-muted-foreground"
            : "bg-fuchsia-600/20 text-fuchsia-200",
      )}
      data-testid="attention-strip"
      data-state={attention.state}
      title={TITLE[attention.state]}
    >
      <Icon className={cn("size-3.5 shrink-0", attention.state === "aiming" && "animate-pulse")} aria-hidden />
      <span className="min-w-0 flex-1 truncate">{describeAttention(attention)}</span>
      {attention.moving ? (
        <Badge variant="outline" className="shrink-0 border-current text-[10px] text-inherit">
          moving
        </Badge>
      ) : null}
      {attention.tilt_deg ? (
        <Badge
          variant="outline"
          className="shrink-0 border-current text-[10px] text-inherit"
          title="The robot's camera is tilted up to see faces and hands"
        >
          camera up {attention.tilt_deg.toFixed(0)}°
        </Badge>
      ) : null}
    </div>
  );
}
