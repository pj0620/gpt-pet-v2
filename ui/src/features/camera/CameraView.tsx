import { Camera } from "lucide-react";

import { useStatusQuery } from "@/api/queries";
import { PanelCard } from "@/components/PanelCard";
import { Button } from "@/components/ui/button";
import { FreeCameraView } from "@/features/camera/FreeCameraView";
import { LiveImage } from "@/features/camera/LiveImage";
import { type CameraMode, usePortalStore } from "@/stores/portalStore";

/** Depth needs a `get_depth_view` MCP tool on the server side; until then that mode is inert. */
const DEPTH_AVAILABLE = false;

const MODES: { mode: CameraMode; label: string }[] = [
  { mode: "camera", label: "Robot" },
  { mode: "depth", label: "Depth" },
  { mode: "free", label: "Free" },
];

/** The camera panel's source switch: the robot's camera, depth (later), and, in the simulator
 * only, a free camera you can fly around the room. */
export function CameraView() {
  const mode = usePortalStore((store) => store.cameraMode);
  const setMode = usePortalStore((store) => store.setCameraMode);
  const status = useStatusQuery();
  const freeAvailable = status.data?.features?.free_camera === true;
  const effective: CameraMode =
    mode === "free" && !freeAvailable ? "camera" : mode === "depth" && !DEPTH_AVAILABLE ? "camera" : mode;

  const modes = (
    <fieldset className="m-0 flex items-center gap-1 border-0 p-0">
      <legend className="sr-only">Camera source</legend>
      {MODES.filter((entry) => entry.mode !== "free" || freeAvailable).map((entry) => {
        const disabled = entry.mode === "depth" && !DEPTH_AVAILABLE;
        const title = disabled
          ? "Depth view needs a get_depth_view tool on the MCP server."
          : entry.mode === "free"
            ? "Simulator only: fly a camera around the room"
            : undefined;
        return (
          <Button
            key={entry.mode}
            size="xs"
            variant={effective === entry.mode ? "default" : "outline"}
            aria-pressed={effective === entry.mode}
            disabled={disabled}
            title={title}
            onClick={() => setMode(entry.mode)}
          >
            {entry.label}
          </Button>
        );
      })}
    </fieldset>
  );

  return (
    <PanelCard title="Camera View" icon={<Camera className="size-4" />} actions={modes}>
      {effective === "free" ? (
        <FreeCameraView />
      ) : (
        <LiveImage
          name={effective === "depth" ? "depth" : "camera"}
          alt={effective === "depth" ? "Depth view from the robot" : "Current view from the robot's camera"}
          testId="camera-image"
          placeholder="Current view from the camera appears here after the first get_current_view call."
        />
      )}
    </PanelCard>
  );
}
