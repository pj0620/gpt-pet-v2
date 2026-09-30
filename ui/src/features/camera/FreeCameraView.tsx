import { ArrowDown, ArrowLeft, ArrowRight, ArrowUp, Minus, Plus } from "lucide-react";
import {
  type KeyboardEvent as ReactKeyboardEvent,
  type ReactNode,
  type PointerEvent as ReactPointerEvent,
  type WheelEvent as ReactWheelEvent,
  useCallback,
  useEffect,
  useRef,
  useState,
} from "react";

import { api, imageUrl } from "@/api/client";
import { Button } from "@/components/ui/button";
import {
  addDelta,
  type CameraDelta,
  compactDelta,
  DRAG_DEG_PER_PX,
  deltaForKey,
  isZeroDelta,
  LOOK_STEP_DEG,
  MOVE_STEP_M,
  WHEEL_DEG_PER_UNIT,
  ZERO_DELTA,
  ZOOM_STEP_DEG,
} from "@/features/camera/cameraDeltas";
import { cn } from "@/lib/utils";
import { usePortalStore } from "@/stores/portalStore";
import type { CameraMoveBody, CameraPose, CameraPreset, CameraResponse } from "@/types/api";

const IDLE_REFRESH_MS = 2000;
const PRESETS: CameraPreset[] = ["chase", "front", "top"];

function describe(error: unknown): string {
  return error instanceof Error ? error.message : String(error);
}

/**
 * Simulator only: a camera you fly around the room. Drag to look, wheel to zoom, WASD / Q / E to
 * move, arrows to look, or use the buttons. Inputs are coalesced into one request at a time.
 */
export function FreeCameraView() {
  const version = usePortalStore((store) => store.imageVersions.free);
  const robotVersion = usePortalStore((store) => store.imageVersions.camera);
  const setImageVersion = usePortalStore((store) => store.setImageVersion);
  const connection = usePortalStore((store) => store.connection);
  const [pose, setPose] = useState<CameraPose | null>(null);
  const [src, setSrc] = useState<string | null>(null);
  const [error, setError] = useState<string | null>(null);
  const [follow, setFollow] = useState(false);
  const pending = useRef<CameraDelta>(ZERO_DELTA);
  const inFlight = useRef(false);
  const drag = useRef<{ x: number; y: number } | null>(null);

  const apply = useCallback(
    (response: CameraResponse) => {
      setPose(response.pose);
      setImageVersion("free", response.version);
      setError(null);
    },
    [setImageVersion],
  );

  // One request at a time; whatever arrives meanwhile is folded into the next one.
  const flush = useCallback(async () => {
    if (inFlight.current || isZeroDelta(pending.current)) return;
    const body = compactDelta(pending.current);
    pending.current = ZERO_DELTA;
    inFlight.current = true;
    try {
      apply(await api.cameraMove(body));
    } catch (exc) {
      setError(describe(exc));
    } finally {
      inFlight.current = false;
      if (!isZeroDelta(pending.current)) void flush();
    }
  }, [apply]);

  const move = useCallback(
    (delta: CameraMoveBody) => {
      pending.current = addDelta(pending.current, delta);
      void flush();
    },
    [flush],
  );

  const reset = useCallback(
    async (mode: CameraPreset) => {
      try {
        apply(await api.cameraReset(mode));
      } catch (exc) {
        setError(describe(exc));
      }
    },
    [apply],
  );

  // First load: current pose and frame.
  useEffect(() => {
    if (connection !== "open") return;
    let cancelled = false;
    api
      .camera()
      .then((response) => {
        if (!cancelled) apply(response);
      })
      .catch((exc: unknown) => {
        if (!cancelled) setError(describe(exc));
      });
    return () => {
      cancelled = true;
    };
  }, [connection, apply]);

  // Follow: re-pose behind the robot whenever the robot delivers a new frame.
  useEffect(() => {
    if (connection !== "open" || !follow || robotVersion === 0) return;
    void reset("chase");
  }, [robotVersion, follow, connection, reset]);

  // Idle refresh so scene changes show even when nobody touches the camera.
  useEffect(() => {
    if (connection !== "open") return;
    const id = setInterval(() => {
      if (!inFlight.current)
        api
          .cameraRefresh()
          .then(apply)
          .catch(() => undefined);
    }, IDLE_REFRESH_MS);
    return () => clearInterval(id);
  }, [connection, apply]);

  // Swap the image only once the new frame has loaded (no flicker).
  useEffect(() => {
    if (version === 0 || connection !== "open") return;
    const url = imageUrl("free", version);
    let cancelled = false;
    const image = new Image();
    image.onload = () => {
      if (!cancelled) setSrc(url);
    };
    image.src = url;
    return () => {
      cancelled = true;
    };
  }, [version, connection]);

  const onPointerDown = (event: ReactPointerEvent<HTMLDivElement>) => {
    drag.current = { x: event.clientX, y: event.clientY };
    event.currentTarget.setPointerCapture(event.pointerId);
    event.currentTarget.focus();
  };
  const onPointerMove = (event: ReactPointerEvent<HTMLDivElement>) => {
    if (!drag.current) return;
    const dx = event.clientX - drag.current.x;
    const dy = event.clientY - drag.current.y;
    drag.current = { x: event.clientX, y: event.clientY };
    move({ yaw: dx * DRAG_DEG_PER_PX, pitch: dy * DRAG_DEG_PER_PX });
  };
  const onPointerUp = (event: ReactPointerEvent<HTMLDivElement>) => {
    drag.current = null;
    event.currentTarget.releasePointerCapture(event.pointerId);
  };
  const onWheel = (event: ReactWheelEvent<HTMLDivElement>) => {
    move({ zoom: -event.deltaY * WHEEL_DEG_PER_UNIT });
  };
  const onKeyDown = (event: ReactKeyboardEvent<HTMLDivElement>) => {
    const delta = deltaForKey(event.key);
    if (delta === null) return;
    event.preventDefault();
    move(delta);
  };

  const control = (label: string, delta: CameraMoveBody, icon: ReactNode) => (
    <Button size="icon-xs" variant="outline" aria-label={label} title={label} onClick={() => move(delta)}>
      {icon}
    </Button>
  );

  return (
    <div className="flex h-full flex-col" data-testid="free-camera" data-version={version}>
      <div
        role="application"
        aria-label="Free camera view: drag to look, scroll to zoom, WASD to move"
        // biome-ignore lint/a11y/noNoninteractiveTabindex: the viewport is a keyboard-driven camera control (WASD, arrows)
        tabIndex={0}
        onPointerDown={onPointerDown}
        onPointerMove={onPointerMove}
        onPointerUp={onPointerUp}
        onPointerCancel={onPointerUp}
        onWheel={onWheel}
        onKeyDown={onKeyDown}
        className={cn(
          "relative flex min-h-0 flex-1 cursor-grab touch-none items-center justify-center bg-black/40 outline-none select-none",
          "focus-visible:ring-2 focus-visible:ring-ring/60 active:cursor-grabbing",
        )}
      >
        {src ? (
          <img
            src={src}
            alt="Free camera view of the room"
            data-testid="free-camera-image"
            className="max-h-full max-w-full object-contain"
            draggable={false}
          />
        ) : (
          <p className="p-3 text-xs text-muted-foreground">{error ?? "Loading the free camera…"}</p>
        )}
        {pose ? (
          <span className="absolute bottom-1 left-2 font-mono text-[10px] text-muted-foreground">
            x {pose.position.x.toFixed(2)} y {pose.position.y.toFixed(2)} z {pose.position.z.toFixed(2)} · yaw{" "}
            {pose.yaw.toFixed(0)}° pitch {pose.pitch.toFixed(0)}° fov {pose.fov.toFixed(0)}°
          </span>
        ) : null}
        {error && src ? <span className="absolute top-1 left-2 text-[10px] text-destructive">{error}</span> : null}
      </div>
      <div className="flex shrink-0 flex-wrap items-center gap-1 border-t border-border px-2 py-1 text-xs">
        <span className="mr-1 text-muted-foreground">move</span>
        {control("Move forward", { forward: MOVE_STEP_M }, <ArrowUp />)}
        {control("Move back", { forward: -MOVE_STEP_M }, <ArrowDown />)}
        {control("Move left", { right: -MOVE_STEP_M }, <ArrowLeft />)}
        {control("Move right", { right: MOVE_STEP_M }, <ArrowRight />)}
        {control("Move up", { up: MOVE_STEP_M }, <span className="text-[10px]">up</span>)}
        {control("Move down", { up: -MOVE_STEP_M }, <span className="text-[10px]">dn</span>)}
        <span className="mx-1 text-muted-foreground">look</span>
        {control("Look left", { yaw: -LOOK_STEP_DEG }, <ArrowLeft />)}
        {control("Look right", { yaw: LOOK_STEP_DEG }, <ArrowRight />)}
        {control("Look up", { pitch: -LOOK_STEP_DEG }, <ArrowUp />)}
        {control("Look down", { pitch: LOOK_STEP_DEG }, <ArrowDown />)}
        <span className="mx-1 text-muted-foreground">zoom</span>
        {control("Zoom in", { zoom: ZOOM_STEP_DEG }, <Plus />)}
        {control("Zoom out", { zoom: -ZOOM_STEP_DEG }, <Minus />)}
        <span className="mx-1 text-muted-foreground">preset</span>
        {PRESETS.map((mode) => (
          <Button key={mode} size="xs" variant="outline" onClick={() => void reset(mode)}>
            {mode}
          </Button>
        ))}
        <label className="ml-auto flex items-center gap-1 text-muted-foreground">
          <input type="checkbox" checked={follow} onChange={(event) => setFollow(event.target.checked)} />
          follow robot
        </label>
      </div>
    </div>
  );
}
