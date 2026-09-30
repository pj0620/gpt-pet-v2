import type {
  CameraMoveBody,
  CameraPreset,
  CameraResponse,
  ControlAction,
  ImageName,
  PetState,
  PetStatus,
  ProfileName,
  Settings,
  SubmitGoalBody,
} from "@/types/api";

export class ApiError extends Error {
  readonly status: number;

  constructor(status: number, message: string) {
    super(message);
    this.name = "ApiError";
    this.status = status;
  }
}

async function fetchJson<T>(input: string, init?: RequestInit): Promise<T> {
  const response = await fetch(input, {
    ...init,
    headers: { Accept: "application/json", ...(init?.headers ?? {}) },
  });
  if (!response.ok) {
    throw new ApiError(response.status, `${response.status} ${response.statusText} for ${input}`);
  }
  return (await response.json()) as T;
}

function postJson<T>(input: string, body?: unknown): Promise<T> {
  return fetchJson<T>(input, {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: body === undefined ? undefined : JSON.stringify(body),
  });
}

/** The pet server's REST surface. Everything streams through `/api/events` afterwards. */
export const api = {
  getState: () => fetchJson<PetState>("/api/state"),
  getStatus: () => fetchJson<PetStatus>("/api/status"),
  getSettings: () => fetchJson<Settings>("/api/settings"),
  postGoal: (body: SubmitGoalBody) => postJson<PetState>("/api/goals", body),
  control: (action: ControlAction) => postJson<PetStatus>(`/api/control/${action}`),
  extendGoals: (goals: number) => postJson<PetStatus>("/api/control/extend", { goals }),
  setProfile: (name: ProfileName) => postJson<PetStatus>("/api/profile", { name }),
  // simulator-only free camera (404 outside the sim profile)
  camera: () => fetchJson<CameraResponse>("/api/sim/camera"),
  cameraRefresh: () => postJson<CameraResponse>("/api/sim/camera/refresh"),
  cameraMove: (body: CameraMoveBody) => postJson<CameraResponse>("/api/sim/camera/move", body),
  cameraReset: (mode: CameraPreset) => postJson<CameraResponse>("/api/sim/camera/reset", { mode }),
};

const IMAGE_FILES: Record<ImageName, string> = {
  camera: "frame.jpg",
  map: "map.png",
  depth: "depth.png",
  free: "free.jpg",
};

/** Cache-busted image URL; the server also sends `Cache-Control: no-store`. */
export function imageUrl(name: ImageName, version: number): string {
  return `/api/${IMAGE_FILES[name]}?v=${version}`;
}
