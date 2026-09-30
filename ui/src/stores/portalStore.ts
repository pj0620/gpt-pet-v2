import { create } from "zustand";

import { type EventsState, emptyEvents, ingestEvent } from "@/stores/eventsReducer";
import type { ImageName, PortalEvent, TickEvent } from "@/types/api";

export type ConnectionState = "connecting" | "open" | "closed";
export type CameraMode = "camera" | "depth" | "free";

export interface PortalStore {
  connection: ConnectionState;
  events: EventsState;
  lastTick: TickEvent | null;
  imageVersions: Record<ImageName, number>;
  selectedKey: string | null;
  toolFilter: string | null;
  cameraMode: CameraMode;
  ingest: (event: PortalEvent) => void;
  setConnection: (connection: ConnectionState) => void;
  select: (key: string | null) => void;
  setToolFilter: (name: string | null) => void;
  setCameraMode: (mode: CameraMode) => void;
  setImageVersion: (name: ImageName, version: number) => void;
  clearEvents: () => void;
}

/** Stream-derived data and UI state. Goal/tick/status state lives in the TanStack Query cache. */
export const usePortalStore = create<PortalStore>()((set) => ({
  connection: "closed",
  events: emptyEvents(),
  lastTick: null,
  imageVersions: { camera: 0, map: 0, depth: 0, free: 0 },
  selectedKey: null,
  toolFilter: null,
  cameraMode: "camera",
  ingest: (event) =>
    set((state) => {
      switch (event.type) {
        case "adk":
        case "notice":
          return { events: ingestEvent(state.events, event) };
        case "tick":
          return { events: ingestEvent(state.events, event), lastTick: event };
        case "image":
          return { imageVersions: { ...state.imageVersions, [event.name]: event.version } };
        default:
          return {};
      }
    }),
  setConnection: (connection) => set({ connection }),
  select: (selectedKey) => set({ selectedKey }),
  setToolFilter: (toolFilter) => set({ toolFilter }),
  setCameraMode: (cameraMode) => set({ cameraMode }),
  setImageVersion: (name, version) =>
    set((state) =>
      version > state.imageVersions[name] ? { imageVersions: { ...state.imageVersions, [name]: version } } : {},
    ),
  clearEvents: () => set({ events: emptyEvents(), selectedKey: null }),
}));
