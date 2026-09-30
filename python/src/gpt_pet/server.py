"""`gpt-pet serve`: the pet's tick loop plus the portal's HTTP API in one process.

Endpoints (all under /api): `events` (SSE), `state`, `status`, `stats`, `stats/reset`,
`settings`, `goals`, `control/{pause|resume|tick}`, `profile`, `frame.jpg`, `map.png`,
`depth.png`. The built portal (`ui/dist`) is served at `/` when present.
"""
from __future__ import annotations

import asyncio
import json
import logging
from collections import deque
from collections.abc import AsyncIterator
from contextlib import asynccontextmanager
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Literal

from fastapi import FastAPI, HTTPException, Request
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import Response
from fastapi.staticfiles import StaticFiles
from google.adk.events.event import Event
from pydantic import BaseModel, Field
from sse_starlette.sse import EventSourceResponse

import gpt_pet
from gpt_pet.brain import build_brain
from gpt_pet.frames import Frame, FrameStore
from gpt_pet.mcp_proxy import McpProxy, ToolResult
from gpt_pet.runtime import PetRuntime, TickResult
from gpt_pet.settings import Settings, load_settings

log = logging.getLogger("gpt_pet.server")

IMAGE_ROUTES: dict[str, str] = {"frame.jpg": "camera", "map.png": "map", "depth.png": "depth", "free.jpg": "free"}
MAP_REFRESH_ACTIVE_S = 1.0
"""Map re-render period while a tick runs (the robot may be driving between LLM calls)."""
MAP_REFRESH_IDLE_S = 2.0
"""Map re-render period otherwise (the navigator can still be finishing a goal). `get_map`
renders from the simulator's last event without stepping it, so this stays cheap."""
CAMERA_PRESETS = ("chase", "front", "top")
HISTORY_SIZE = 500
STATS_THROTTLE_S = 1.0
"""At most one `stats` frame per second while counters change."""
STATS_HEARTBEAT_S = 5.0
"""A `stats` frame at least this often, so uptime and per-minute rates keep moving when idle."""
DEV_ORIGINS = ["http://localhost:5173", "http://127.0.0.1:5173"]


def default_ui_dist() -> Path:
    # src/gpt_pet/__init__.py -> gpt_pet -> src -> python -> gpt-pet-v2
    return Path(gpt_pet.__file__).resolve().parents[3] / "ui" / "dist"


def _jsonable(value: Any) -> Any:
    return json.loads(json.dumps(value, default=str))


def event_payloads(event: Event, tick: int) -> list[dict[str, Any]]:
    """The portal's `adk` payloads for one ADK event: one per tool call/response, plus text."""
    if event.partial or event.content is None:
        return []
    base = {"tick": tick, "author": event.author, "timestamp": event.timestamp, "invocation_id": event.invocation_id}
    s1 = (event.custom_metadata or {}).get("s1")
    if s1:
        base["s1"] = s1  # System 1 took this turn instead of Gemini
    payloads: list[dict[str, Any]] = []
    for call in event.get_function_calls():
        payloads.append({**base, "functionCall": {"id": call.id or "", "name": call.name or "", "args": _jsonable(dict(call.args or {}))}})
    for response in event.get_function_responses():
        payloads.append(
            {**base, "functionResponse": {"id": response.id or "", "name": response.name or "", "response": _jsonable(response.response)}}
        )
    text = "".join(part.text for part in (event.content.parts or []) if part.text and not part.thought).strip()
    if text and not payloads:
        payloads.append({**base, "text": text})
    return payloads


@dataclass(frozen=True)
class SseFrame:
    id: int
    event: str
    data: str

    def as_dict(self) -> dict[str, str]:
        return {"id": str(self.id), "event": self.event, "data": self.data}


class Broadcaster:
    """Fan-out of SSE frames to connected clients, with a replay buffer for `Last-Event-ID`."""

    def __init__(self, history: int = HISTORY_SIZE) -> None:
        self._subscribers: set[asyncio.Queue[SseFrame]] = set()
        self._history: deque[SseFrame] = deque(maxlen=history)
        self._next_id = 1

    @property
    def last_id(self) -> int:
        return self._next_id - 1

    def publish(self, event: str, data: Any, *, keep: bool = True) -> SseFrame:
        """Send a frame to every subscriber. `keep=False` skips the replay buffer, for frequent
        snapshots that would otherwise push the event log out of it."""
        frame = SseFrame(id=self._next_id, event=event, data=json.dumps(data, default=str))
        self._next_id += 1
        if keep:
            self._history.append(frame)
        for queue in list(self._subscribers):
            queue.put_nowait(frame)
        return frame

    def subscribe(self) -> asyncio.Queue[SseFrame]:
        queue: asyncio.Queue[SseFrame] = asyncio.Queue()
        self._subscribers.add(queue)
        return queue

    def unsubscribe(self, queue: asyncio.Queue[SseFrame]) -> None:
        self._subscribers.discard(queue)

    def replay(self, after_id: int | None) -> list[SseFrame]:
        if after_id is None:
            return []
        return [frame for frame in self._history if frame.id > after_id]


class PetService:
    """Owns the running `PetRuntime`, swaps it on profile change, and feeds the broadcaster."""

    def __init__(self, settings: Settings, broadcaster: Broadcaster, *, start_paused: bool = False, refresh_map: bool = True) -> None:
        self.settings = settings
        self.broadcaster = broadcaster
        self.frames = FrameStore()
        self.runtime: PetRuntime | None = None
        self._task: asyncio.Task[int] | None = None
        self._lock = asyncio.Lock()
        self._loop: asyncio.AbstractEventLoop | None = None
        self._start_paused = start_paused
        self._refresh_map = refresh_map
        self.proxy: McpProxy | None = None
        self.camera_pose: dict[str, Any] | None = None
        self._map_task: asyncio.Task[None] | None = None
        self._stats_task: asyncio.Task[None] | None = None
        self._stats_handle: asyncio.TimerHandle | None = None
        self.frames.subscribe(self._on_frame)

    # --- lifecycle -------------------------------------------------------------------------

    async def start(self) -> None:
        async with self._lock:
            await self._start_runtime()

    async def stop(self) -> None:
        async with self._lock:
            await self._stop_runtime()

    async def _start_runtime(self) -> None:
        self._loop = asyncio.get_running_loop()
        url = getattr(self.settings.mcp, "url", None)
        self.proxy = McpProxy(str(url)) if url is not None else None
        self.camera_pose = None
        brain = build_brain(self.settings, frames=self.frames)
        brain.stats.on_change = self._stats_changed
        runtime = PetRuntime(
            brain,
            user_id="portal",
            on_event=self._on_event,
            on_tick=self._on_tick,
            on_status=self.publish_status,
            on_notice=self._on_notice,
        )
        await runtime.start()
        if self._start_paused:
            runtime.pause()
        self.runtime = runtime
        self._task = asyncio.create_task(runtime.run_loop(), name="gpt-pet-loop")
        if self._refresh_map and self.proxy is not None:
            self._map_task = asyncio.create_task(self._map_loop(), name="map-refresh")
        self._stats_task = asyncio.create_task(self._stats_loop(), name="stats-heartbeat")
        log.info("pet loop started (profile=%s, paused=%s)", self.settings.profile.name, runtime.paused)
        self.publish_status()

    async def _stop_runtime(self) -> None:
        runtime, task = self.runtime, self._task
        self.runtime, self._task = None, None
        if self._stats_handle is not None:
            self._stats_handle.cancel()
            self._stats_handle = None
        for background in (self._map_task, self._stats_task):
            if background is None:
                continue
            background.cancel()
            try:
                await background
            except (asyncio.CancelledError, Exception):  # noqa: BLE001
                pass
        self._map_task, self._stats_task = None, None
        if runtime is None:
            return
        runtime.stop()
        if task is not None:
            task.cancel()
            try:
                await task
            except (asyncio.CancelledError, Exception):  # noqa: BLE001 - the loop is being torn down
                pass
        await runtime.close()
        if self.proxy is not None:
            await self.proxy.close()
            self.proxy = None
        log.info("pet loop stopped")

    async def switch_profile(self, name: str) -> None:
        async with self._lock:
            settings = load_settings(name)
            await self._stop_runtime()
            self.settings = settings
            await self._start_runtime()

    # --- views -----------------------------------------------------------------------------

    def status(self) -> dict[str, Any]:
        runtime = self.runtime
        mcp = self.settings.mcp
        return {
            "profile": self.settings.profile.name,
            "paused": runtime.paused if runtime else True,
            "tick": runtime.ticks if runtime else 0,
            "running_tick": runtime.running_tick if runtime else False,
            "mcp_url": str(getattr(mcp, "url", "") or getattr(mcp, "command", "")),
            "last_tick_seconds": round(runtime.last_tick.seconds, 2) if runtime and runtime.last_tick else None,
            "waiting_s": round(runtime.waiting_s, 1) if runtime and runtime.waiting_s is not None else None,
            "goal_limit_reached": runtime.limit_reached if runtime else False,
            "goal_limit": runtime.goal_limit if runtime else None,
            "goals_used": runtime.goals_used if runtime else 0,
            "image_versions": self.frames.versions(),
            "features": {"free_camera": self.free_camera_enabled},
        }

    @property
    def free_camera_enabled(self) -> bool:
        return bool(self.settings.features.free_camera and self.proxy is not None)

    async def camera_call(self, tool: str, args: dict[str, Any] | None = None) -> dict[str, Any]:
        """Drive the simulator's free camera through the proxy; stores the frame as `free`."""
        if not self.free_camera_enabled or self.proxy is None:
            raise HTTPException(status_code=404, detail="the free camera exists only in the simulator profile")
        try:
            result = await self.proxy.call(tool, args)
        except Exception as exc:  # noqa: BLE001
            raise HTTPException(status_code=502, detail=f"{tool} failed: {exc}") from exc
        if result.is_error:
            raise HTTPException(status_code=502, detail=result.text or f"{tool} failed")
        if result.image:
            frame = self.frames.put("free", result.image, result.mime_type or "image/jpeg")
            version = frame.version
        else:
            version = self.frames.versions()["free"]
        if result.data:
            self.camera_pose = result.data
        return {"pose": self.camera_pose, "version": version}

    async def state(self) -> dict[str, Any]:
        return await self.runtime.state() if self.runtime else {}

    def stats(self) -> dict[str, Any]:
        return self.runtime.stats.snapshot() if self.runtime else {}

    def require_runtime(self) -> PetRuntime:
        if self.runtime is None:
            raise HTTPException(status_code=503, detail="the pet is not running")
        return self.runtime

    # --- hooks -----------------------------------------------------------------------------

    def publish_status(self) -> None:
        self.broadcaster.publish("status", self.status())

    async def publish_state(self) -> None:
        self.broadcaster.publish("state", await self.state())

    def _on_event(self, event: Event, tick: int) -> None:
        for payload in event_payloads(event, tick):
            self.broadcaster.publish("adk", payload)

    def _on_notice(self, notice: dict[str, Any]) -> None:
        self.broadcaster.publish("notice", notice)

    def publish_stats(self) -> None:
        self._stats_handle = None
        if self.runtime is not None:
            self.broadcaster.publish("stats", self.stats(), keep=False)

    def _stats_changed(self) -> None:
        """Counters moved: publish soon, coalescing bursts (a tick records dozens of calls)."""
        loop = self._loop
        if self._stats_handle is None and loop is not None and not loop.is_closed():
            self._stats_handle = loop.call_later(STATS_THROTTLE_S, self.publish_stats)

    async def _stats_loop(self) -> None:
        while True:
            await asyncio.sleep(STATS_HEARTBEAT_S)
            self.publish_stats()

    def _on_tick(self, result: TickResult) -> None:
        self.broadcaster.publish("tick", result.summary_dict())
        log.info(result.summary())
        loop = self._loop or asyncio.get_running_loop()
        loop.create_task(self.publish_state())
        if self._refresh_map:
            loop.create_task(self.refresh_map())

    def _on_frame(self, frame: Frame) -> None:
        payload = {"name": frame.name, "version": frame.version}
        loop = self._loop
        if loop is None or loop.is_closed():
            return
        loop.call_soon_threadsafe(self.broadcaster.publish, "image", payload)

    async def refresh_map(self) -> None:
        """Fetch the top-down map directly from the MCP server so the portal's map stays current.
        `get_map` renders from the simulator's last event without stepping it, so this is cheap;
        an unchanged image keeps its version so the portal does not reload it."""
        if self.proxy is None:
            return
        try:
            result: ToolResult = await self.proxy.call("get_map", {})
            if result.image:
                self.frames.put_if_changed("map", result.image, result.mime_type or "image/png")
        except Exception as exc:  # noqa: BLE001 - the map is a nicety, never fatal
            log.warning("map refresh failed: %s", exc)

    async def _map_loop(self) -> None:
        """Keep the map moving while the robot does: fast during a tick, slow otherwise."""
        while True:
            runtime = self.runtime
            active = runtime is not None and runtime.running_tick
            await asyncio.sleep(MAP_REFRESH_ACTIVE_S if active else MAP_REFRESH_IDLE_S)
            await self.refresh_map()


class GoalBody(BaseModel):
    goal: str = Field(min_length=1, max_length=500)
    sub_goals: list[str] = Field(default_factory=list, max_length=3)


class ProfileBody(BaseModel):
    name: Literal["sim", "real"]


class ExtendBody(BaseModel):
    goals: int = Field(ge=1, le=100)


class CameraMoveBody(BaseModel):
    forward: float = Field(0.0, ge=-5, le=5)
    right: float = Field(0.0, ge=-5, le=5)
    up: float = Field(0.0, ge=-5, le=5)
    yaw: float = Field(0.0, ge=-360, le=360)
    pitch: float = Field(0.0, ge=-180, le=180)
    zoom: float = Field(0.0, ge=-90, le=90)


class CameraResetBody(BaseModel):
    mode: Literal["chase", "front", "top"] = "chase"


def create_app(settings: Settings, *, ui_dist: Path | None = None, start_paused: bool = False, refresh_map: bool = True) -> FastAPI:
    broadcaster = Broadcaster()
    service = PetService(settings, broadcaster, start_paused=start_paused, refresh_map=refresh_map)

    @asynccontextmanager
    async def lifespan(_: FastAPI) -> AsyncIterator[None]:
        await service.start()
        try:
            yield
        finally:
            await service.stop()

    app = FastAPI(title="GPTPet", lifespan=lifespan)
    app.state.service = service
    app.state.broadcaster = broadcaster
    app.add_middleware(CORSMiddleware, allow_origins=DEV_ORIGINS, allow_methods=["*"], allow_headers=["*"])

    @app.get("/api/status")
    async def get_status() -> dict[str, Any]:
        return service.status()

    @app.get("/api/state")
    async def get_state() -> dict[str, Any]:
        return await service.state()

    @app.get("/api/stats")
    async def get_stats() -> dict[str, Any]:
        return service.stats()

    @app.post("/api/stats/reset")
    async def post_stats_reset() -> dict[str, Any]:
        """Start the counters over, e.g. to measure one stretch of the run."""
        service.require_runtime().stats.reset()
        return service.stats()

    @app.get("/api/settings")
    async def get_settings() -> dict[str, Any]:
        return service.settings.model_dump(mode="json")

    @app.post("/api/goals")
    async def post_goal(body: GoalBody) -> dict[str, Any]:
        runtime = service.require_runtime()
        await runtime.submit_goal(body.goal, body.sub_goals)
        state = await service.state()
        broadcaster.publish("state", state)
        return state

    # --- simulator-only free camera (404 outside the sim profile) ---

    @app.get("/api/sim/camera")
    async def get_camera() -> dict[str, Any]:
        if not service.free_camera_enabled:
            raise HTTPException(status_code=404, detail="the free camera exists only in the simulator profile")
        if service.camera_pose is None:
            return await service.camera_call("sim_camera_view")
        return {"pose": service.camera_pose, "version": service.frames.versions()["free"]}

    @app.post("/api/sim/camera/refresh")
    async def post_camera_refresh() -> dict[str, Any]:
        return await service.camera_call("sim_camera_view")

    @app.post("/api/sim/camera/move")
    async def post_camera_move(body: CameraMoveBody) -> dict[str, Any]:
        return await service.camera_call("sim_camera_move", body.model_dump())

    @app.post("/api/sim/camera/reset")
    async def post_camera_reset(body: CameraResetBody) -> dict[str, Any]:
        return await service.camera_call("sim_camera_reset", {"mode": body.mode})

    @app.post("/api/control/extend")
    async def post_extend(body: ExtendBody) -> dict[str, Any]:
        """Allow N more goals this run (and resume if the loop paused on the limit)."""
        runtime = service.require_runtime()
        runtime.extend_goal_limit(body.goals)
        return service.status()

    @app.post("/api/control/{action}")
    async def post_control(action: str) -> dict[str, Any]:
        runtime = service.require_runtime()
        if action == "pause":
            runtime.pause()
        elif action == "resume":
            runtime.resume()
        elif action == "tick":
            runtime.request_tick()
        else:
            raise HTTPException(status_code=404, detail=f"unknown control action {action!r}")
        return service.status()

    @app.post("/api/profile")
    async def post_profile(body: ProfileBody) -> dict[str, Any]:
        await service.switch_profile(body.name)
        await service.publish_state()
        return service.status()

    def image_endpoint(name: str):
        async def get_image() -> Response:
            frame = service.frames.get(name)
            if frame is None:
                raise HTTPException(status_code=404, detail=f"no {name} frame yet")
            return Response(
                content=frame.data,
                media_type=frame.mime_type,
                headers={"Cache-Control": "no-store", "X-Frame-Version": str(frame.version)},
            )

        return get_image

    # Explicit paths: a catch-all `/api/{filename}` would swallow `/api/events`.
    for filename, image_name in IMAGE_ROUTES.items():
        app.add_api_route(f"/api/{filename}", image_endpoint(image_name), methods=["GET"], name=f"image_{image_name}")

    @app.get("/api/events")
    async def get_events(request: Request) -> EventSourceResponse:
        queue = broadcaster.subscribe()
        last_event_id = request.headers.get("last-event-id")
        after_id = int(last_event_id) if last_event_id and last_event_id.isdigit() else None

        async def stream() -> AsyncIterator[dict[str, str]]:
            try:
                # Recent history first (all of it for a fresh page, only newer frames on a
                # reconnect), then fresh status/state snapshots so nothing stale wins.
                for frame in broadcaster.replay(after_id if after_id is not None else 0):
                    yield frame.as_dict()
                snapshot_id = str(broadcaster.last_id)
                yield {"id": snapshot_id, "event": "status", "data": json.dumps(service.status(), default=str)}
                yield {"id": snapshot_id, "event": "state", "data": json.dumps(await service.state(), default=str)}
                yield {"id": snapshot_id, "event": "stats", "data": json.dumps(service.stats(), default=str)}
                while True:
                    if await request.is_disconnected():
                        break
                    try:
                        frame = await asyncio.wait_for(queue.get(), timeout=15.0)
                    except asyncio.TimeoutError:
                        continue
                    yield frame.as_dict()
            finally:
                broadcaster.unsubscribe(queue)

        return EventSourceResponse(stream(), ping=15)

    dist = ui_dist if ui_dist is not None else default_ui_dist()
    if (dist / "index.html").is_file():
        app.mount("/", StaticFiles(directory=str(dist), html=True), name="portal")
        log.info("serving the portal from %s", dist)
    else:
        log.info("no built portal at %s; only /api is served", dist)

    return app
