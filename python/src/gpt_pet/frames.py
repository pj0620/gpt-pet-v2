"""Latest camera/map frames captured from MCP tool results, for the portal.

The executor's after-tool callback stores the image bytes here; the server reads them and
announces new versions over SSE. Frames never travel inside the event stream.
"""
from __future__ import annotations

import threading
from collections.abc import Callable
from dataclasses import dataclass

IMAGE_TOOLS: dict[str, str] = {
    "get_current_view": "camera",
    "get_map": "map",
    "get_depth_view": "depth",
}
"""MCP tool name -> portal image name."""

IMAGE_NAMES = ("camera", "map", "depth", "free")


@dataclass(frozen=True)
class Frame:
    name: str
    data: bytes
    mime_type: str
    version: int


Listener = Callable[[Frame], None]


class FrameStore:
    """Thread-safe latest-frame-per-name store with version counters and listeners."""

    def __init__(self) -> None:
        self._frames: dict[str, Frame] = {}
        self._listeners: list[Listener] = []
        self._lock = threading.Lock()

    def put(self, name: str, data: bytes, mime_type: str) -> Frame:
        with self._lock:
            previous = self._frames.get(name)
            frame = Frame(name=name, data=data, mime_type=mime_type, version=(previous.version if previous else 0) + 1)
            self._frames[name] = frame
            listeners = list(self._listeners)
        for listener in listeners:
            listener(frame)
        return frame

    def put_if_changed(self, name: str, data: bytes, mime_type: str) -> Frame:
        """Like `put`, but identical bytes keep the current version and notify nobody."""
        with self._lock:
            current = self._frames.get(name)
        if current is not None and current.data == data:
            return current
        return self.put(name, data, mime_type)

    def get(self, name: str) -> Frame | None:
        with self._lock:
            return self._frames.get(name)

    def versions(self) -> dict[str, int]:
        with self._lock:
            return {name: (self._frames[name].version if name in self._frames else 0) for name in IMAGE_NAMES}

    def subscribe(self, listener: Listener) -> Callable[[], None]:
        with self._lock:
            self._listeners.append(listener)

        def unsubscribe() -> None:
            with self._lock:
                if listener in self._listeners:
                    self._listeners.remove(listener)

        return unsubscribe
