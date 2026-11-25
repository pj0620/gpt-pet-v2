"""Offline: the frame store used by the portal."""
from __future__ import annotations

from gpt_pet.frames import FrameStore


def test_versions_bump_per_name_and_listeners_fire() -> None:
    store = FrameStore()
    seen = []
    unsubscribe = store.subscribe(seen.append)
    assert store.versions() == {"camera": 0, "map": 0, "depth": 0, "free": 0}
    first = store.put("camera", b"jpeg-1", "image/jpeg")
    second = store.put("camera", b"jpeg-2", "image/jpeg")
    store.put("map", b"png", "image/png")
    assert (first.version, second.version) == (1, 2)
    assert store.get("camera") == second
    assert store.versions() == {"camera": 2, "map": 1, "depth": 0, "free": 0}
    assert [frame.name for frame in seen] == ["camera", "camera", "map"]
    unsubscribe()
    store.put("camera", b"jpeg-3", "image/jpeg")
    assert len(seen) == 3
    assert store.get("depth") is None


def test_put_if_changed_skips_identical_bytes() -> None:
    store = FrameStore()
    seen = []
    store.subscribe(seen.append)
    first = store.put_if_changed("map", b"png-1", "image/png")
    same = store.put_if_changed("map", b"png-1", "image/png")
    changed = store.put_if_changed("map", b"png-2", "image/png")
    assert (first.version, same.version, changed.version) == (1, 1, 2)
    assert len(seen) == 2
