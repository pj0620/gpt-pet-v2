"""Offline: stopping a launched simulator's process group, on real processes (no simulator)."""
from __future__ import annotations

import os
import subprocess
import time

from gpt_pet.simlaunch import stop_mcp_server


def group(script: str) -> subprocess.Popen[bytes]:
    """A process group like the one `launch_mcp_server` starts: a leader in its own session."""
    return subprocess.Popen(["/bin/sh", "-c", script], start_new_session=True)


def group_gone(leader: int) -> bool:
    try:
        os.killpg(leader, 0)
    except ProcessLookupError:
        return True
    except PermissionError:  # macOS: only zombies left
        return True
    return False


def test_a_group_whose_leader_already_exited_unreaped_stops_cleanly() -> None:
    process = group("exit 0")
    time.sleep(0.5)  # the leader is now an unreaped zombie: macOS answers EPERM to signals
    stop_mcp_server(process)
    assert process.returncode == 0 and group_gone(process.pid)


def test_the_whole_group_stops_children_included() -> None:
    process = group("sleep 60 & wait")
    time.sleep(0.3)
    stop_mcp_server(process, timeout_s=5)
    assert process.returncode is not None and group_gone(process.pid)


def test_a_group_that_ignores_sigterm_is_killed() -> None:
    process = group("trap '' TERM; sleep 60 & wait")
    time.sleep(0.3)
    started = time.monotonic()
    stop_mcp_server(process, timeout_s=0.5)
    assert process.returncode is not None and group_gone(process.pid)
    assert time.monotonic() - started < 5
    stop_mcp_server(None)  # nothing launched: nothing to do
