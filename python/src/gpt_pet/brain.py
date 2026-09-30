"""The brain: one tick of the pet's life as an ADK Workflow.

    goal_setter -> update_goal_memory -> executor

Each node's output is the next node's input: the decision, then the active goal record. The
continuous loop over ticks lives outside ADK (see `gpt_pet.runtime` and `gpt_pet.cli`), which is
where a future System 1 scheduler will go.
"""
from __future__ import annotations

from dataclasses import dataclass

from google.adk.tools.mcp_tool.mcp_toolset import McpToolset
from google.adk.workflow import START, FunctionNode, Workflow

from gpt_pet.agents.executor import build_executor
from gpt_pet.agents.goal_setter import build_goal_setter
from gpt_pet.frames import FrameStore
from gpt_pet.goals import make_update_goal_memory
from gpt_pet.mcp import build_toolset
from gpt_pet.settings import Settings

BRAIN_NAME = "gpt_pet"
MEMORY_NODE_NAME = "update_goal_memory"


@dataclass
class Brain:
    workflow: Workflow
    toolset: McpToolset
    """Kept for explicit cleanup: Runner.close() does not close toolsets under a Workflow root."""
    settings: Settings
    frames: FrameStore
    """Latest camera/map frames captured from tool results (used by `gpt-pet serve`)."""


def build_brain(settings: Settings, frames: FrameStore | None = None) -> Brain:
    frames = frames or FrameStore()
    toolset = build_toolset(settings.mcp)
    goal_setter = build_goal_setter(settings)
    memory = FunctionNode(func=make_update_goal_memory(settings.brain), name=MEMORY_NODE_NAME)
    executor = build_executor(settings, toolset, frames)
    workflow = Workflow(
        name=BRAIN_NAME,
        description="One tick of GPTPet's life: decide a goal, store it, execute it.",
        edges=[(START, goal_setter, memory, executor)],
    )
    return Brain(workflow=workflow, toolset=toolset, settings=settings, frames=frames)
