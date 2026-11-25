"""The brain: one tick of the pet's life as an ADK Workflow.

    goal_setter -> update_goal_memory -> executor

Each node's output is the next node's input: the decision, then the active goal record. The
continuous loop over ticks lives outside ADK (see `gpt_pet.runtime` and `gpt_pet.cli`), which is
where a future System 1 scheduler will go.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Callable

from google.adk.tools.mcp_tool.mcp_toolset import McpToolset
from google.adk.workflow import START, FunctionNode, Workflow

from gpt_pet.agents.executor import build_executor
from gpt_pet.agents.fallbacks import executor_on_model_error, goal_setter_on_model_error
from gpt_pet.agents.goal_setter import build_goal_setter
from gpt_pet.frames import FrameStore
from gpt_pet.goals import GoalInterrupt, make_update_goal_memory
from gpt_pet.mcp import build_toolset
from gpt_pet.mcp_proxy import McpProxy
from gpt_pet.people import ATTENTION_TOOLS, People, PersonWatch
from gpt_pet.places import Places
from gpt_pet.s1 import RULES_ONLY, SystemOne
from gpt_pet.settings import Settings, StreamableHttpMcpSettings
from gpt_pet.stats import RunStats

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
    stats: RunStats
    """This run's live statistics; `PetRuntime` attaches the plugin that feeds it."""
    s1: SystemOne | None = None
    """System 1 in front of Gemini, when `[s1] enabled`."""
    places: Places | None = None
    """The pet's memory of places, when `[brain] places_memory`."""
    map_proxy: McpProxy | None = None
    """The goal setter's own MCP session for `get_map`, when `[brain] map_for_goals`."""
    interrupt: GoalInterrupt = field(default_factory=GoalInterrupt)
    """Set by the runtime when the owner's new goal cancels the goal in progress mid-tick."""
    on_view: Callable[..., Any] | None = None
    """Told about views the brain takes outside the tool flow (the person watch), as
    `(by, image_bytes, mime_type, payload, tick)`; the trace recorder listens here."""
    people: People | None = None
    """Who the pet has seen and when it reacted, with `[people] enabled`."""
    watch: PersonWatch | None = None
    """Looks for people during drives and rests, with `[people] watch`."""

    def listen_to_views(self, listener: Callable[..., Any] | None) -> None:
        """Send every view taken outside the tool flow to `listener` (see `on_view`)."""
        self.on_view = listener
        if self.watch is not None:
            self.watch.on_view = listener

    async def close(self) -> None:
        """Close what the brain opened besides the toolset (which `PetRuntime` closes)."""
        if self.map_proxy is not None:
            await self.map_proxy.close()
        if self.watch is not None:
            await self.watch.close()


def cue_reader_model(settings: Settings) -> tuple[str, Any]:
    """The cue reader's model and its settings: `[people] cue_model` (else the executor's), a
    low-detail image (a low-detail crop reads as well as a high-detail one, experiments/people/
    cue_benchmark.py, at a tenth of the image tokens), and the executor's reasoning effort only
    when it is the executor's model (another model may not take the argument at all)."""
    name = settings.people.cue_model or settings.model.executor
    update: dict[str, Any] = {"image_detail": "low"}
    if name != settings.model.executor:
        update["reasoning_effort"] = ""
    return name, settings.model.model_copy(update=update)


def build_brain(settings: Settings, frames: FrameStore | None = None, stats: RunStats | None = None) -> Brain:
    frames = frames or FrameStore()
    stats = stats or RunStats()
    people_on = settings.people.enabled
    social = None
    reader = None
    if people_on and settings.people.engage:
        from gpt_pet.settings import resolve_config_path
        from gpt_pet.social import CueReader, SocialMemory

        memory_path = None
        if settings.people.memory_file:
            memory_path = Path(settings.people.memory_file)
            if not memory_path.is_absolute():
                memory_path = resolve_config_path(settings.profile.name).parent / memory_path
        social = SocialMemory(settings.people, path=memory_path)
        reader = CueReader(*cue_reader_model(settings), stats)
    people = People(settings.people.labels, react_every_s=settings.people.react_every_s, social=social) if people_on else None
    if people is not None:
        people.reader = reader
    people_labels = frozenset(label.lower() for label in settings.people.labels) if people_on else frozenset()
    places = (
        Places(here_m=settings.brain.here_m, failed_guard=settings.brain.failed_guard, people_labels=people_labels)
        if settings.brain.places_memory
        else None
    )
    s1 = SystemOne(settings, stats, places, people) if settings.s1.enabled else None
    stats.s1_model = (RULES_ONLY if settings.s1.rules_only else settings.s1.model) if s1 else None
    if people is not None:
        stats.people_counts = lambda: dict(people.counts)
    mcp = settings.mcp
    if people is not None and mcp.tool_filter:  # `[people]` shows the executor the attention tools
        mcp = mcp.model_copy(update={"tool_filter": [*mcp.tool_filter, *(t for t in ATTENTION_TOOLS if t not in mcp.tool_filter)]})
    toolset = build_toolset(mcp)
    url = str(settings.mcp.url) if isinstance(settings.mcp, StreamableHttpMcpSettings) else None
    watch = PersonWatch(people, settings.people.watch_every_s, frames=frames, url=url) if people is not None and settings.people.watch else None
    map_proxy = None
    if settings.brain.map_for_goals and isinstance(settings.mcp, StreamableHttpMcpSettings):
        map_proxy = McpProxy(str(settings.mcp.url))
    goal_places = places if settings.brain.map_for_goals else None
    goal_setter = build_goal_setter(
        settings,
        s1.goal_setter_turn if s1 else None,
        goal_setter_on_model_error,
        places=goal_places,
        map_proxy=map_proxy,
        on_map=stats.record_map_shown,
        people=people,
    )
    known_target = None
    if goal_places is not None:
        # a goal may target a remembered place, or (`[people]`) a person the pet has seen
        known_target = (lambda id_: id_ in goal_places.destinations or id_ in people.last) if people is not None else goal_places.destinations.__contains__
    explore_done = settings.s1.enabled and settings.s1.explore_ticks > 0
    memory = FunctionNode(func=make_update_goal_memory(settings.brain, known_target, explore_done=explore_done), name=MEMORY_NODE_NAME)
    interrupt = GoalInterrupt()
    executor = build_executor(
        settings, toolset, frames, stats, s1.executor_turn if s1 else None, executor_on_model_error, places, interrupt, people, watch,
        reader,
    )
    workflow = Workflow(
        name=BRAIN_NAME,
        description="One tick of GPTPet's life: decide a goal, store it, execute it.",
        edges=[(START, goal_setter, memory, executor)],
    )
    return Brain(
        workflow=workflow,
        toolset=toolset,
        settings=settings,
        frames=frames,
        stats=stats,
        s1=s1,
        places=places,
        map_proxy=map_proxy,
        interrupt=interrupt,
        people=people,
        watch=watch,
    )
