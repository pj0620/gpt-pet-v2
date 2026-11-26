"""Profile settings: .ini files validated into pydantic models.

Two profiles ship inside the package (`gpt_pet/config/sim.ini` and `real.ini`).

Profile selection order: explicit argument, then the `GPTPET_PROFILE` environment variable,
then `sim`. Directory order: explicit argument, then `GPTPET_CONFIG_DIR`, then the package's
`config` directory. Every model forbids unknown keys, so a typo in an ini file fails loudly.
"""
from __future__ import annotations

import configparser
import os
import shlex
from importlib import resources
from pathlib import Path
from typing import Annotated, Any, Literal

from pydantic import BaseModel, ConfigDict, Field, HttpUrl, field_validator, model_validator

DEFAULT_PROFILE = "sim"
PROFILE_ENV = "GPTPET_PROFILE"
CONFIG_DIR_ENV = "GPTPET_CONFIG_DIR"


class _Strict(BaseModel):
    model_config = ConfigDict(extra="forbid")


class ProfileSettings(_Strict):
    name: Literal["sim", "real"]


class ModelSettings(_Strict):
    goal_setter: str = "gemini-3.5-flash"
    executor: str = "gemini-3.5-flash"
    temperature: float = Field(0.2, ge=0, le=2)
    request_timeout_s: float = Field(20, ge=0)
    """Give up on a Gemini call after this long (0 = never). A timed-out call costs one turn, not the
    run: the agents' fallbacks stand in for it (`gpt_pet.agents.fallbacks`). The other providers'
    models (`gpt_pet.agents.models`) are bounded by it too."""
    reasoning_effort: Literal["", "none", "minimal", "low", "medium", "high"] = ""
    """How hard a reasoning model thinks before it answers; empty keeps each API's default. Gemini
    takes it as its thinking level (`minimal` .. `high`), OpenAI's reasoning models as
    `reasoning_effort` (and then get no temperature, which they reject)."""
    image_detail: Literal["", "low", "medium", "high"] = ""
    """How finely the model sees each image (camera frames, the map); empty keeps each API's
    default. Gemini takes it as its media resolution (a 640x480 frame: ~1,100 tokens by default,
    ~300 at low), OpenAI as each image's `detail` (`medium` sends `auto`); Claude and local
    models ignore it."""
    max_output_tokens: int = Field(2048, ge=16)
    """The most tokens an OpenAI-compatible or Anthropic model may write per call (reasoning
    included); Gemini keeps its own default."""
    ollama_url: HttpUrl = Field(default=HttpUrl("http://127.0.0.1:11434"))
    """Serves the models named `ollama/<tag>` in `goal_setter` / `executor` (`gpt_pet.agents.ollama`)."""
    ollama_num_ctx: int = Field(16384, ge=2048)
    """A local model's context window. Ollama silently drops the start of a longer prompt, so it
    must hold the executor's largest turn (instructions, tools, up to four camera frames)."""
    ollama_num_predict: int = Field(1024, ge=16)
    """The most tokens a local model may write per call, so a model stuck repeating itself stops."""
    ollama_keep_alive: str = "30m"
    """How long Ollama keeps a local model loaded after its last call."""
    ollama_think: bool = False
    """Let a local model that can think do so before it answers (slower). Ignored by the others."""


def _split_csv(value: Any) -> Any:
    if isinstance(value, str):
        return [item.strip() for item in value.split(",") if item.strip()]
    return value


class _McpBase(_Strict):
    tool_filter: list[str] = Field(default_factory=list)
    """MCP tool names the agents may see. Empty means every tool the server exposes."""
    tool_list_cache_ttl_s: float | None = 600
    """How long ADK caches the server's tool list; None re-lists on every LLM turn."""

    @field_validator("tool_filter", mode="before")
    @classmethod
    def _parse_tool_filter(cls, value: Any) -> Any:
        return _split_csv(value)


class StdioMcpSettings(_McpBase):
    """Spawn the MCP server as a child process and talk over stdio."""

    transport: Literal["stdio"]
    command: str
    args: list[str] = Field(default_factory=list)
    cwd: Path | None = None
    timeout_s: float = 300
    """Connect timeout AND per-tool-call timeout (ADK uses one value for both over stdio)."""

    @field_validator("args", mode="before")
    @classmethod
    def _parse_args(cls, value: Any) -> Any:
        if isinstance(value, str):
            return shlex.split(value)
        return value


class _HttpMcpBase(_McpBase):
    url: HttpUrl
    connect_timeout_s: float = 30
    read_timeout_s: float = 300
    """Per-tool-call timeout. The simulator's first call boots Unity, so keep this generous."""


class StreamableHttpMcpSettings(_HttpMcpBase):
    transport: Literal["streamable_http"]


class SseMcpSettings(_HttpMcpBase):
    transport: Literal["sse"]


McpSettings = Annotated[
    StdioMcpSettings | StreamableHttpMcpSettings | SseMcpSettings,
    Field(discriminator="transport"),
]


class BrainSettings(_Strict):
    default_goal: str = "explore your surroundings and find something interesting"
    max_llm_calls_per_tick: int = Field(20, ge=2)
    goal_history_limit: int = Field(10, ge=1)
    max_goal_attempts: int = Field(3, ge=1)
    """Executor stretches a goal may consume before it is abandoned automatically."""
    goal_delay_s: float = Field(0, ge=0)
    """Pause after any tick that started a new goal (rate-limits goals while testing)."""
    tick_delay_s: float = Field(0, ge=0)
    """Pause after every other tick."""
    max_goals_per_run: int = Field(0, ge=0)
    """Goals the loop may start per run before pausing itself; 0 means unlimited. Resuming
    from the portal grants another window of the same size."""
    places_memory: bool = False
    """Remember places across ticks and mark every view the executor reads: destinations already
    within reach (`here`), reached before (`visited`), or failed (`gpt_pet.places`). With S1 on,
    it also skips and ranks its options by those marks."""
    here_m: float = Field(1.5, gt=0)
    """With `places_memory`: an object at most this far away (its `distance_m`) counts as within
    reach, so a drive there would go nowhere, until drives teach a longer reach. The simulator
    measures to an object's centre, where its planner already counts most objects within 2 m as
    reached (sim.ini: 2.0); a depth camera measures to the nearest surface."""
    map_for_goals: bool = False
    """Show the goal setter the robot's top-down map (`get_map`) whenever Gemini decides a goal,
    and, with `places_memory`, a summary of what the pet has done and the places it has seen but
    not visited, any of which a new goal may name as its `target_id`."""
    failed_guard: bool = False
    """With `places_memory`: refuse a drive to a destination a drive already failed to reach from
    about where the robot stands (views mark it `failed_here`), and tell the goal setter which
    places drives failed to reach (those go last in its list)."""
    failure_guidance: bool = False
    """Tell the executor not to retry a failed drive from the same spot (back up or turn first),
    and the goal setter that one failed drive is no reason to abandon a goal."""


class NavSettings(_Strict):
    """How the brain waits out a drive. `set_nav_goal` returns to the model only once the drive
    has ended, so no LLM calls are spent while the robot drives (see `gpt_pet.navwait`)."""

    poll_interval_s: float = Field(0.5, gt=0)
    """How often the brain itself checks get_nav_status during a drive."""
    max_wait_s: float = Field(120, gt=0)
    """Stop waiting after this long; the drive continues and the executor reports it as ongoing."""


class S1Settings(_Strict):
    """System 1: a local decision model (nimble on Ollama, or any endpoint speaking TypeSafe's
    System One API) takes the turns it can decide with confidence; Gemini takes the rest and
    still writes every new goal (see `gpt_pet.s1`)."""

    enabled: bool = False
    base_url: HttpUrl = Field(default=HttpUrl("http://127.0.0.1:11434"))
    """Serves `POST /v1/systemone` (Ollama >= 0.35)."""
    model: str = "nimble"
    min_confidence: float = Field(0.75, ge=0, le=1)
    """Certainty an answer needs before S1 acts on it: the API's confidence for a choice (1 -
    normalized entropy), the margin |p(yes) - p(no)| for a yes/no answer, so at 0.75 "the
    success criteria are met" needs p >= 0.875. Tune it with the portal's stats."""
    timeout_s: float = Field(30, gt=0)
    """Per decision. The first call after the model unloads includes loading it (~9 s here)."""
    scan_rule: bool = False
    """Turn by rule when a view offers nothing new to drive to, and write the report by rule when
    the tick's looks run out (instead of handing the turn to Gemini)."""
    explore_policy: bool = False
    """Carry out Gemini's direction by rule: drive to a goal's `target_id` first, and on an
    explore goal pick unvisited passages first, then the farthest unvisited objects (needs
    `[brain] places_memory`). S1 still never sets a goal."""
    unstuck_rule: bool = False
    """With `[brain] places_memory`: after drives failed twice in a row without the robot moving,
    back up and turn around (by rule) before anything else."""
    explore_ticks: int = Field(0, ge=0)
    """With the places memory: keep an explore goal by rule, without asking anyone, for up to this
    many stretches while unvisited places remain and the robot is not stuck (0: off). Fewer goal
    decisions, so less time waiting for Gemini; `[brain] max_goal_attempts` must leave room."""
    rules_only: bool = False
    """S1 without its decision model: the rules (look first, look again after moving, the scan
    rule, the explore policy) still take their turns, and whatever the model would decide goes to
    Gemini (or the local model in its place). For a machine that cannot hold the decision model
    (nimble needs 9.5 GB) next to a local System 2."""


class PeopleSettings(_Strict):
    """`[people]`: the pet notices people and engages them (`gpt_pet.people`; gptpet-plans: People
    Interaction, step 2). Off by default: then none of it runs and the attention tools stay hidden."""

    enabled: bool = False
    labels: list[str] = Field(default_factory=lambda: ["Person"])
    """Destination labels that are people (the simulator's person; a detector's class on the robot)."""
    watch: bool = True
    """Look for people while the robot drives (a person coming into view stops a drive that is not
    about them) and while it rests between ticks (a person wakes it up). No model calls."""
    watch_every_s: float = Field(1.5, gt=0)
    react_every_s: float = Field(60.0, ge=0)
    """After reacting to someone (stopping a drive, waking up, turning to them, asking the goal
    setter) the pet does not do it again for them for this long, unless its goal is about them."""
    attention_wait_s: float = Field(15.0, gt=0)
    """`set_attention` returns once the person is framed or lost, or after this long."""
    engage: bool = False
    """Interest check (`gpt_pet.social`; gptpet-plans: People Step 3): turning to someone whose
    interest is unknown goes on into watching them for `engage_window_s` and deciding whether they
    want to interact, from their body language (a vision call per look, `cue_model`) and their
    track. Interested: "Human wants to interact with me!" and the goal setter picks an interaction.
    Not interested: the pet leaves them be for `ignore_for_s` (doubling for each disinterest in a
    row, up to `ignore_max_s`)."""
    engage_window_s: float = Field(10.0, gt=0)
    engage_look_every_s: float = Field(2.0, gt=0)
    ignore_for_s: float = Field(120.0, ge=0)
    ignore_max_s: float = Field(900.0, ge=0)
    cue_model: str = ""
    """The vision model reading body language; empty: the executor's model."""
    memory_file: str = ""
    """Keep what the pet learns about people (who to leave be, which cues meant interest) in this
    JSON file across sessions; empty: in memory only. Relative paths are from the config dir."""

    @field_validator("labels", mode="before")
    @classmethod
    def _parse_labels(cls, value: Any) -> Any:
        return _split_csv(value)


class FeatureSettings(_Strict):
    free_camera: bool = False
    """Simulator only: a portal-controlled camera that flies around the room. Needs the
    `sim_camera_*` tools of ai2thor-mcp; the real robot has no such thing."""
    attention: bool = False
    """The robot can keep a person framed on its own (the set_attention tools of ai2thor-mcp and
    of gpt-pet-mcp on the real robot): the portal polls get_attention_status and shows what the
    robot is doing when it turns, tilts or walks up."""


class Settings(_Strict):
    profile: ProfileSettings
    model: ModelSettings = Field(default_factory=ModelSettings)
    mcp: McpSettings
    brain: BrainSettings = Field(default_factory=BrainSettings)
    tool_limits: dict[str, int] = Field(default_factory=dict)
    """Per-tick call cap per tool name (ini section [tool_limits]). Unlisted tools are uncapped."""
    nav: NavSettings = Field(default_factory=NavSettings)
    s1: S1Settings = Field(default_factory=S1Settings)
    people: PeopleSettings = Field(default_factory=PeopleSettings)
    features: FeatureSettings = Field(default_factory=FeatureSettings)

    @field_validator("tool_limits")
    @classmethod
    def _limits_are_positive(cls, value: dict[str, int]) -> dict[str, int]:
        bad = {name: limit for name, limit in value.items() if limit < 1}
        if bad:
            raise ValueError(f"tool_limits must be >= 1: {bad}")
        return value

    @model_validator(mode="after")
    def _drives_can_be_waited_out(self) -> "Settings":
        tools = self.mcp.tool_filter  # empty means every tool the server exposes
        if tools and "set_nav_goal" in tools and "get_nav_status" not in tools:
            raise ValueError("[mcp] tool_filter lists set_nav_goal without get_nav_status, which the brain waits out drives with")
        return self


def default_config_dir() -> Path:
    return Path(str(resources.files("gpt_pet.config")))


def resolve_profile(profile: str | None = None) -> str:
    return profile or os.environ.get(PROFILE_ENV) or DEFAULT_PROFILE


def resolve_config_path(profile: str | None = None, config_dir: Path | None = None) -> Path:
    if config_dir is None:
        env_dir = os.environ.get(CONFIG_DIR_ENV)
        config_dir = Path(env_dir) if env_dir else default_config_dir()
    return Path(config_dir) / f"{resolve_profile(profile)}.ini"


def read_ini(path: Path) -> dict[str, dict[str, str]]:
    """Read an ini file into {section: {key: value}} without interpolation."""
    if not path.is_file():
        raise FileNotFoundError(f"settings file not found: {path}")
    parser = configparser.ConfigParser(interpolation=None, inline_comment_prefixes=(";", "#"))
    parser.optionxform = str  # keep key case: tool names are case-sensitive
    parser.read(path, encoding="utf-8")
    return {section: dict(parser.items(section)) for section in parser.sections()}


def load_settings(profile: str | None = None, config_dir: Path | None = None) -> Settings:
    """Load and validate the ini file for a profile (see module docstring for the lookup order)."""
    name = resolve_profile(profile)
    path = resolve_config_path(name, config_dir)
    if not path.is_file():
        raise FileNotFoundError(f"no settings file for profile {name!r}: {path}")
    return Settings.model_validate(read_ini(path))
