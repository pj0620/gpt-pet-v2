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

from pydantic import BaseModel, ConfigDict, Field, HttpUrl, field_validator

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


class FeatureSettings(_Strict):
    free_camera: bool = False
    """Simulator only: a portal-controlled camera that flies around the room. Needs the
    `sim_camera_*` tools of ai2thor-mcp; the real robot has no such thing."""


class Settings(_Strict):
    profile: ProfileSettings
    model: ModelSettings = Field(default_factory=ModelSettings)
    mcp: McpSettings
    brain: BrainSettings = Field(default_factory=BrainSettings)
    tool_limits: dict[str, int] = Field(default_factory=dict)
    """Per-tick call cap per tool name (ini section [tool_limits]). Unlisted tools are uncapped."""
    features: FeatureSettings = Field(default_factory=FeatureSettings)

    @field_validator("tool_limits")
    @classmethod
    def _limits_are_positive(cls, value: dict[str, int]) -> dict[str, int]:
        bad = {name: limit for name, limit in value.items() if limit < 1}
        if bad:
            raise ValueError(f"tool_limits must be >= 1: {bad}")
        return value


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
