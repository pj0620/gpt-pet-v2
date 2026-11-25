"""What every agent's Gemini calls share: sampling, a thinking level and an image resolution when
they are set, and a request timeout so a hung call fails (and an agent's fallback stands in for it)
instead of stalling the pet. Other providers' adapters read the same settings themselves."""
from __future__ import annotations

from google.genai import types

from gpt_pet.settings import ModelSettings

THINKING_LEVELS = {"minimal": types.ThinkingLevel.MINIMAL, "low": types.ThinkingLevel.LOW, "medium": types.ThinkingLevel.MEDIUM, "high": types.ThinkingLevel.HIGH}
MEDIA_RESOLUTIONS = {
    "low": types.MediaResolution.MEDIA_RESOLUTION_LOW,
    "medium": types.MediaResolution.MEDIA_RESOLUTION_MEDIUM,
    "high": types.MediaResolution.MEDIA_RESOLUTION_HIGH,
}


def content_config(model: ModelSettings, name: str = "") -> types.GenerateContentConfig:
    """The config for the agent whose model is `name`; Gemini models get `[model] reasoning_effort`
    as their thinking level and `image_detail` as their media resolution (which also covers the
    images inside tool results)."""
    timeout = types.HttpOptions(timeout=int(model.request_timeout_s * 1000)) if model.request_timeout_s > 0 else None
    gemini = name.startswith("gemini")
    level = THINKING_LEVELS.get(model.reasoning_effort) if gemini else None
    thinking = types.ThinkingConfig(thinking_level=level) if level is not None else None
    resolution = MEDIA_RESOLUTIONS.get(model.image_detail) if gemini else None
    return types.GenerateContentConfig(temperature=model.temperature, http_options=timeout, thinking_config=thinking, media_resolution=resolution)
