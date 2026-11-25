"""Which model an agent runs, by the name in `[model] goal_setter` / `executor`.

- `gemini-...` (no prefix): Gemini, through ADK's own client;
- `openai/<model>`, `openrouter/<vendor>/<model>`: Chat Completions (`gpt_pet.agents.openai_chat`);
- `anthropic/<model>`: Claude (`gpt_pet.agents.anthropic`);
- `ollama/<tag>`: a local model (`gpt_pet.agents.ollama`).
"""
from __future__ import annotations

import os

from google.adk.models.base_llm import BaseLlm

from gpt_pet.agents.anthropic import KEY_ENV as ANTHROPIC_KEY_ENV
from gpt_pet.agents.anthropic import AnthropicLlm, is_anthropic
from gpt_pet.agents.ollama import OllamaLlm, is_ollama
from gpt_pet.agents.openai_chat import PROVIDERS, OpenAIChatLlm, is_openai_chat
from gpt_pet.settings import ModelSettings

GEMINI_KEY_ENVS = ("GOOGLE_API_KEY", "GEMINI_API_KEY")


def agent_model(name: str, model: ModelSettings) -> str | BaseLlm:
    """What an agent runs: a Gemini model id as it is, or the adapter for another provider's model."""
    if is_ollama(name):
        return OllamaLlm.from_settings(name, model)
    if is_openai_chat(name):
        return OpenAIChatLlm.from_settings(name, model)
    if is_anthropic(name):
        return AnthropicLlm.from_settings(name, model)
    return name


def is_gemini(name: str) -> bool:
    return not (is_ollama(name) or is_openai_chat(name) or is_anthropic(name))


def key_env(name: str) -> str | None:
    """The environment variable holding the API key `name`'s provider needs (None: no key, or
    Gemini's, which ADK finds under either of its names)."""
    if is_openai_chat(name):
        return next(key for prefix, (_, key) in PROVIDERS.items() if name.startswith(prefix))
    if is_anthropic(name):
        return ANTHROPIC_KEY_ENV
    return None


def missing_keys(model: ModelSettings) -> list[str]:
    """The API key variables the goal setter's and executor's models need but are not set."""
    needed: list[str] = []
    for name in (model.goal_setter, model.executor):
        if is_gemini(name) and not any(os.environ.get(env) for env in GEMINI_KEY_ENVS):
            needed.append(GEMINI_KEY_ENVS[0])
        elif (env := key_env(name)) and not os.environ.get(env):
            needed.append(env)
    return list(dict.fromkeys(needed))
