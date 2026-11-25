"""Live: other providers' small cloud models in Gemini's place (`[model]` names with an
`openai/`, `anthropic/` or `openrouter/` prefix, and Gemini's own small models), against the real
APIs and ai2thor-mcp. Asserts structure, never wording or quality (experiments/cloud_models
measures that). Each provider needs its key in python/src/gpt_pet/.env; OpenRouter's is optional.
Override a provider's model with `GPTPET_<PROVIDER>_TEST_MODEL`."""
from __future__ import annotations

import logging
import os

import pytest
from google.adk.models.google_llm import Gemini
from google.adk.models.llm_request import LlmRequest
from google.genai import types

from gpt_pet.agents.content import content_config
from gpt_pet.agents.goal_setter import MAP_CAPTION, render_goal_setter_prompt
from gpt_pet.agents.models import agent_model
from gpt_pet.brain import build_brain
from gpt_pet.experiment import cost_row, load_prices
from gpt_pet.goals import TargetedGoalDecision
from gpt_pet.mcp_proxy import McpProxy
from gpt_pet.runtime import PetRuntime, is_s1_event
from gpt_pet.settings import Settings

pytestmark = [pytest.mark.live, pytest.mark.timeout(900)]

log = logging.getLogger("gpt_pet.tests")

PROVIDERS = {
    # provider: (key variable, default model, required)
    "gemini": ("GOOGLE_API_KEY", "gemini-3.5-flash-lite", True),
    "openai": ("OPENAI_API_KEY", "openai/gpt-6-luna", True),
    "anthropic": ("ANTHROPIC_API_KEY", "anthropic/claude-haiku-4-5", True),
    "openrouter": ("OPENROUTER_API_KEY", "openrouter/mistralai/mistral-small-3.2-24b-instruct", False),
}


@pytest.fixture(params=sorted(PROVIDERS))
def cloud_model(request: pytest.FixtureRequest) -> str:
    key, default, required = PROVIDERS[request.param]
    if not os.environ.get(key):
        message = f"{key} is not set (python/src/gpt_pet/.env); this test needs the real {request.param} API"
        pytest.fail(message) if required else pytest.skip(message)
    return os.environ.get(f"GPTPET_{request.param.upper()}_TEST_MODEL", default)


def running_on(settings: Settings, name: str) -> Settings:
    """`name` as goal setter and executor with the map and places memory; S1 off, so the model
    takes every turn; OpenAI models without reasoning, the only effort Chat Completions allows
    with tools on gpt-5.4 and later (as the experiment's bots run them)."""
    model = settings.model.model_copy(update={"goal_setter": name, "executor": name, "reasoning_effort": "none" if name.startswith("openai/") else ""})
    brain = settings.brain.model_copy(update={"places_memory": True, "map_for_goals": True, "goal_delay_s": 0, "tick_delay_s": 0})
    s1 = settings.s1.model_copy(update={"enabled": False, "scan_rule": False, "explore_policy": False})
    return settings.model_copy(update={"model": model, "brain": brain, "s1": s1})


async def test_a_cloud_model_decides_a_goal_from_the_real_map(mcp_server: str, cloud_model: str, sim_settings: Settings) -> None:
    proxy = McpProxy(mcp_server)
    try:
        await proxy.call("get_current_view")  # the map grows from what the camera saw
        found = await proxy.call("get_map")
    finally:
        await proxy.close()
    assert found.image, found.text[:200]
    settings = running_on(sim_settings, cloud_model)
    model = agent_model(cloud_model, settings.model)
    model = Gemini(model=model) if isinstance(model, str) else model
    config = content_config(settings.model, cloud_model)
    config.system_instruction = render_goal_setter_prompt({}, sim_settings)
    config.response_schema = TargetedGoalDecision
    config.response_mime_type = "application/json"
    image = types.Part.from_bytes(data=found.image, mime_type=found.mime_type or "image/png")
    request = LlmRequest(model=model.model, contents=[types.Content(role="user", parts=[types.Part(text=MAP_CAPTION), image])], config=config)
    responses = [r async for r in model.generate_content_async(request)]
    text = "".join(part.text or "" for part in responses[-1].content.parts if not part.thought)
    decision = TargetedGoalDecision.model_validate_json(text)
    usage = responses[-1].usage_metadata
    log.info("%s decided %s (%s input tokens)", cloud_model, decision.model_dump(exclude_none=True), usage.prompt_token_count)
    assert usage.prompt_token_count > 200, "the map image was read"


async def test_a_cloud_model_runs_a_tick_in_place_of_gemini(mcp_server: str, cloud_model: str, sim_settings: Settings) -> None:
    brain = build_brain(running_on(sim_settings, cloud_model))
    async with PetRuntime(brain, user_id="live-cloud") as pet:
        result = await pet.run_tick()
        snapshot = pet.stats.snapshot()
    log.info("tick 1 on %s: %s goal=%s", cloud_model, result.summary(), (result.goal or {}).get("goal"))
    assert not result.failed, result.error
    calls = [fc.name for e in result.events if e.author == "executor" and not is_s1_event(e) for fc in e.get_function_calls()]
    assert calls, "the model called the robot's tools itself"
    assert (result.state.get("current_goal") or {}).get("goal"), "the model set a goal"
    [(name, counts)] = snapshot["llm"]["by_model"].items()
    assert cloud_model.endswith(name) and counts["calls"] == snapshot["llm"]["calls"] >= 2 and counts["input"] > 0
    cost = cost_row(snapshot, snapshot["uptime_s"], load_prices())
    log.info("%s: %s, tokens %s, cost %s", cloud_model, calls, counts, cost)
    assert cost["cost_usd"] != "" and not cost["cost_unpriced"], f"{name} needs a price in experiments/pricing.json"


async def test_a_cloud_model_takes_over_ticks_system_1_began(
    mcp_server: str, cloud_model: str, nimble: str, sim_settings: Settings
) -> None:
    """The production brain (System 1 on, every exploration switch): System 1 takes the first turns
    of a tick and hands the rest to the cloud model, whose request then holds System 1's calls
    with the ids ADK stripped. Every tick must run through."""
    settings = running_on(sim_settings, cloud_model)
    s1 = settings.s1.model_copy(update={"enabled": True, "scan_rule": True, "explore_policy": True})
    brain = build_brain(settings.model_copy(update={"s1": s1}))
    async with PetRuntime(brain, user_id="live-cloud-s1") as pet:
        results = [await pet.run_tick() for _ in range(3)]
    handed_over = 0
    for result in results:
        log.info("tick %d on %s with S1: %s", result.number, cloud_model, result.summary())
        assert not result.failed, result.error
        executor = [e for e in result.events if e.author == "executor" and e.content and e.content.role == "model"]
        s1_first = next((i for i, e in enumerate(executor) if is_s1_event(e)), None)
        if s1_first is not None and any(not is_s1_event(e) for e in executor[s1_first + 1 :]):
            handed_over += 1
    log.info("%s took over %d of 3 ticks System 1 began", cloud_model, handed_over)
