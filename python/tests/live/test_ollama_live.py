"""Live: a local vision model served by Ollama in Gemini's place (`ollama/<tag>` in `[model]`),
against the real Ollama and ai2thor-mcp. Asserts structure, never wording or quality (that is
what experiments/local_vlm measures). The model is `GPTPET_LOCAL_MODEL` (default
`qwen3-vl:2b-instruct`, small and quick); it must have Ollama's vision and tools capabilities."""
from __future__ import annotations

import logging
import os

import pytest
from google.adk.models.llm_request import LlmRequest
from google.genai import types

from gpt_pet.agents.goal_setter import MAP_CAPTION, render_goal_setter_prompt
from gpt_pet.agents.ollama import OLLAMA_PREFIX, OllamaLlm
from gpt_pet.brain import build_brain
from gpt_pet.goals import TargetedGoalDecision
from gpt_pet.mcp_proxy import McpProxy
from gpt_pet.runtime import PetRuntime, event_payloads, is_s1_event
from gpt_pet.settings import Settings

pytestmark = [pytest.mark.live, pytest.mark.timeout(900)]

LOCAL_MODEL_ENV = "GPTPET_LOCAL_MODEL"
DEFAULT_LOCAL_MODEL = "qwen3-vl:2b-instruct"

log = logging.getLogger("gpt_pet.tests")


@pytest.fixture(scope="session")
def local_vlm(sim_settings: Settings) -> str:
    """The Ollama tag of a pulled vision model that can call tools."""
    tag = os.environ.get(LOCAL_MODEL_ENV, DEFAULT_LOCAL_MODEL)
    model = OllamaLlm(model=tag, base_url=str(sim_settings.model.ollama_url).rstrip("/"))
    import asyncio

    try:
        capabilities = asyncio.run(model.model_capabilities())
    except Exception as exc:  # noqa: BLE001 - any failure means the model is not there to test
        pytest.fail(f"Ollama cannot describe {tag!r} ({exc}); run `ollama pull {tag}` (or set {LOCAL_MODEL_ENV})")
    missing = {"vision", "tools"} - capabilities
    if missing:
        pytest.fail(f"{tag} lacks Ollama's {sorted(missing)} capabilities: {sorted(capabilities)}")
    return tag


def running_locally(settings: Settings, tag: str) -> Settings:
    """The local model as goal setter and executor, with the map and places memory; S1 off, so the
    local model takes every turn."""
    name = OLLAMA_PREFIX + tag
    model = settings.model.model_copy(update={"goal_setter": name, "executor": name, "request_timeout_s": 120})
    brain = settings.brain.model_copy(update={"places_memory": True, "map_for_goals": True, "goal_delay_s": 0, "tick_delay_s": 0})
    s1 = settings.s1.model_copy(update={"enabled": False, "scan_rule": False, "explore_policy": False})
    return settings.model_copy(update={"model": model, "brain": brain, "s1": s1})


async def test_a_local_model_decides_a_goal_from_the_real_map(mcp_server: str, local_vlm: str, sim_settings: Settings) -> None:
    proxy = McpProxy(mcp_server)
    try:
        await proxy.call("get_current_view")  # the map grows from what the camera saw
        found = await proxy.call("get_map")
    finally:
        await proxy.close()
    assert found.image, found.text[:200]
    model = OllamaLlm.from_settings(OLLAMA_PREFIX + local_vlm, running_locally(sim_settings, local_vlm).model)
    request = LlmRequest(
        contents=[types.Content(role="user", parts=[types.Part(text=MAP_CAPTION), types.Part.from_bytes(data=found.image, mime_type=found.mime_type or "image/png")])],
        config=types.GenerateContentConfig(
            system_instruction=render_goal_setter_prompt({}, sim_settings), temperature=0.2, response_schema=TargetedGoalDecision
        ),
    )
    await model.warm_up()
    [response] = [r async for r in model.generate_content_async(request)]
    text = "".join(part.text or "" for part in response.content.parts if not part.thought)
    decision = TargetedGoalDecision.model_validate_json(text)  # Ollama enforced the schema
    timed = response.custom_metadata["ollama"]
    log.info("%s decided %s in %.1f s (%s)", local_vlm, decision.model_dump(exclude_none=True), timed["total_s"], timed)
    assert timed["prompt_tokens"] > 300, "the map image was read"  # an image costs hundreds of tokens
    assert response.usage_metadata.total_token_count == timed["prompt_tokens"] + timed["output_tokens"]


async def test_a_local_model_runs_ticks_in_place_of_gemini(mcp_server: str, local_vlm: str, sim_settings: Settings) -> None:
    brain = build_brain(running_locally(sim_settings, local_vlm))
    payloads: list[dict] = []
    async with PetRuntime(brain, user_id="live-ollama", on_event=lambda event, tick: payloads.extend(event_payloads(event, tick))) as pet:
        results = [await pet.run_tick(), await pet.run_tick()]
        snapshot = pet.stats.snapshot()
    for result in results:
        log.info("tick %d: %s goal=%s", result.number, result.summary(), (result.goal or {}).get("goal"))
        assert not result.failed, result.error
    executor_calls = [
        fc.name for result in results for event in result.events if event.author == "executor" and not is_s1_event(event)
        for fc in event.get_function_calls()
    ]
    assert executor_calls, "the local model called the robot's tools itself"
    assert (results[0].state.get("current_goal") or {}).get("goal"), "the local model set a goal"
    local_times = snapshot["llm"]["local"]
    log.info("local model: %s; tools called: %s", local_times, executor_calls)
    assert local_times["calls"] == snapshot["llm"]["calls"] >= 2 and local_times["output_tok_s"]
    assert snapshot["llm"]["maps_shown"] >= 1, "the goal setter saw the map"
    assert any("ollama" in p for p in payloads if p.get("author") == "executor"), "the decision log carries the local timings"


async def test_system_1_rules_carry_a_local_model_without_their_decision_model(
    mcp_server: str, local_vlm: str, sim_settings: Settings
) -> None:
    """`[s1] rules_only`: the rules take their turns, nimble is never asked, and what it would have
    judged goes to the local model (the set-up for a GPU too small for both models)."""
    settings = running_locally(sim_settings, local_vlm)
    s1 = settings.s1.model_copy(update={"enabled": True, "rules_only": True, "scan_rule": True, "explore_policy": True})
    brain = build_brain(settings.model_copy(update={"s1": s1}))
    async with PetRuntime(brain, user_id="live-ollama-rules") as pet:
        results = [await pet.run_tick(), await pet.run_tick()]
        snapshot = pet.stats.snapshot()
    for result in results:
        log.info("tick %d: %s goal=%s", result.number, result.summary(), (result.goal or {}).get("goal"))
        assert not result.failed, result.error
    stats = snapshot["s1"]
    log.info("S1: %s", stats)
    assert stats["model"] == "rules only" and stats["decider"]["calls"] == 0
    assert stats["by"].get("rules", 0) > 0 and not stats["by"].get("nimble")
    assert any(reason.endswith("rules only") for reason in stats["escalations"]), stats["escalations"]
    assert snapshot["llm"]["local"]["calls"] == snapshot["llm"]["calls"] >= 2
