"""Offline: the workflow is wired as designed. Building it makes no connection."""
from __future__ import annotations

import importlib

import pytest
from google.adk.tools.mcp_tool.mcp_toolset import McpToolset
from google.adk.workflow import BaseNode, Workflow

from gpt_pet.agents.executor import PLACES_GUIDANCE, TARGET_GUIDANCE
from gpt_pet.agents.fallbacks import executor_on_model_error, goal_setter_on_model_error
from gpt_pet.brain import BRAIN_NAME, MEMORY_NODE_NAME, build_brain
from gpt_pet.goals import GoalDecision, TargetedGoalDecision
from gpt_pet.places import Places
from gpt_pet.settings import PROFILE_ENV, Settings, load_settings


def exploring(settings: Settings, on: bool) -> Settings:
    """The profile with every exploration switch on or off, whatever its defaults, and the
    failure fixes off (tests/test_unstuck.py covers them)."""
    brain = settings.brain.model_copy(update={"places_memory": on, "map_for_goals": on, "failed_guard": False, "failure_guidance": False})
    s1 = settings.s1.model_copy(update={"scan_rule": on, "explore_policy": on, "unstuck_rule": False, "explore_ticks": 0})
    return settings.model_copy(update={"brain": brain, "s1": s1})


def test_brain_is_a_three_node_chain() -> None:
    brain = build_brain(load_settings("sim"))
    workflow = brain.workflow
    assert isinstance(workflow, Workflow)
    assert workflow.name == BRAIN_NAME
    graph = workflow.graph
    assert graph is not None
    names = [node.name for node in graph.nodes if node.name != "__START__"]
    assert names == ["goal_setter", MEMORY_NODE_NAME, "executor"]
    sources = {edge.from_node.name for edge in graph.edges}
    terminal = [name for name in names if name not in sources]
    assert terminal == ["executor"]
    assert all(edge.route is None for edge in graph.edges)


def test_agents_are_configured_as_designed() -> None:
    brain = build_brain(exploring(load_settings("sim"), on=False))
    nodes = {node.name: node for node in brain.workflow.graph.nodes}
    goal_setter = nodes["goal_setter"]
    executor = nodes["executor"]
    assert goal_setter.output_schema is GoalDecision
    assert goal_setter.output_key == "goal_decision"
    assert goal_setter.tools == []
    assert executor.output_key == "last_report"
    assert any(isinstance(tool, McpToolset) for tool in executor.tools)
    assert brain.toolset in executor.tools
    assert executor.after_tool_callback is not None


def test_system_1_sits_in_front_of_both_agents_only_when_enabled() -> None:
    settings = exploring(load_settings("sim"), on=False)
    enabled = build_brain(settings)
    nodes = {node.name: node for node in enabled.workflow.graph.nodes}
    assert enabled.s1 is not None and enabled.stats.s1_model == "nimble"
    assert nodes["goal_setter"].before_model_callback == enabled.s1.goal_setter_turn
    assert nodes["executor"].before_model_callback == enabled.s1.executor_turn

    disabled = build_brain(settings.model_copy(update={"s1": settings.s1.model_copy(update={"enabled": False})}))
    nodes = {node.name: node for node in disabled.workflow.graph.nodes}
    assert disabled.s1 is None and disabled.stats.s1_model is None
    assert nodes["goal_setter"].before_model_callback is None and nodes["executor"].before_model_callback is None


def test_gemini_calls_time_out_and_a_fallback_stands_in_for_a_failed_one() -> None:
    settings = load_settings("sim")
    assert settings.model.request_timeout_s == 20
    nodes = {node.name: node for node in build_brain(settings).workflow.graph.nodes}
    for name in ("goal_setter", "executor"):
        config = nodes[name].generate_content_config
        assert config.http_options is not None and config.http_options.timeout == 20_000  # milliseconds
        assert config.temperature == settings.model.temperature
    assert nodes["goal_setter"].on_model_error_callback is goal_setter_on_model_error
    assert nodes["executor"].on_model_error_callback is executor_on_model_error

    patient = settings.model_copy(update={"model": settings.model.model_copy(update={"request_timeout_s": 0})})
    nodes = {node.name: node for node in build_brain(patient).workflow.graph.nodes}
    assert nodes["goal_setter"].generate_content_config.http_options is None  # 0 waits forever
    assert nodes["executor"].generate_content_config.http_options is None


def test_the_exploration_switches_are_on_in_the_simulator_and_off_on_the_real_robot() -> None:
    sim, real = load_settings("sim"), load_settings("real")
    switches = lambda s: (s.brain.places_memory, s.brain.map_for_goals, s.s1.scan_rule, s.s1.explore_policy)  # noqa: E731
    assert switches(sim) == (True, True, True, True)  # validated by experiments/suites/exploration.json
    assert switches(real) == (False, False, False, False) and real.s1.enabled is False


def test_the_places_memory_and_its_guidance_exist_only_when_enabled() -> None:
    settings = exploring(load_settings("sim"), on=False)
    plain = build_brain(settings)
    assert plain.places is None
    assert PLACES_GUIDANCE not in executor_prompt(plain)

    remembering = build_brain(settings.model_copy(update={"brain": settings.brain.model_copy(update={"places_memory": True})}))
    assert isinstance(remembering.places, Places)
    assert remembering.places.here_m == settings.brain.here_m == 2.0  # the simulator measures to centres
    assert executor_prompt(remembering).endswith(PLACES_GUIDANCE)
    assert remembering.s1 is not None and remembering.s1.marks is True


def test_with_the_map_for_goals_gemini_sees_the_map_and_may_name_a_target() -> None:
    settings = exploring(load_settings("sim"), on=False)
    brain = build_brain(exploring(settings, on=True))
    nodes = {node.name: node for node in brain.workflow.graph.nodes}
    assert brain.map_proxy is not None and brain.map_proxy.url == str(settings.mcp.url)
    goal_setter = nodes["goal_setter"]
    assert goal_setter.output_schema is TargetedGoalDecision
    callbacks = goal_setter.before_model_callback
    assert isinstance(callbacks, list) and callbacks[0] == brain.s1.goal_setter_turn  # S1's gate first: no map when S1 decides
    assert callbacks[1].__name__ == "attach_map"
    assert executor_prompt(brain).endswith(PLACES_GUIDANCE + TARGET_GUIDANCE)

    map_only = build_brain(settings.model_copy(update={"brain": settings.brain.model_copy(update={"map_for_goals": True})}))
    goal_setter = {node.name: node for node in map_only.workflow.graph.nodes}["goal_setter"]
    assert map_only.places is None and goal_setter.output_schema is GoalDecision  # no memory: no targets
    assert map_only.map_proxy is not None


def executor_prompt(brain) -> str:
    executor = {node.name: node for node in brain.workflow.graph.nodes}["executor"]
    return executor.instruction(None)


def test_adk_web_entry_exposes_root_agent(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setenv(PROFILE_ENV, "sim")
    import gpt_pet.agent as agent_module

    module = importlib.reload(agent_module)
    assert isinstance(module.root_agent, BaseNode)
    assert module.root_agent.name == BRAIN_NAME
