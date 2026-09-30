"""Offline: the workflow is wired as designed. Building it makes no connection."""
from __future__ import annotations

import importlib

import pytest
from google.adk.tools.mcp_tool.mcp_toolset import McpToolset
from google.adk.workflow import BaseNode, Workflow

from gpt_pet.brain import BRAIN_NAME, MEMORY_NODE_NAME, build_brain
from gpt_pet.goals import GoalDecision
from gpt_pet.settings import PROFILE_ENV, load_settings


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
    brain = build_brain(load_settings("sim"))
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


def test_adk_web_entry_exposes_root_agent(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setenv(PROFILE_ENV, "sim")
    import gpt_pet.agent as agent_module

    module = importlib.reload(agent_module)
    assert isinstance(module.root_agent, BaseNode)
    assert module.root_agent.name == BRAIN_NAME
