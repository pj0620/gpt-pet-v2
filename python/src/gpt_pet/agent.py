"""Entry point for `adk web src`.

The ADK dev UI imports this module and serves `root_agent`. One user message runs one tick of
the pet's life: goal_setter -> update_goal_memory -> executor. The profile comes from the
`GPTPET_PROFILE` environment variable (set in `src/gpt_pet/.env`), defaulting to `sim`.
"""
from gpt_pet.brain import build_brain
from gpt_pet.settings import load_settings

root_agent = build_brain(load_settings()).workflow
