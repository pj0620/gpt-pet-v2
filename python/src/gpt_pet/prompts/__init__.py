"""Prompt files shipped with the package, loaded by name."""
from importlib import resources


def load_prompt(name: str) -> str:
    return resources.files("gpt_pet.prompts").joinpath(f"{name}.md").read_text(encoding="utf-8")
