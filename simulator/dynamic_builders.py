from utils.helpers import list_folder
from typing import List
from pathlib import Path


def export_available_builders(extra_paths: List[Path] = []) -> dict:
    available_builders = {}
    # TODO: Add an include option to add more model folder on simulation.json
    for f in [Path('./simulator/builders')] + extra_paths:
        modules = list_folder(f)
        for file_name, module in modules.items():
            tag = module.__dict__['TYPE']
            available_builders[tag] = module
    return available_builders
