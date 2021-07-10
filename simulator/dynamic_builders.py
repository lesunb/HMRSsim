from simulator.utils.helpers import list_folder
from typing import List
from pathlib import Path
from simulator import builders

def export_available_builders(extra_paths: List[Path] = []) -> dict:
    available_builders = builders.__dict__
    for f in extra_paths:
        modules = list_folder(f)
        for _, module in modules.items():
            tag = module.__dict__['TYPE']
            available_builders[tag] = module
    return available_builders
