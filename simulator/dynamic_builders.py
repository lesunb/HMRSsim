from simulator.utils.helpers import list_folder
from typing import List
from pathlib import Path

def export_available_builders(extra_paths: List[Path] = []) -> dict:
    root_builders = __file__.replace('dynamic_builders.py', 'builders')
    extra_paths.append(root_builders)
    available_builders = {}
    for f in extra_paths:
        modules = list_folder(Path(f))
        for _, module in modules.items():
            tag = module.__dict__['TYPE']
            available_builders[tag] = module
    return available_builders
