from utils.helpers import list_folder
from pathlib import Path


def export_available_models():
    available_models = {}
    # TODO: Add an include option to add more model folder on simulation.json
    modules = list_folder(Path('./simulator/models'))
    for file_name, module in modules.items():
        tag = module.__dict__['MODEL']
        available_models[tag] = module
    return available_models
