from simulator.utils.helpers import list_folder
from pathlib import Path

def export_available_models():
    root_models = Path(__file__.replace('dynamic_models.py', 'models'))
    available_models = {}

    modules = list_folder(root_models)
    for _, module in modules.items():
        tag = module.__dict__['MODEL']
        available_models[tag] = module
    return available_models
