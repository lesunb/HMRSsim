from simulator.utils.helpers import list_folder
from pathlib import Path
from simulator import models

def export_available_models():
    available_models = models.__dict__

    modules = list_folder(Path('./models'))
    for _, module in modules.items():
        tag = module.__dict__['MODEL']
        available_models[tag] = module
    return available_models
