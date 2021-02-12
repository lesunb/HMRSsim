from utils.helpers import list_folder
from utils.config import working_directory

def export_available_builders():
    available_builders = {}
    # TODO: Add an include option to add more model folder on simulation.json
    modules = list_folder(f'./{working_directory}/builders')
    for file_name, module in modules.items():
        tag = module.__dict__['TYPE']
        available_builders[tag] = module
    return available_builders
