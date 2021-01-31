from utils.helpers import list_folder


def export_available_builders():
    available_builders = {}
    # TODO: Add an include option to add more model folder on simulation.json
    modules = list_folder('./builders')
    for file_name, module in modules.items():
        tag = module.__dict__['TYPE']
        available_builders[tag] = module
    return available_builders
