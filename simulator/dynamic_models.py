import importlib
import os


def export_available_models():
    available_models = {}
    # Changes dir to the components directory
    # TODO: Add an include option to add more model folder on simulation.json
    for component in os.listdir('./models'):
        file_name, extension = os.path.splitext(component)
        if not extension == '.py':
            continue
        if file_name.startswith('__') and file_name.endswith('__'):
            continue
        module = importlib.import_module(f'models.{file_name}')
        tag = module.__dict__['MODEL']
        available_models[tag] = module
    return available_models
