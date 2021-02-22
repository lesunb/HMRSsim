from dynamic_importer import init_component, ComponentInitError
import logging


def initialize_components(components):
    logger = logging.getLogger(__name__)
    initialized_components = []
    for component_name, args in components.items():
        try:
            initialized_components.append(init_component(component_name, args))
        except ComponentInitError:
            logger.error(f'Failed to create component {component_name} for entity from json.')
    return initialized_components
