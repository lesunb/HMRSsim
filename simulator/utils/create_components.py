import logging
import pathlib
from simulator.dynamic_importer import init_component, ComponentInitError, expand_available_components


def initialize_components(components):
    logger = logging.getLogger(__name__)
    initialized_components = []
    for component_name, args in components.items():
        try:
            initialized_components.append(init_component(component_name, args))
        except ComponentInitError:
            logger.error(f'Failed to create component {component_name} for entity from json.')
    return initialized_components


def import_external_component(context: str) -> None:
    path = pathlib.Path(context) / 'components'
    expand_available_components([path])
