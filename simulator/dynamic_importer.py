"""Extracts components that can be imported dynamically into the simulation.

   EXPORTS:
   available_components: dict -- dict of components that can be dynamically imported.
                                 key is component file name. value is the module.
   init_component -- instantiate a component and return it.

"""
import logging
import typing

from utils.helpers import list_folder

# TODO: Add an include option to add more component folder on simulation.json
available_components = list_folder('./components')


def init_component(component_name: str, args: typing.List[typing.Any]):
    """Instantiates a component and returns it.

       KEYWORD ARGUMENTS:
           component_name -- name of the component to instantiate.
                             Must be one of the available components.
           args -- List of arguments to be passed to __init__ method of the component
    """
    logger = logging.getLogger(__name__)
    logger.info(f'Initing component {component_name} with values {args}')
    if component_name not in available_components:
        raise Exception(f"Component {component_name} is not available")
    module = available_components[component_name]
    return module.__dict__[component_name](*args)
