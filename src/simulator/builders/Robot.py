import json
from esper import World

from simulator import dynamic_importer
from simulator import mxCellDecoder

TYPE = 'robot'

COMPONENT_DEPENDENCIES = []
SYSTEM_DEPENDENCIES = []


def build_object(cell, world: World, window_options, draw2entity):
    (components, style) = mxCellDecoder.parse_object(cell, window_options, shape="robot")
    ent = world.create_entity()
    # Custom components
    for key, val in cell.attrib.items():
        if key.startswith('component_'):
            component_name = key[10:]  # removes "component_" from the name
            init_values = json.loads(val)
            component = dynamic_importer.init_component(component_name, init_values)
            components.append(component)
    for c in components:
        world.add_component(ent, c)
    return {style['id']: [ent, style]}, [(ent, style['id'])], {}
