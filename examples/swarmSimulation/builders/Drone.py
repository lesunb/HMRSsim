import json

from simulator import dynamic_importer
from simulator import mxCellDecoder

from components.Hover import Hover
from simulator.components.Velocity import Velocity
from simulator.components.ProximitySensor import ProximitySensor

TYPE = 'drone'

COMPONENT_DEPENDENCIES = []
SYSTEM_DEPENDENCIES = []


def build_object(cell, world, window_options, draw2entity):
    (components, style) = mxCellDecoder.parse_object(cell, window_options)
    ent = world.create_entity()
    # Custom components
    for key, val in cell.attrib.items():
        if key.startswith('component_'):
            component_name = key[10:]  # removes "component_" from the name
            init_values = json.loads(val)
            component = dynamic_importer.init_component(component_name, init_values)
            components.append(component)
    hover = Hover()
    vel = Velocity()
    components += [hover, vel]
    for c in components:
        world.add_component(ent, c)
    return {style['id']: [ent, style]}, [(ent, style['id'])], {}
