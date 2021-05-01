import copy

from simulator import mxCellDecoder
from simulator.components.Pickable import Pickable

TYPE = 'pickable'


def build_object(cell, world, window_options, draw2entity):
    skeleton = copy.copy(cell)
    (components, style) = mxCellDecoder.parse_object(cell, window_options)
    pick = Pickable(float(cell.attrib['weight']), cell.attrib['name'], skeleton)
    components.append(pick)
    ent = world.create_entity()
    for c in components:
        world.add_component(ent, c)
    return {}, [], {style['name']: ent}
