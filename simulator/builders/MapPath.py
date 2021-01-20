from components.Map import Map
from components.Path import Path

TYPE = 'map-path'


def build_object(cell, world, window_options, draw2entity):
    mxCell = cell[0]
    points = Path.from_mxCell(mxCell, window_options[0][1])
    objId = cell.attrib.get('origin', '')
    key = cell.attrib.get('key', '')
    if key == '':
        print(f"Map entry without key. Using default value")
        key = 'Default'
    (ent, _) = draw2entity.get(objId, (None, None))
    if ent is None:
        print(f"Path origin ({objId}) not found. Trying target.")
    else:
        if world.has_component(ent, Map):
            map = world.component_for_entity(ent, Map)
            if key == 'Default':
                key += str(len(map))
            map.paths[key] = points
        else:
            if key == 'Default':
                key += '0'
            newMap = Map({key: points})
            world.add_component(ent, newMap)
    return {}, [], {}