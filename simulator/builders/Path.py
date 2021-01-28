from components.Path import Path

TYPE = 'path'


def build_object(cell, world, window_options, draw2entity):
    mxCell = cell[0]
    points = Path.from_mxCell(mxCell, window_options[0][1])
    obj = mxCell.attrib.get('source', None)
    (ent, _) = draw2entity.get(obj, (None, None))
    if ent is None:
        print(f"Path origin ({obj}) not found. Trying target.")
    else:
        print(f"Adding path to entity {ent}")
        world.add_component(ent, Path(points))
    return {}, [], {}
