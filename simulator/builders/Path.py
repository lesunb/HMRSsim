import logging
import esper

from components.Path import Path
from components.Position import Position
from typehints.build_types import DependencyNotFound
from xml.etree.ElementTree import Element

TYPE = 'path'


def build_object(cell: Element, world: esper.World, window_options, draw2entity: dict):
    logger = logging.getLogger(__name__)
    mxCell = cell[0]

    points = path_from_mxCell(mxCell, draw2entity, world)
    obj = mxCell.attrib.get('source', None)
    (ent, _) = draw2entity.get(obj, (None, None))
    if ent is None:
        raise DependencyNotFound('Path origin not found')
    else:
        logger.debug(f"Adding path to entity {ent}")
        world.add_component(ent, Path(points))
    return {}, [], {}


def path_from_mxCell(cell: Element, draw2entity, world: esper.World):
    """Extracts the points of a path from XML."""
    logger = logging.getLogger(__name__)
    points = []
    lastPoint = None
    geometry = cell[0]
    for el in geometry:
        if el.tag == 'mxPoint':
            if el.attrib['as'] == 'targetPoint':
                lastPoint = parse_mxPoint(el)
            else:
                points.append(parse_mxPoint(el))
        elif el.tag == 'Array' and el.attrib['as'] == 'points':
            for p in el:
                points.append(parse_mxPoint(p))
        else:
            logger.error(f'Path object has unknown element {el} in cell geometry')
            return Exception('Failed to create Path')
    if lastPoint:
        points.append(lastPoint)
    # Check if there's a target object
    if 'target' in cell.attrib:
        target_ent = draw2entity.get(cell.attrib['target'], None)
        if target_ent is None:
            raise DependencyNotFound("Path target not found")
        target_pos = world.component_for_entity(target_ent[0], Position)
        points.append(target_pos.center)
    return points


def parse_mxPoint(el):
    return float(el.attrib['x']), float(el.attrib['y'])
