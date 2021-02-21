import logging
import esper

from components.Map import Map

from components.Position import Position
from typehints.build_types import DependencyNotFound
from xml.etree.ElementTree import Element

TYPE = 'map-path'


def build_object(cell, world: esper.World, window_options, draw2entity):
    logger = logging.getLogger(__name__)
    mxCell = cell[0]
    points = path_from_mxCell(mxCell, draw2entity, world)
    key = cell.attrib.get('key', '')
    if key == '':
        logger.warning(f"Map entry without key. Using default value")
        key = 'Default'
    # Entity 1 is the simulation entity
    if world.has_component(1, Map):
        simulation_map = world.component_for_entity(1, Map)
        if key == 'Default':
            key += str(len(simulation_map))
        simulation_map.paths[key] = points
    else:
        if key == 'Default':
            key += '0'
        simulation_map = Map({key: points})
        world.add_component(1, simulation_map)
    logger.debug(f'Added Path {key} to simulation map - {points}')
    return {}, [], {}


def path_from_mxCell(cell: Element, draw2entity, world: esper.World):
    """Extracts the points of a path from XML."""
    logger = logging.getLogger(__name__)
    points = []
    lastPoint = None
    geometry = cell[0]
    # Check if there's a source object - 1st point
    if 'source' in cell.attrib:
        source_ent = draw2entity.get(cell.attrib['source'], None)
        if source_ent is None:
            raise DependencyNotFound("Path source not found")
        source_pos = world.component_for_entity(source_ent[0], Position)
        points.append(source_pos.center)
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
    # Check if there's a target object - Last point
    if 'target' in cell.attrib:
        target_ent = draw2entity.get(cell.attrib['target'], None)
        if target_ent is None:
            raise DependencyNotFound("Path target not found")
        target_pos = world.component_for_entity(target_ent[0], Position)
        points.append(target_pos.center)
    return points


def parse_mxPoint(el):
    return float(el.attrib.get('x', 0)), float(el.attrib.get('y', 0))
