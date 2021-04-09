import logging
import esper

from simulator.components.Map import Map
from utils.Navigation import add_nodes_from_points

from simulator.components.Position import Position
from typehints.build_types import DependencyNotFound
from xml.etree.ElementTree import Element

TYPE = 'map-path'


def build_object(cell, world: esper.World, window_options, draw2entity):
    logger = logging.getLogger(__name__)
    mxCell = cell[0]
    points = path_from_mxCell(mxCell, draw2entity, world)
    if len(points) <= 1:
        raise Exception(f'Map path has {len(points)} points. Minimum is 2.')

    # Entity 1 is the simulation entity
    if world.has_component(1, Map):
        simulation_map = world.component_for_entity(1, Map)
    else:
        simulation_map = Map()
        world.add_component(1, simulation_map)
    add_nodes_from_points(simulation_map, points)
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
            raise Exception('Failed to create Path')
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
