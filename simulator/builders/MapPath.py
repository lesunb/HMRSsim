import logging
import esper

from components.Map import Map
from utils.Navigation import normalize_point, merge_edges

from components.Position import Position
from typehints.build_types import DependencyNotFound
from xml.etree.ElementTree import Element

from typehints.component_types import Point
from typing import List, Dict

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


def add_nodes_from_points(map_component: Map, points: List[Point]):
    points = list(map(lambda p: normalize_point(p, map_component), points))
    # Treat the edges
    node_map: Dict[Point, List[Point]] = {points[0]: [points[1]], points[-1]: [points[-2]]}
    # Other points
    for idx in range(1, len(points) - 1):
        node_map[points[idx]] = [points[idx-1], points[idx+1]]
    for k, v in node_map.items():
        node_map[k] = merge_edges(v, map_component.nodes.get(k, []))
    map_component.nodes.update(node_map)


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
