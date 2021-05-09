from simulator.typehints.component_types import Point
from typing import NamedTuple, List, Dict
import math

Node = List[Point]
POI = NamedTuple('POI', [('tag', str), ('point', Point)])


class PathNotFound(Exception):
    """Exception when the path finding algorithm can't find a path from source to destination."""
    def __init__(self, source: Point, target: Point, partial_path=None):
        self.partial_path = partial_path
        if partial_path is None:
            self.message = f'No path was found from {source} to {target}'
        else:
            self.message = f'Found only partial path from {source} to {target}'


def normalize_point(point: Point, map_component) -> Point:
    """Transforms a point coordinate into its super-point coordinate."""
    point_width = map_component.point_width
    x = (point[0] // point_width) * point_width + (point_width // 2)
    y = (point[1] // point_width) * point_width + (point_width // 2)
    return x, y


def merge_edges(a: List[Point], b: List[Point]):
    return list(set(a + b))


def distance(a: Point, b: Point) -> float:
    """Euclidean distance of 2 points."""
    return math.sqrt((a[0] - b[0])**2 + (a[1] - b[1])**2)


def add_nodes_from_points(map_component, points: List[Point]):
    """Updated a map_component from a new list of points (aka Path)."""
    if len(points) < 2:
        return
    points = list(map(lambda p: normalize_point(p, map_component), points))
    # Check if edges are repeated
    while len(points) >= 2 and points[-1] == points[-2]:
        points.pop()
    if len(points) < 2:
        return
    if points[0] == points[1]:
        points = points[1:]
    if len(points) < 2:
        return
    # Treat the edges
    node_map: Dict[Point, List[Point]] = {points[0]: [points[1]], points[-1]: [points[-2]]}
    # Other points
    for idx in range(1, len(points) - 1):
        node_map[points[idx]] = [points[idx-1], points[idx+1]]
    for k, v in node_map.items():
        # Try to connect this (potentially) new node with other nodes already in the map
        other_nodes = map_component.nodes.keys()
        node_distance_to_source = map(lambda p: distance(k, p), other_nodes)
        nodes_within_acceptable_distance = filter(
            lambda x: x[1] <= map_component.wander_max_dist and x[0] != k,
            zip(other_nodes, node_distance_to_source)
        )
        close_nodes = list(map(lambda x: x[0], nodes_within_acceptable_distance))
        v += close_nodes
        for n in close_nodes:
            map_component.nodes[n] = merge_edges(map_component.nodes[n], [k])
        node_map[k] = merge_edges(list(set(v)), map_component.nodes.get(k, []))
    map_component.nodes.update(node_map)
