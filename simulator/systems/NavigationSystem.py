import math

from queue import Queue

from components.Map import Map
from components.Path import Path
from utils.Navigation import Point, normalize_point, Node, PathNotFound, merge_edges
from typing import List, Dict


def find_route(map_component: Map, source: Point, target: Point) -> Path:
    """Finds a route for the source point to the target point, using map nodes."""
    normalized_target = normalize_point(target, map_component)
    parent = {source: (-1, -1)}
    queue = Queue()
    queue.put(source)
    while not queue.empty():
        # Typical search
        curr = queue.get()
        norm, conn = create_live_node(map_component, curr, target)
        if curr != norm:
            parent[norm] = curr
        if norm == normalized_target:
            parent[target] = normalized_target
            return extract_path(parent, target)
        for c in conn:
            if c not in parent:
                parent[c] = norm
                queue.put(c)
    # At this point no path was found.
    # We raise PathNotFound exception and return the path that leads as close as possible to target
    closest = min(
        map(
            lambda x: (x, distance(x, target)),
            parent.keys()
        ),
        key=lambda x: x[1]
    )
    best_path = extract_path(parent, closest[0]) if distance(closest[0], target) < map_component.wander_max_dist else None
    raise PathNotFound(source, target, best_path)


def create_live_node(map_component: Map, source: Point, target: Point) -> (Point, Node):
    """Gets connections for a node that's potentially not in the graph"""
    normalized_source = normalize_point(source, map_component)
    normalized_target = normalize_point(target, map_component)
    # 1st we check if node is in the map
    if normalized_source in map_component.nodes:
        nodes = map_component.nodes[normalized_source] \
                + ([normalized_target] if distance(normalized_source, normalized_target) < map_component.wander_max_dist else [])
        return normalized_source, nodes
    # If not, then we "create" a new node tat connects to all others
    # Withing a certain range
    node_locations = list(map_component.nodes.keys()) + [normalized_target]
    node_distance_to_source = map(lambda p: distance(normalized_source, p), node_locations)
    nodes_within_acceptable_distance = filter(
        lambda x: x[1] <= map_component.wander_max_dist,
        zip(node_locations, node_distance_to_source)
    )
    return normalized_source, list(map(lambda x: x[0], nodes_within_acceptable_distance))


def extract_path(parent: dict, target: Point) -> Path:
    """Backtracks parent dict to find the path from source to target."""
    reversed_path = [target]
    curr = target
    while curr != (-1, -1):
        reversed_path.append(curr)
        curr = parent[curr]
    return Path(reversed(reversed_path))


def distance(a: Point, b: Point) -> float:
    """Euclidean distance of 2 points"""
    return math.sqrt((a[0] - b[0])**2 + (a[1] - b[1])**2)


def add_nodes_from_points(map_component: Map, points: List[Point]):
    """Updated a map_component from a new list of points (aka Path)"""
    if len(points) < 2:
        return
    points = list(map(lambda p: normalize_point(p, map_component), points))
    # Check if edges are repeated
    if points[-1] == points[-2]:
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
        node_map[k] = merge_edges(v, map_component.nodes.get(k, []))
    map_component.nodes.update(node_map)
