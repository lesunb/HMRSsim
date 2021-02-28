from queue import Queue

from components.Map import Map
from components.Path import Path
from utils.Navigation import Point, normalize_point, Node, PathNotFound, merge_edges, distance
from typing import List, Dict

import logging


def find_route(map_component: Map, source: Point, target: Point) -> Path:
    """Finds a route for the source point to the target point, using map nodes."""
    logger = logging.getLogger(__name__)
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
            lambda x: (x, distance(x, normalized_target)),
            parent.keys()
        ),
        key=lambda x: x[1]
    )
    logger.debug(f'Path not found. Best path ends in {closest[0]}')
    best_path = extract_path(parent, closest[0])
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
    zipped = zip(node_locations, node_distance_to_source)
    nodes_within_acceptable_distance = filter(
        lambda x: x[1] <= map_component.wander_max_dist,
        zipped
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

