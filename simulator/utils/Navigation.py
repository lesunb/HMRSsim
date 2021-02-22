from typehints.component_types import Point
from typing import NamedTuple, List

Node = List[Point]
POI = NamedTuple('POI', [('tag', str), ('point', Point)])


def normalize_point(point: Point, map_component) -> Point:
    """Transforms a point coordinate into its super-point coordinate"""
    point_width = map_component.point_width
    x = (point[0] // point_width) * point_width + (point_width // 2)
    y = (point[1] // point_width) * point_width + (point_width // 2)
    return x, y


def merge_edges(a: List[Point], b: List[Point]):
    return list(set(a + b))
