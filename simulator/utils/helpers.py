import math
import os
import importlib.util
import importlib
from simulator import primitives
from collision import Vector
from simulator.typehints.component_types import ShapeDefinition, Point
from typing import Tuple, Union, List, Dict
from pathlib import Path
import logging

logger = logging.getLogger(__name__)
ShapeType = Union[primitives.Rectangle, primitives.Ellipse]


def parse_style(style):
    s = {}
    items = style.split(';')
    for item in items:
        if item == "":
            continue
        elif '=' not in item:
            s[item] = True
            continue
        [key, value] = item.split('=')
        s[key] = value
    return s


def get_rel_points(center: Point, points: List[Point]) -> List[Vector]:
    return list(map(lambda x: Vector(x[0] - center[0], x[1] - center[1]), points))


def tuple2vector(x: Point) -> Vector:
    return Vector(x[0], x[1])


def rotate_around_point(xy, radians, origin=(0, 0)):
    """Rotate a point around a given point.
    
    I call this the "high performance" version since we're caching some
    values that are needed >1 time. It's less readable than the previous
    function but it's faster.
    From: https://gist.github.com/LyleScott/e36e08bfb23b1f87af68c9051f985302
    """
    x, y = xy
    offset_x, offset_y = origin
    adjusted_x = (x - offset_x)
    adjusted_y = (y - offset_y)
    cos_rad = math.cos(radians)
    sin_rad = math.sin(radians)
    qx = offset_x + cos_rad * adjusted_x + sin_rad * adjusted_y
    qy = offset_y + -sin_rad * adjusted_x + cos_rad * adjusted_y

    return qx, qy


def rotate_shape_definition(definition: ShapeDefinition, angle: float, center: Point) -> ShapeDefinition:
    if angle < 0:
        angle = 360 + angle
    new_center = rotate_around_point(definition[0], math.radians(angle), center)
    new_points = list(map(lambda p: rotate_around_point(p, math.radians(angle), center), definition[1]))
    return new_center, new_points

    
def mirror_shape_definition_horizontally(definition: ShapeDefinition, shape_center: Point) -> ShapeDefinition:
    def_center = definition[0]
    new_def_center = ((shape_center[0] - def_center[0]) + shape_center[0], def_center[1])
    new_points = list(map(lambda p: ((shape_center[0] - p[0]) + shape_center[0], p[1]), definition[1]))
    return new_def_center, new_points


def mirror_shape_definition_vertically(definition: ShapeDefinition, shape_center: Point) -> ShapeDefinition:
    def_center = definition[0]
    new_def_center = (def_center[0], (shape_center[1] - def_center[1]) + shape_center[1])
    new_points = list(map(lambda p: (p[0], (shape_center[1] - p[1]) + shape_center[1]), definition[1]))
    return new_def_center, new_points

def list_folder(path: Path) -> Dict:
    available = {}
    if not path.exists():
        logger.log(5, f'[list_folder] Path {path} not found')
        return {}
    try:
        for component in os.listdir(path):
            file_name, extension = os.path.splitext(component)
            if not extension == '.py':
                continue
            if file_name.startswith('__') and file_name.endswith('__'):
                continue
            try:
                spec = importlib.util.spec_from_file_location(file_name, path / component)
                module = importlib.util.module_from_spec(spec)
                available[file_name] = module
                spec.loader.exec_module(module)
            except AttributeError:
                logger.error(f'Failed to load module {path / file_name}')
    except FileNotFoundError:
        return {}
    return available

def import_components_from_folder(path: Path, prefix: str):
    available = {}
    if not path.exists():
        logger.log(5, f'[list_folder] Path {path} not found')
        return {}
    try:
        for component in os.listdir(path):
            file_name, extension = os.path.splitext(component)
            if not extension == '.py':
                continue
            if file_name.startswith('__') and file_name.endswith('__'):
                continue
            try:
                module = importlib.import_module(prefix + '.' + file_name)
                available[file_name] = module
            except AttributeError:
                logger.error(f'Failed to load module {path / file_name}')
    except FileNotFoundError:
        return {}
    return available