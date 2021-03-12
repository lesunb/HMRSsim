from typing import List, Tuple
from typehints.component_types import Component, ShapeDefinition
from xml.etree.ElementTree import Element
from simulator.components.Collidable import Collidable
from simulator.components.Position import Position
from utils.helpers import *

MODEL = 'mxgraph.floorplan.room'


def from_mxCell(el: Element, line_width=10) -> Tuple[List[Component], dict]:
    # Parse style
    style = parse_style(el.attrib['style'])
    if style.get('shape', "") != 'mxgraph.floorplan.room':
        raise Exception("Cannot create Wall from {}: shape is not mxgraph.floorplan.room".format(el))

    # Get geometry
    geometry = el[0]
    x = float(geometry.attrib.get('x', '0'))
    y = float(geometry.attrib.get('y', '0'))
    width = float(geometry.attrib['width'])
    height = float(geometry.attrib['height'])
    # Create drawing
    pos = Position(x=x, y=y, w=width, h=height, movable=False)
    center = (pos.x + pos.w // 2, pos.y + pos.h // 2)
    points = [
        (pos.x, pos.y),
        (pos.x, pos.y + pos.h),
        (pos.x - line_width // 2, pos.y + pos.h),
        (pos.x + pos.w + line_width // 2, pos.y + pos.h),
        (pos.x + pos.w, pos.y + pos.h),
        (pos.x + pos.w, pos.y),
        (pos.x + pos.w + line_width // 2, pos.y),
        (pos.x - line_width // 2, pos.y)
    ]
    # Collision points
    # TODO: Verify corners
    boxes: List[ShapeDefinition] = [
        (
            (pos.x + line_width // 2, pos.y + pos.h // 2),
            [(pos.x, pos.y), (pos.x, pos.y + pos.h), (pos.x + line_width, pos.y + pos.h), (pos.x + line_width, pos.y)]
        ),
        (
            (pos.x + pos.w // 2, pos.y + pos.h + line_width // 2),
            [(pos.x, pos.y + pos.h), (pos.x, pos.y + pos.h + line_width), (pos.x + pos.w, pos.y + pos.h + line_width),
             (pos.x + pos.w, pos.y + pos.h)]
        ),
        (
            (pos.x + pos.w + line_width // 2, pos.y + pos.h // 2),
            [(pos.x + pos.w, pos.y + pos.h), (pos.x + pos.w + line_width, pos.y + pos.h),
             (pos.x + pos.w + line_width, pos.y), (pos.x + pos.w, pos.y)]
        ),
        (
            (pos.x + pos.w // 2, pos.y + line_width // 2),
            [(pos.x, pos.y), (pos.x, pos.y + line_width), (pos.x + pos.w, pos.y + line_width), (pos.x + pos.w, pos.y)]
        )
    ]

    if style.get('rotation', '') != '':
        rotate = int(style['rotation'])
        if rotate < 0:
            rotate = 360 - rotate
        boxes = list(map(lambda sd: rotate_shape_definition(sd, rotate, center), boxes))

    return [pos, Collidable(boxes)], style
