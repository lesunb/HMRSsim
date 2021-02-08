from components.Collidable import Collidable
from components.Position import Position
from utils.helpers import *

from typehints.component_types import Component, ShapeDefinition
from typing import Tuple, List

MODEL = 'mxgraph.floorplan.wallCorner'


def from_mxCell(el, line_width=10) -> Tuple[List[Component], dict]:
    # Parse style
    style = parse_style(el.attrib['style'])
    if style.get('shape', "") != 'mxgraph.floorplan.wallCorner':
        raise Exception("Cannot create Wall from {}: shape is not mxgraph.floorplan.wallCorner".format(el))
    # Get parent
    parent_element = el.attrib['parent']
    direction = style.get('direction', 'east')
    style['parent'] = parent_element

    # Get geometry
    geometry = el[0]
    x = float(geometry.attrib.get('x', '0'))
    y = float(geometry.attrib.get('y', '0'))
    width = float(geometry.attrib['width'])
    height = float(geometry.attrib['height'])
    # Create drawing
    pos = Position(x=x, y=y, w=width, h=height, movable=False)
    center = (pos.x + pos.w // 2, pos.y + pos.h // 2)

    boxes: List[ShapeDefinition] = [
        (
            (pos.x + line_width // 2, pos.y + pos.h // 2),
            [(pos.x, pos.y), (pos.x + line_width, pos.y), (pos.x + line_width, pos.y + pos.h), (pos.x, pos.y + pos.h)]
        ),
        (
            (pos.x + pos.w // 2, pos.y + line_width // 2),
            [(pos.x, pos.y), (pos.x + pos.w, pos.y), (pos.x + pos.w, pos.y + line_width), (pos.x, pos.y + line_width)]
        )
    ]

    # Get the right corner
    if direction == 'north':
        boxes = list(map(lambda sd: rotate_shape_definition(sd, 90, pos.center), boxes))
    elif direction == 'south':
        # FIXME: This rotation is off by 5 units for some reason.
        boxes = list(map(lambda sd: rotate_shape_definition(sd, -90, pos.center), boxes))
    elif direction == 'west':
        boxes = list(map(lambda sd: rotate_shape_definition(sd, 180, pos.center), boxes))

    # Check for rotation
    if style.get('rotation', '') != '':
        rotate = int(style['rotation'])
        if rotate < 0:
            rotate = 360 + rotate
        boxes = list(map(lambda sd: rotate_shape_definition(sd, rotate, pos.center), boxes))

    return [pos, Collidable(boxes)], style
