from simulator.components.Collidable import Collidable
from simulator.components.Position import Position
from simulator.utils.helpers import rotate_shape_definition, parse_style

from typing import List, Tuple
from simulator.typehints.component_types import ShapeDefinition, Component


MODEL = 'mxgraph.floorplan.wallU'


def from_mxCell(el, line_width=10) -> Tuple[List[Component], dict]:
    # Parse style
    style = parse_style(el.attrib['style'])
    if style.get('shape', "") != 'mxgraph.floorplan.wallU':
        raise Exception("Cannot create Wall from {}: shape is not mxgraph.floorplan.wallU".format(el))
    # Get parent
    style['parent'] = el.attrib['parent']

    # Get geometry
    geometry = el[0]
    x = float(geometry.attrib.get('x', '0'))
    y = float(geometry.attrib.get('y', '0'))
    width = float(geometry.attrib['width'])
    height = float(geometry.attrib['height'])
    # Create drawing
    pos = Position(x=x, y=y, w=width, h=height, movable=False)
    # Collision box
    boxes: List[ShapeDefinition] = [
        (
            (pos.x + line_width // 2, pos.y + pos.h // 2),
            [(pos.x, pos.y), (pos.x + line_width, pos.y), (pos.x + line_width, pos.y + pos.h), (pos.x, pos.y + pos.h)]
        ),
        (
            (pos.x + pos.w // 2, pos.y + line_width // 2),
            [(pos.x, pos.y), (pos.x + pos.w, pos.y), (pos.x + pos.w, pos.y + line_width), (pos.x, pos.y + line_width)]
        ),
        (
            (pos.x + pos.w - (line_width // 2), pos.y + pos.h // 2),
            [(pos.x + pos.w - line_width, pos.y), (pos.x + pos.w, pos.y), (pos.x + pos.w, pos.y + pos.h), (pos.x + pos.w - line_width, pos.y + pos.h)]
        )
    ]

    if style.get('rotation', '') != '':
        rotate = int(style['rotation'])
        rotate = (rotate + 360) % 360
        boxes = list(map(lambda sd: rotate_shape_definition(sd, -rotate, pos.center), boxes))

    return [pos, Collidable(boxes)], style
