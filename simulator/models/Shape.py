import primitives as primitives
from components.Collidable import Collidable
from components.Position import Position
from components.Velocity import Velocity
from utils.helpers import parse_style

from typing import Tuple, List
from typehints.component_types import Component

MODEL = 'default'


def from_object(el, line_width=10) -> Tuple[List[Component], dict]:
    options = el.attrib

    components, style = from_mxCell(el[0], line_width)
    if 'collidable' not in options:
        options['collidable'] = True
    if 'movable' not in options:
        options['movable'] = True

    options.update(style)
    pos = components[0]
    if options['type'] == 'robot':
        components.append(Velocity(x=0, y=0))
    if 'collision_tag' in options:
        coll = list(filter(lambda c: isinstance(c, Collidable), components))[0]
        coll.event_tag = options['collision_tag']
    return components, options


def from_mxCell(el, lineWidth=10) -> Tuple[List[Component], dict]:
    # Parse style
    style = parse_style(el.attrib['style'])
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

    rotate = 0
    if style.get('rotation', '') != '':
        rotate = int(style['rotation'])
        if rotate < 0:
            rotate = 360 + rotate
    pos.angle = rotate
    center = (pos.x + pos.w // 2, pos.y + pos.h // 2)

    if 'ellipse' in style:
        draw = primitives.Ellipse(center, width, height, style, rotate)
        col_points = draw._get_points()
    else:
        col_points = pos._get_box()

    return [pos, Collidable([(center, col_points)])], style
