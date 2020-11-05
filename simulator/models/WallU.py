import pyglet
import primitives as primitives

from collision import Concave_Poly
from components.Collidable import Collidable
from components.Position import Position
from utils.helpers import *


def from_mxCell(el, batch, windowSize, lineWidth=10):
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
    (x, y) = translate_coordinates((x, y), windowSize, height)
    pos = Position(x=x, y=y, w=width, h=height, movable=False)
    center = (pos.x + pos.w // 2, pos.y + pos.h // 2)
    points = [
        (pos.x, pos.y),
        (pos.x, pos.y + pos.h),
        (pos.x - lineWidth // 2, pos.y + pos.h),
        (pos.x + pos.w + lineWidth // 2, pos.y + pos.h),
        (pos.x + pos.w, pos.y + pos.h),
        (pos.x + pos.w, pos.y)
    ]

    # Collision box
    col_points = [
        (pos.x - lineWidth // 2, pos.y),
        (pos.x - lineWidth // 2, pos.y + pos.h + lineWidth // 2),
        (pos.x + pos.w, pos.y + pos.h + lineWidth // 2),
        (pos.x + pos.w + lineWidth // 2, pos.y),
        (pos.x + pos.w - lineWidth // 2, pos.y),
        (pos.x + pos.w - lineWidth // 2, pos.y + pos.h - lineWidth // 2),
        (pos.x + lineWidth // 2, pos.y + pos.h - lineWidth // 2),
        (pos.x + lineWidth // 2, pos.y)
    ]

    if style.get('rotation', '') != '':
        rotate = int(style['rotation'])
        if rotate < 0:
            rotate = 360 + rotate
        points = map(lambda x: rotate_around_point(x, math.radians(rotate), center), points)
        col_points = map(lambda x: rotate_around_point(x, math.radians(rotate), center), col_points)

    drawing = primitives.Line(list(points), style)
    drawing.add_to_batch(batch)

    label = el.attrib.get('value', '')
    if label:
        label = pyglet.text.HTMLLabel(label,
                                      batch=batch,
                                      x=center[0], y=center[1],
                                      anchor_x='center', anchor_y='center')

    col_points = map(lambda x: Vector(x[0] - center[0], x[1] - center[1]), col_points)
    box = Concave_Poly(Vector(center[0], center[1]), list(col_points))

    return ([pos, Collidable(shape=box)], style)
