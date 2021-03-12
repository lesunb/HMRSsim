from collision import Poly, Vector
from simulator.components.Collidable import Collidable
from simulator.components.Position import Position
from utils.helpers import parse_style

MODEL = 'mxgraph.floorplan.wall'

def from_mxCell(el, lineWidth=10):
    # Parse style
    style = parse_style(el.attrib['style'])
    if style.get('shape') != 'mxgraph.floorplan.wall':
        raise Exception("Cannot create Wall from {}: shape is not mxgraph.floorplan.wall".format(el))
    # Get parent
    parent_element = el.attrib['parent']
    style['parent'] = parent_element
    # Get geometry
    geometry = el[0]
    x = float(geometry.attrib.get('x', '0'))
    y = float(geometry.attrib.get('y', '0'))
    width = float(geometry.attrib['width'])
    height = float(geometry.attrib['height'])
    # Create drawing
    pos = Position(x=x, y=y, w=width, h=height, movable=False)

    rotate = 0
    if 'rotation' in style:
        rotate = float(style['rotation'])
        rotate = (360 + rotate) % 360
    pos.angle = rotate

    # Create collision box
    col_points = pos._get_box()
    center = (pos.x + pos.w // 2, pos.y + pos.h // 2)
    return [pos, Collidable([(center, col_points)])], style
