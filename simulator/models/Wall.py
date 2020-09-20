import primitives as primitives
import pyglet
from collision import Poly, Vector
from components.Collidable import Collidable
from components.Position import Position
from components.Renderable import Renderable
from utils.helpers import parse_style, translate_coordinates


def from_mxCell(el, batch, windowSize, lineWidth=10):
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
  (x, y) = translate_coordinates((x, y), windowSize, height)
  pos = Position(x=x, y=y, w=width, h=height, movable=False)
  
  rotate = 0
  if 'rotation' in style:
    rotate = int(style['rotation'])
    if rotate < 0:
      rotate = 360 + rotate
  pos.angle = rotate

  label = el.attrib.get('value', '')
  if label:
    label = pyglet.text.HTMLLabel(label,
                                  batch=batch,
                                  x=center[0], y=center[1],
                                  anchor_x='center', anchor_y='center')
  
  # Create collision box
  col_points = pos._get_box()
  center = (pos.x + pos.w // 2, pos.y + pos.h // 2)
  col_points = list(map(lambda x: Vector(x[0] - center[0], x[1] - center[1]), col_points))
  collision_box = Poly(Vector(center[0], center[1]), col_points)
  rectangle = primitives.Rectangle(x, y, width, height, style, rotate)
  rectangle.add_to_batch(batch)
  return ([pos, Collidable(shape=collision_box)], style)