import simulator.primitives as primitives
import json

from collision import Poly, Vector
from components.Collidable import Collidable
from components.Position import Position
from components.Renderable import Renderable
from components.POI import POI
from simulator.utils.helpers import parse_style, translate_coordinates


def from_object(el, batch, windowSize, lineWidth=10):
  options = el.attrib

  components, style, draw = from_mxCell(el[0], batch, windowSize, lineWidth)
  if 'collidable' not in options:
    options['collidable'] = True
  if 'movable' not in options:
    options['movable'] = True
  if 'POI' in options:
    points = json.loads(options['POI'])
    points = list(map((lambda p: translate_coordinates(p, windowSize, 0)), points))
    components.append(POI(points=points))

  options.update(style)
  if options['movable']:
    pos = components[0]
    center = (pos.x + pos.w // 2, pos.y + pos.h // 2)
    components.append(Renderable(sprite=draw, primitive=True, center=center))
  return (components, options)


def from_mxCell(el, batch, windowSize, lineWidth=10):
  # Parse style
  style = parse_style(el.attrib['style'])
  # Get parent
  style['parent'] = el.attrib['parent']

  # Get geometry
  geometry = el[0]
  x = int(geometry.attrib.get('x', '0'))
  y = int(geometry.attrib.get('y', '0'))
  width = int(geometry.attrib['width'])
  height = int(geometry.attrib['height'])
  # Create drawing
  (x, y) = translate_coordinates((x, y), windowSize, height)
  pos = Position(x=x, y=y, w=width, h=height, movable=False)

  rotate = 0
  if style.get('rotation', '') != '':
    rotate = int(style['rotation'])
    if rotate < 0:
      rotate = 360 + rotate
  pos.angle = rotate

  draw = None
  col_points = None
  center = (pos.x + pos.w // 2, pos.y + pos.h // 2)

  if 'ellipse' in style:
    draw = primitives.Ellipse(center, width, height, style, rotate)
    col_points = draw._get_points()
  else:
    draw = primitives.Rectangle(x, y, width, height, style, rotate)
    col_points = pos._get_box()

  batch_draw = draw.add_to_batch(batch)
  col_points = list(map(lambda x: Vector(x[0] - center[0], x[1] - center[1]), col_points))
  collision_box = Poly(Vector(center[0], center[1]), col_points)

  return ([pos, Collidable(shape=collision_box)], style, batch_draw)