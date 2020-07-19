import simulator.primitives as primitives

from collision import Poly, Vector
from components.Collidable import Collidable
from components.Position import Position
from components.Renderable import Renderable
from components.Velocity import Velocity
from simulator.utils.helpers import parse_style, translate_coordinates


class Shape:

  @staticmethod
  def from_object(el, windowSize, lineWidth=10):
    options = el.attrib
    if 'collidable' not in options:
      options['collidable'] = True
    s = Shape.from_mxCell(el[0], windowSize, lineWidth)
    s.options = options
    return s

  @staticmethod
  def from_mxCell(el, windowSize, lineWidth=10):
    # Parse style
    style = parse_style(el.attrib['style'])
    # Get parent
    parent_element = el.attrib['parent']
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

    col_points = list(map(lambda x: Vector(x[0] - center[0], x[1] - center[1]), col_points))
    collision_box = Poly(Vector(center[0], center[1]), col_points)

    return Shape(pos, draw, collision_box, parent=parent_element)

  # TODO: Update to make accept Ellipses
  @staticmethod
  def from_points(x, y, width, height):
    rectangle = primitives.Rectangle(x, y, width, height)
    collision_box = Poly.from_box((x, y), width, height)
    return Wall(rectangle, collision_box)


  def __init__(self,
               position: Position,
               drawing,
               collision: Poly,
               parent=None,
               options=None):

    if options == None:
      self.options = {
        'colidable': True,
        'movable': False
      }
    else:
      self.options = options
    self.draw = drawing
    self.pos = position
    self.boundaries = collision
    self.batch_draw = None

  def add_to_batch(self, batch):
    if self.batch_draw == None:
      self.batch_draw = self.draw.add_to_batch(batch)
    return self.batch_draw

  def add_to_world(self, world):
    pos = self.pos
    self.entity = world.create_entity()

    world.add_component(self.entity, self.pos)
    if self.options.get('collidable', True):
      world.add_component(self.entity, Collidable(shape=self.boundaries))
    if self.options.get('movable', False):
      center = pos.x + pos.w // 2, pos.y + pos.h // 2
      world.add_component(self.entity, Renderable(sprite=self.batch_draw, primitive=True, center=center))
      world.add_component(self.entity, Velocity())
    return self.entity