import simulator.primitives as primitives

from collision import Poly
from components.Collidable import Collidable
from components.Position import Position
from simulator.utils.helpers import parse_style, translate_coordinates


class Wall:

  @staticmethod
  def from_mxCell(el, windowSize):
    # Parse style
    style = parse_style(el.attrib['style'])
    if style.get('shape') != 'mxgraph.floorplan.wall':
      raise Exception("Cannot create Wall from {}: shape is not mxgraph.floorplan.wall".format(el))
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
    pos = Position(x=x, y=y, w=width, h=height)
    # Create collision box
    collision_box = Poly.from_box(
                      (pos.x + pos.w // 2, pos.y + pos.h // 2),
                      pos.w, pos.h
                    )
    rectangle = primitives.Rectangle(x, y, width, height, style)
    return Wall(pos, rectangle, collision_box, parent=parent_element)

  @staticmethod
  def from_points(x, y, width, height):
    rectangle = primitives.Rectangle(x, y, width, height)
    collision_box = Poly.from_box((x, y), width, height)
    return Wall(rectangle, collision_box)


  def __init__(self,
               position: Position,
               drawing: primitives.Rectangle,
               collision: Poly,
               collidable=True,
               parent=None):

    self.draw = drawing
    self.pos = position
    if collidable:
      self.boundaries = collision

  def add_to_batch(self, batch):
    self.draw.add_to_batch(batch)

  def add_to_world(self, world):
    self.entity = world.create_entity()
    world.add_component(self.entity, Collidable(shape=self.boundaries))
    world.add_component(self.entity, self.pos)
    return self.entity
