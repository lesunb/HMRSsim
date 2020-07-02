import simulator.primitives as primitives

from collision import Poly, Vector    # If we change Collision and Collision system we might use Poly (optimized). But they need to handle more than 1 shape per entity
from components.Collidable import Collidable
from components.Position import Position
from simulator.utils.helpers import *


class Room:

  @staticmethod
  def from_mxCell(el, windowSize, lineWidth=10):
    # Parse style
    style = parse_style(el.attrib['style'])
    if style.get('shape', "") != 'mxgraph.floorplan.room':
      raise Exception("Cannot create Wall from {}: shape is not mxgraph.floorplan.room".format(el))
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
    center = (pos.x + pos.w // 2, pos.y + pos.h // 2)
    points = [
      (pos.x, pos.y),
      (pos.x, pos.y + pos.h),
      (pos.x - lineWidth // 2, pos.y + pos.h),
      (pos.x + pos.w + lineWidth // 2, pos.y + pos.h),
      (pos.x + pos.w, pos.y + pos.h),
      (pos.x + pos.w, pos.y),
      (pos.x + pos.w + lineWidth // 2, pos.y),
      (pos.x - lineWidth // 2, pos.y)
    ]

    # TODO: Maybe having the room collidable inside of it is not the best idea
    box = Poly.from_box(
                      Vector(pos.x + pos.w // 2, pos.y + pos.h // 2),
                      pos.w, pos.h
                    )

    if style.get('rotation', '') != '':
      rotate = int(style['rotation'])
      if rotate < 0:
        rotate = 360 - rotate
      box.angle = rotate
      points = map(lambda x: rotate_around_point(x, math.radians(rotate), center), points)

    drawing = primitives.Line(list(points), style)

    return Room(
      pos,
      drawing,
      box
    )

  def __init__(self,
               position: Position,
               drawing: primitives.Line,
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
