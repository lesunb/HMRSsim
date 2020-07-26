import simulator.primitives as primitives

from collision import Poly, Vector    
from typing import List
from components.Collidable import Collidable
from components.Position import Position
from simulator.utils.helpers import *


def from_mxCell(el, batch, windowSize, lineWidth=10):
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
  # Collision points
  # TODO: Verify corners
  boxes = [
    (
      (pos.x + lineWidth // 2, pos.y + pos.h // 2),
      [(pos.x, pos.y), (pos.x, pos.y + pos.h), (pos.x + lineWidth, pos.y + pos.h), (pos.x + lineWidth, pos.y)]
    ),
    (
      (pos.x + pos.w // 2, pos.y + pos.h + lineWidth // 2),
      [(pos.x, pos.y + pos.h), (pos.x, pos.y + pos.h + lineWidth), (pos.x + pos.w, pos.y + pos.h + lineWidth), (pos.x + pos.w, pos.y + pos.h)]
    ),
    (
      (pos.x + pos.w + lineWidth // 2, pos.y + pos.h // 2),
      [(pos.x + pos.w, pos.y + pos.h), (pos.x + pos.w + lineWidth, pos.y + pos.h), (pos.x + pos.w + lineWidth, pos.y), (pos.x + pos.w, pos.y)]
    ),
    (
      (pos.x + pos.w // 2, pos.y + lineWidth // 2),
      [(pos.x, pos.y), (pos.x, pos.y + lineWidth), (pos.x + pos.w, pos.y + lineWidth), (pos.x + pos.w, pos.y)]
    )
  ]

  boxes = list(map(lambda x: Poly(tuple2vector(x[0]), get_rel_points(x[0], x[1])), boxes))
  if style.get('rotation', '') != '':
    rotate = int(style['rotation'])
    if rotate < 0:
      rotate = 360 - rotate
    for box in boxes:
      box.angle = rotate
    points = map(lambda x: rotate_around_point(x, math.radians(rotate), center), points)

  drawing = primitives.Line(list(points), style)
  drawing.add_to_batch(batch)

  return ([pos, Collidable(shape=boxes)], style)
  
