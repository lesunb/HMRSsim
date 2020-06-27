import pyglet
import simulator.utils.helpers as helpers

class Rectangle:
  @staticmethod
  def from_mxCell(el):
    style = helpers.parse_style(el.attrib['style'])
    parent = el.attrib['parent']
    geometry = el[0]
    x, y = geometry.attrib['x'], geometry.attrib['y']
    width, height = geometry.attrib['width'], geometry.attrib['height']
    return Rectangle(x, y, width, height, style)


  def __init__(self, x, y, width, height, style={}):
    self.x = x
    self.y = y
    self.width = width
    self.height = height
    self.style = style

  def __str__(self):
    return "Rectangle[({}, {}) {} {}]".format(self.x, self.y, self.width, self.height)

  def add_to_batch(self, batch):
    x = self.x
    y = self.y
    width = self.width
    height = self.height
    color = list(map(
                    lambda x: int(x*255),
                    helpers.hex_to_rgb(self.style.get('fillColor', '#000000'))
                    ))
    batch.add(4, pyglet.gl.GL_QUADS, None,
      ('v2i', (x, y, x+width, y, x+width, y+height, x, y+height)),
      ('c3B', (color[0], color[1], color[2], color[0], color[1], color[2], color[0], color[1], color[2], color[0], color[1], color[2]))
    )

class Line:
  @staticmethod
  def from_mxCell(el):
    points = []
    geometry = el[0]
    src = geometry[0]
    target = geometry[1]
    if len(geometry) > 2:
      arr = geometry[2]
      for p in arr:
        points += [p.attrib['x'], p.attrib['y']]
    return Line([src.attrib['x'], src.attrib['y']] + points + [target.attrib['x'], target.attrib['y']])

  def __init__(self, points):
    self.points = points

  def add_to_batch(self, batch):
    points = list(map(int, self.points))
    print(len(points) // 2, points)
    batch.add(len(points) // 2, pyglet.gl.GL_LINES, None, ('v2i', points))