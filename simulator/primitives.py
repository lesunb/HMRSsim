import pyglet
import math
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


  def __init__(self, x, y, width, height, style={}, angle=0):
    self.center = (x, y)
    self.width = width
    self.height = height
    self.style = style
    self.angle = angle

  def __str__(self):
    return "Rectangle[({}, {}) {} {}]".format(self.x, self.y, self.width, self.height)

  def _get_points(self):
    x, y = self.center
    width = self.width
    height = self.height
    points = [(x, y), (x+width, y), (x+width, y+height), (x, y+height)]
    center = (x + width // 2, y + height // 2)
    if self.angle != 0:
        points = map(lambda x: helpers.rotate_around_point(x, math.radians(self.angle), center), points)
    return points
    
  def add_to_batch(self, batch):
    x, y = self.center
    width = self.width
    height = self.height
    color = list(map(
                    lambda x: int(x*255),
                    helpers.hex_to_rgb(self.style.get('fillColor', '#000000'))
                    ))
    points = [(x, y), (x+width, y), (x+width, y+height), (x, y+height)]
    center = (x + width // 2, y + height // 2)
    if self.angle != 0:
        points = map(lambda x: helpers.rotate_around_point(x, math.radians(self.angle), center), points)
    
    points_print = []
    colors = []
    for p in points:
      points_print.append(int(p[0]))
      points_print.append(int(p[1]))
      colors += color[:3]

    return batch.add(4, pyglet.gl.GL_QUADS, None,
      ('v2i', points_print),
      ('c3B', colors)
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
    return Line(
             [src.attrib['x'], src.attrib['y']] +
             points +
             [target.attrib['x'], target.attrib['y']]
           )

  def __init__(self, points, style={}):
    self.points = points
    self.style = style

  def __str__(self):
    r = "Line[\n"
    for p in self.points:
      r += "({},{}),\n".format(p[0], p[1])
    r += "]"
    return r

  def add_to_batch(self, batch):
    points = []
    color_array = []
    color = list(map(
                    lambda x: int(x*255),
                    helpers.hex_to_rgb(self.style.get('fillColor', '#000000'))
                    ))
    for t in self.points:
      points.append(t[0])
      points.append(t[1])
      color_array += color[0:3]

    return batch.add(
      len(points) // 2,
      pyglet.gl.GL_LINES,
      None,
      ('v2f', points),
      ('c3B', color_array))

class Ellipse():
  """General equation for ellipse: (x-h)^2 / a^2) + (y-k)^2 / b^2 = 1
  - a is radius on x-axis
  - b is radius on y-axis
  - (h,k) are the coordinates of ellipse's center
  """

  def __init__(self, center, width, height, style = {}, rotate = 0):
    self.center = center
    self.a = width // 2
    self.b = height // 2
    self.style = style
    self.angle = rotate

  # (x / a)^2 + (y / b)^2 = 1 --> (y/b)^2 = 1 - (x/a)^2 --> y = math.sqrt(1 - (x/2)^2) * b
  def add_to_batch(self, batch):

    color = list(map(
                    lambda x: int(x*255),
                    helpers.hex_to_rgb(self.style.get('fillColor', '#000000'))
                    ))
    color = color[:3]
    points = []
    rev = []
    h = self.center[0]
    k = self.center[1]
    for i in range(h - self.a, h + self.a + 1):
      y = math.sqrt(1 - ((i - h)/self.a)**2) * self.b + k
      points.append((i, y))
      rev.append((i, k - (y - k)))
    points += list(reversed(rev))

    if self.angle != 0:
        points = map(lambda x: helpers.rotate_around_point(x, math.radians(self.angle), self.center), points)

    v = []
    for p in points:
      v.append(p[0])
      v.append(p[1])
    
    return batch.add(
      len(v) // 2,
      pyglet.gl.GL_POLYGON,
      pyglet.graphics.Group(),
      ('v2f', v),
      ('c3b', color * (len(v) // 2))
    )

  def _get_points(self):
    points = []
    rev = []
    h = self.center[0]
    k = self.center[1]
    step = math.floor(self.a / 20) + 1
    for i in range(h - self.a, h + self.a + 1, step):
      y = math.sqrt(1 - ((i - h)/self.a)**2) * self.b + k
      points.append((i, y))
      rev.append((i, k - (y - k)))
    points += list(reversed(rev[1:-1]))

    if self.angle != 0:
        points = map(lambda x: helpers.rotate_around_point(x, math.radians(self.angle), self.center), points)
    return points


