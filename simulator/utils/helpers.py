import math
from collision import Vector

def hex_to_rgb(hex_color):
  r = hex_color[1:3]
  g = hex_color[3:5]
  b = hex_color[5:]
  return (int(r, 16) / 255, int(g, 16) / 255, int(b, 16) / 255, 0.0)

def parse_style(style):
  s = {}
  items = style.split(';')
  for item in items:
    if item == "":
      continue
    elif '=' not in item:
      s[item] = True
      continue
    [key, value] = item.split('=')
    s[key] = value
  return s

def translate_coordinates(coordinates, dimens, height):
  y = dimens[1] - coordinates[1] - height
  return (coordinates[0], y)

def get_rel_points(center, points):
  return list(map(lambda x: Vector(x[0] - center[0], x[1] - center[1]), points))

def tuple2vector(x):
  return Vector(x[0], x[1])
  
def rotate_around_point(xy, radians, origin=(0, 0)):
    """Rotate a point around a given point.
    
    I call this the "high performance" version since we're caching some
    values that are needed >1 time. It's less readable than the previous
    function but it's faster.
    From: https://gist.github.com/LyleScott/e36e08bfb23b1f87af68c9051f985302
    """
    x, y = xy
    offset_x, offset_y = origin
    adjusted_x = (x - offset_x)
    adjusted_y = (y - offset_y)
    cos_rad = math.cos(radians)
    sin_rad = math.sin(radians)
    qx = offset_x + cos_rad * adjusted_x + sin_rad * adjusted_y
    qy = offset_y + -sin_rad * adjusted_x + cos_rad * adjusted_y

    return qx, qy