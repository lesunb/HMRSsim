
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
    [key, value] = item.split('=')
    s[key] = value
  return s

def translate_coordinates(coordinates, dimens, height):
  y = dimens[1] - coordinates[1] - height
  return (coordinates[0], y)