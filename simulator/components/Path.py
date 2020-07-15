
import simulator.utils.helpers as helpers 
class Path:

  @staticmethod
  def from_mxCell(el, windowHeight):
    points = []
    geometry = el[0]
    src = (float(geometry[0].attrib['x']), float(geometry[0].attrib['y'])) 
    src = helpers.translate_coordinates(src, (0, windowHeight), 0)
    target = (float(geometry[1].attrib['x']), float(geometry[1].attrib['y'])) 
    target = helpers.translate_coordinates(target, (0, windowHeight), 0)
    if len(geometry) > 2:
      arr = geometry[2]
      for p in arr:
        points += [helpers.translate_coordinates((float(p.attrib['x']), float(p.attrib['y'])), (0, windowHeight), 0)]
    return  [src] + points + [target]

  def __init__(self, points, speed=5):
    self.points = points
    self.curr_point = 0
    self.speed = speed

  def __str__(self):
    return f"Path{self.points} at point {self.curr_point}"