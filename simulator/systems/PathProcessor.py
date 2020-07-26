import esper

from components.Path import Path
from components.Position import Position

class PathProcessor(esper.Processor):
    def __init__(self):
        super().__init__()

    def process(self, dt):

      for ent, (pos, path) in self.world.get_components(Position, Path):
        #print(f"Processing {ent}")
        point = path.points[path.curr_point]
        pos_center = pos.center
        print(f"[Path] Point {point} is {path.curr_point}th point")
        if pos_center[0] == point[0] and pos_center[1] == point[1]:
          print("Going to next point")
          path.curr_point += 1
          if path.curr_point == len(path.points):
            # end of path
            print("Removing Path component from", ent)
            pos.changed = False or pos.changed
            self.world.remove_component(ent, Path)
            return
          point = path.points[path.curr_point]
          print(f"Point {point} is {path.curr_point}th point")

        dx = point[0] - pos_center[0]
        if dx > 0:
          dx = min(path.speed, dx)
        else:
          dx = max(- path.speed, dx)
        dy = point[1] - pos_center[1]
        if dy > 0:
          dy = min(path.speed, dy)
        else:
          dy = max(- path.speed, dy)

        print(f"[Path] Pos: ({pos.x},{pos.y}). Delta: ({dx}, {dy})")
        pos.x += dx
        pos.y += dy
        pos.center = (pos.x + pos.w // 2, pos.y + pos.h // 2)
        pos.changed = True
        