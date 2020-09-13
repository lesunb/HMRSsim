import esper

from components.Path import Path
from components.Position import Position
from components.Velocity import Velocity


class PathProcessor(esper.Processor):
    def __init__(self):
        super().__init__()

    def process(self, kwargs):
        for ent, (pos, path, vel) in self.world.get_components(Position, Path, Velocity):
            # print(f"Processing {ent}")
            point = path.points[path.curr_point]
            pos_center = pos.center
            # print(f"[Path] Point {point} is {path.curr_point}th point")
            if pos_center[0] == point[0] and pos_center[1] == point[1]:
                # print("Going to next point")
                path.curr_point += 1
                if path.curr_point == len(path.points):
                    # end of path
                    vel.x = 0
                    vel.y = 0
                    # print("Removing Path component from", ent)
                    pos.changed = False or pos.changed
                    self.world.remove_component(ent, Path)
                    return
                point = path.points[path.curr_point]
                # print(f"Point {point} is {path.curr_point}th point")

            dx = point[0] - pos_center[0]
            if dx > 0:
                vel.x = min(path.speed, dx)
            else:
                vel.x = max(- path.speed, dx)
            dy = point[1] - pos_center[1]
            if dy > 0:
                vel.y = min(path.speed, dy)
            else:
                vel.y = max(- path.speed, dy)

