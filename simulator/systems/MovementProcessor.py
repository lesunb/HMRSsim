import esper

from components.Position import Position
from components.Velocity import Velocity


class MovementProcessor(esper.Processor):
    def __init__(self, minx, maxx, miny, maxy):
        super().__init__()
        self.minx = minx
        self.miny = miny
        self.maxx = maxx
        self.maxy = maxy

    def process(self, dt):
        # This will iterate over every Entity that has BOTH of these components:
        for ent, (vel, pos) in self.world.get_components(Velocity, Position):
            # Update the Renderable Component's position by it's Velocity:
            # An example of keeping the sprite inside screen boundaries. Basically,
            # adjust the position back inside screen boundaries if it is outside:
            new_x = max(self.minx, pos.x + vel.x)
            new_y = max(self.miny, pos.y + vel.y)

            if pos.x != new_x or pos.y != new_y:
                pos.changed = True
                new_x = min(self.maxx - pos.w, new_x)
                new_y = min(self.maxy - pos.h, new_y)
                pos.x = new_x
                pos.y = new_y
            else:
                pos.changed = False or pos.changed
