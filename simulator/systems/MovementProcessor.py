import esper
import logging

from simulator.components.Position import Position
from simulator.components.Velocity import Velocity


class MovementProcessor(esper.Processor):
    def __init__(self, minx, maxx, miny, maxy):
        super().__init__()
        self.minx = minx
        self.miny = miny
        self.maxx = maxx
        self.maxy = maxy
        self.logger = logging.getLogger(__name__)

    def process(self, env):
        # This will iterate over every Entity that has BOTH of these components:
        for ent, (vel, pos) in self.world.get_components(Velocity, Position):
            # Update the Renderable Component's position by it's Velocity:
            # An example of keeping the sprite inside screen boundaries. Basically,
            # adjust the position back inside screen boundaries if it is outside:
            # old = pos.center # DEBUG
            new_x = max(self.minx, pos.x + vel.x)
            new_y = max(self.miny, pos.y + vel.y)

            if pos.x != new_x or pos.y != new_y or vel.alpha:
                pos.changed = True
                pos.angle = (pos.angle + vel.alpha) % 360
                new_x = min(self.maxx - pos.w, new_x)
                new_y = min(self.maxy - pos.h, new_y)
                pos.x = new_x
                pos.y = new_y
                pos.center = (pos.x + pos.w // 2, pos.y + pos.h // 2)
                # self.logger.debug(f'{old} --> {pos.center}')
            else:
                pos.changed = False
