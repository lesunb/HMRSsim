import esper
import logging

from datetime import datetime, timedelta

from simulator.components.Position import Position
from simulator.components.Velocity import Velocity


class MovementProcessor(esper.Processor):
    def __init__(self, minx, maxx, miny, maxy, sector_size=50):
        super().__init__()
        self.minx = minx
        self.miny = miny
        self.maxx = maxx
        self.maxy = maxy
        self.sector_size = sector_size
        self.logger = logging.getLogger(__name__)
        self.total = timedelta()
        self.runs = 0
        self.created_tiles = False

    def process(self, env):
        logger = logging.getLogger(__name__)
        # start = datetime.now()

        # The Movement Processor is responsible for managing the tiling of the simulation
        # When it starts (which is after the simulation is loaded) it will initialize the sector
        # Of all entities that have a position
        # This is done just once in the first execution
        if not self.created_tiles:
            for ent, (pos,) in self.world.get_components(Position):
                pos.sector = ((pos.y // self.sector_size) * self.maxx) + (pos.x // self.sector_size)
            self.created_tiles = True

        # This will iterate over every Entity that has BOTH of these components:
        for ent, (vel, pos) in self.world.get_components(Velocity, Position):
            # old = pos.center # DEBUG
            new_x = max(self.minx, pos.x + vel.x)
            new_y = max(self.miny, pos.y + vel.y)

            if pos.x != new_x or pos.y != new_y or vel.alpha:
                # print(f'MOVE {ent} - vel {vel}')
                pos.changed = True
                pos.angle = (pos.angle + vel.alpha) % 360
                new_x = min(self.maxx - pos.w, new_x)
                new_y = min(self.maxy - pos.h, new_y)
                pos.x = new_x
                pos.y = new_y
                pos.center = (pos.x + pos.w // 2, pos.y + pos.h // 2)
                pos.sector = ((pos.y // self.sector_size) * self.maxx) + (pos.x // self.sector_size)
                pos.adjacent_sectors = [
                    (pos.sector + dx + dy)
                    for dx in [-1, -1, -1, 1, 1, 1, 0, 0, 0]
                    for dy in [0, -self.maxx, +self.maxx, 0, -self.maxx, +self.maxx, 0, -self.maxx, +self.maxx]
                ]
                # self.logger.debug(f'{old} --> {pos.center}')
            else:
                pos.changed = False
        # end = datetime.now()
        # self.runs += 1
        # self.total += end - start
        # if self.runs % 50 == 0:
        #     logger.debug(f'runs: {self.runs}; total: {self.total}; avg = {self.total / self.runs}')

    def add_sector_info(self, pos: Position):
        pos.sector = ((pos.y // self.sector_size) * self.maxx) + (pos.x // self.sector_size)