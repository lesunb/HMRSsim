import esper
import logging

from collision import Vector, collide
from simulator.components.Velocity import Velocity
from simulator.components.Collidable import Collidable
from simulator.components.Position import Position
from typing import NamedTuple
from typehints.dict_types import SystemArgs
from typehints.component_types import EVENT
from datetime import datetime, timedelta

from colorama import init, Fore
init()

COLORS = [Fore.BLUE, Fore.CYAN, Fore.RED, Fore.GREEN, Fore.YELLOW, Fore.WHITE]
CollisionPayload = NamedTuple('CollisionEvent', [('ent', int), ('other_ent', int)])


class CollisionProcessor(esper.Processor):
    def __init__(self):
        super().__init__()
        self.logger = logging.getLogger(__name__)
        self.total = timedelta()
        self.runs = 0

    def process(self, kwargs: SystemArgs):
        # start = datetime.now()
        logger = logging.getLogger(__name__)
        eventStore = kwargs.get('EVENT_STORE', None)
        all_collidables = self.world.get_components(Collidable, Position)
        for ent, (col, pos, vel) in self.world.get_components(Collidable, Position, Velocity):
            # update the position of the shape
            for shape in col.shapes:
                shape.pos = Vector(*pos.center)
                shape.angle = pos.angle
            # self.logger.debug(f'Entity {ent} - Shapes = {col.shapes}')
            # check for colision
            ents_to_check = filter(
                lambda ent_and_components: ent_and_components[1][1].sector in pos.adjacent_sectors,
                all_collidables
            )
            for otherEnt, (otherCol, otherPos) in ents_to_check:
                if otherEnt == ent:
                    continue
                if otherPos.movable:
                    for shape in otherCol.shapes:
                        shape.pos = Vector(*otherPos.center)
                        shape.angle = otherPos.angle
                if self.checkCollide(col.shapes, otherCol.shapes):
                    # Remove velocity from current entity
                    vel.x = 0
                    vel.y = 0
                    vel.alpha = 0
                    if eventStore:
                        # if col.event_tag == 'genericCollision':
                            # self.logger.debug(choice(COLORS) + f'Collision! {ent} - {otherEnt}')
                        event = EVENT(col.event_tag, CollisionPayload(ent, otherEnt))
                        # self.logger.debug(f'Firing event ent --> otherEnt: {event}')
                        eventStore.put(event)
        # end = datetime.now()
        # self.runs += 1
        # self.total += end - start
        # if self.runs % 50 == 0:
        #     logger.debug(f'runs: {self.runs}; total: {self.total}; avg = {self.total / self.runs}')

    @staticmethod
    def checkCollide(shapes1, shapes2):
        for s1 in shapes1:
            for s2 in shapes2:
                if collide(s1, s2):
                    return True
        return False
