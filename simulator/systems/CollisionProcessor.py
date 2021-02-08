import esper
import logging
from random import choice
from collision import Vector, collide
from components.Velocity import Velocity
from components.Collidable import Collidable
from components.Position import Position
from typing import NamedTuple
from typehints.dict_types import SystemArgs


from colorama import init, Fore
init()

COLORS = [Fore.BLUE, Fore.CYAN, Fore.RED, Fore.GREEN, Fore.YELLOW, Fore.WHITE]
EVENT = NamedTuple('Event', [('type', str), ('payload', object)])


class CollisionProcessor(esper.Processor):
    def __init__(self):
        super().__init__()
        self.logger = logging.getLogger(__name__)

    def process(self, kwargs: SystemArgs):
        eventStore = kwargs.get('EVENT_STORE', None)
        for ent, (col, pos, vel) in self.world.get_components(Collidable, Position, Velocity):
            # update the position of the shape
            for shape in col.shapes:
                shape.pos = Vector(*pos.center)
                shape.angle = pos.angle
            # self.logger.debug(f'Entity {ent} - Shapes = {col.shapes}')
            # check for colision
            for otherEnt, (otherCol, otherPos) in self.world.get_components(Collidable, Position):
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
                    self.logger.debug(choice(COLORS) +
                          f'colision detected between {ent} and {otherEnt}')
                    if eventStore:
                        if col.event_tag == 'genericColision':
                            self.logger.debug(f'Collision! {ent} - {otherEnt}')
                        event = EVENT(col.event_tag, (ent, otherEnt))
                        # self.logger.debug(f'Firing event ent --> otherEnt: {event}')
                        eventStore.put(event)

    @staticmethod
    def checkCollide(shapes1, shapes2):
        for s1 in shapes1:
            for s2 in shapes2:
                if collide(s1, s2):
                    return True
        return False
