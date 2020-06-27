import esper
from random import choice
from collision import Vector, collide
from components.Velocity import Velocity
from components.Collidable import Collidable
from components.Position import Position
from colorama import init, Fore
init()

COLORS = [Fore.BLUE, Fore.CYAN, Fore.RED, Fore.GREEN, Fore.YELLOW, Fore.WHITE]
class CollisionProcessor(esper.Processor):
    def __init__(self):
        super().__init__()

    def process(self, dt):
        for ent, (col, pos, vel) in self.world.get_components(Collidable, Position, Velocity):
            # update the position of the shape
            col.shape.pos = Vector(pos.x + pos.w // 2, pos.y + pos.h // 2)
            # col.shape.pos.angle = pos.angle
            # check for colisio
            for otherEnt, (otherCol, otherPos) in self.world.get_components(Collidable, Position):
                if otherEnt == ent:
                    continue
                otherCol.shape.pos = Vector(otherPos.x + otherPos.w // 2, otherPos.y + otherPos.h // 2)
                if self.checkCollide(col.shape, otherCol.shape):
                    print(choice(COLORS) +
                          f'colision detected between {ent} and {otherEnt}')

    @staticmethod
    def checkCollide(shape1, shape2):
        return collide(shape1, shape2)
