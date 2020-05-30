import esper
from collision import Vector, collide
from components.Collidable import Collidable
from components.Velocity import Velocity
from components.Position import Position


class CollisionProcessor(esper.Processor):
    def __init__(self):
        super().__init__()

    def process(self, dt):
        for ent, (col, pos, vel) in self.world.get_components(Collidable, Position, Velocity):
            # update the position of the shape
            col.shape.pos = Vector(pos.x, pos.y)
            # col.shape.pos.angle = pos.angle
            # check for colisio
            for otherEnt, (otherCol, otherPos) in self.world.get_components(Collidable, Position):
                if otherEnt == ent:
                    continue
                otherCol.shape.pos = Vector(otherPos.x, otherPos.y)
                if self.checkCollide(col.shape, otherCol.shape):
                    print(f'colision detected between {ent} and {otherEnt}')

    @staticmethod
    def checkCollide(shape1, shape2):
        return collide(shape1, shape2)
