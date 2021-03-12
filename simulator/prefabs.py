from collision import Poly
from collections import namedtuple

from simulator.components.Collidable import Collidable
from simulator.components.Renderable import Renderable
from simulator.components.Velocity import Velocity
from simulator.components.Position import Position

import resources.load_resources as loader


def robot(world, batch, x=0, y=0, w=64, h=64):
    pos = Position(x=x, y=y, w=w, h=h)
    box_shape = createBoxShape(pos)

    entity = world.create_entity()
    world.add_component(entity, Velocity(x=0, y=0))
    world.add_component(entity, pos)
    world.add_component(entity, Renderable(
        sprite=loader.sprite(batch, x=pos.x, y=pos.y)))
    world.add_component(entity, Collidable(shape=box_shape))
    return entity


def createBoxShape(box):
    return Poly.from_box((box.x + box.w // 2, box.y + box.y // 2), box.w, box.h)

