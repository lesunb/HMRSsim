from collision import Poly
from collections import namedtuple

from components.Collidable import Collidable
from components.Renderable import Renderable
from components.Velocity import Velocity
from components.Position import Position


from systems.MovementProcessor import MovementProcessor
from systems.CollisionProcessor import CollisionProcessor


import resources.load_resources as loader


def robot(world, batch, x=0, y=0, w=64, h=64):
    Box = namedtuple('Box', 'x y w h')
    box = Box(x, y, w, h)
    box_shape = createBoxShape(box)

    entity = world.create_entity()

    world.add_component(entity, Velocity(x=0, y=0))
    world.add_component(entity, Position(x=box.x, y=box.x))
    world.add_component(entity, Renderable(
        sprite=loader.sprite(batch, x=x, y=y)))

    world.add_component(entity, Collidable(shape=box_shape))
    return entity


def createBoxShape(box):
    return Poly.from_box((box.x, box.y), box.w, box.h)
