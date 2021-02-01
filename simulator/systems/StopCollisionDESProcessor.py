import esper
import simpy
from components.Position import Position
from components.Path import Path
from typehints.dict_types import SystemArgs

from main import EVENT
from systems.PathProcessor import EndOfPathTag, EndOfPathPayload
StopEventTag = 'stopEvent'
GenericCollisionTag = 'genericCollision'


def process(kwargs: SystemArgs):
    event_store = kwargs.get('EVENT_STORE', None)
    world: esper.World = kwargs.get('WORLD', None)
    env: simpy.Environment = kwargs.get('ENV', None)
    if event_store is None:
        raise Exception("Can't find eventStore")

    while True:
        # Gets next collision event
        event = yield event_store.get(lambda ev: ev.type == StopEventTag or ev.type == GenericCollisionTag)
        if event.type == GenericCollisionTag:
            continue
        (ent, otherEnt) = event.payload
        pos = world.component_for_entity(ent, Position)
        other_pos = world.component_for_entity(otherEnt, Position)
        (mx, my) = pos.center
        (ox, oy) = other_pos.center

        if mx < ox:
            # I'm on the left of the other entity
            pos.x = other_pos.x - 1
        elif mx > ox:
            # I'm on the right
            pos.x = other_pos.x + other_pos.w + 1
        # TODO: Handle case when they are in the same X coordinate?...
        # TODO: Handle diagonal contact with round shapes
        if my < oy:
            pos.y = other_pos.y - 1
        elif my > oy:
            pos.y = other_pos.y + other_pos.h + 1
        # TODO: Handle case when they are in the same Y coordinate?...

        # If following path, remove it
        if world.has_component(ent, Path):
            end_of_path = EVENT(EndOfPathTag, EndOfPathPayload(ent, str(env.now)))
            event_store.put(end_of_path)
            world.remove_component(ent, Path)


