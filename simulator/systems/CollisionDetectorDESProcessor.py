from simulator.components.Position import Position
from simulator.components.Collision import Collision
from typehints.dict_types import SystemArgs

def process(kwargs: SystemArgs):
    event_store = kwargs.get('EVENT_STORE', None)
    world = kwargs.get('WORLD', None)
    if event_store is None:
        raise Exception("Can't find eventStore")
    while True:
        event = yield event_store.get(lambda ev: ev.type == 'stopEvent')
        entity, other_entity = event.payload
        position = world.component_for_entity(entity, Position)
        if world.has_component(entity, Collision):
            collision = world.component_for_entity(entity, Collision)
            collision.add_collision(other_entity, kwargs.get('ENV').now, Position(position.x, position.y))