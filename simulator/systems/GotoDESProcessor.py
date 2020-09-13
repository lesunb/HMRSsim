import esper
from typing import NamedTuple

from components.Path import Path
from components.POI import POI
from components.Position import Position

GOTO_PAYLOAD = NamedTuple('GotoPayload', [('entity', int), ('target', int)])
GOTO_EVENT_TAG = 'GoToEvent'


def process(kwargs):
    event_store = kwargs.get('EVENT_STORE', None)
    world = kwargs.get('WORLD', None)
    if event_store is None:
        raise Exception("Can't find eventStore")
    while True:
        # Gets next goto event
        event = yield event_store.get(lambda ev: ev.type == GOTO_EVENT_TAG)
        payload: GOTO_PAYLOAD = event.payload
        entity_poi = world.component_for_entity(payload.entity, POI)
        # Target point
        # TODO: Safety check on index
        target = entity_poi.points[payload.target]
        # Position of entity
        # TODO: Verify if entity is movable
        entity_pos = world.component_for_entity(payload.entity, Position)
        source = (entity_pos.x, entity_pos.y)
        if target == source:
            print("WARN - Already at destination")
        # Now we create a path and add it to the entity
        new_path = Path([source, target])
        world.add_component(payload.entity, new_path)










