import esper
import logging

from typing import NamedTuple, List
from simpy import FilterStore

import components.Script as scriptComponent
from components.Path import Path
from components.POI import POI
from components.Position import Position

from main import EVENT

from systems.PathProcessor import EndOfPathTag
GotoPayload = NamedTuple('GotoPayload', [('entity', int), ('target', int)])
GotoEventTag = 'GoToEvent'
GotoInstructionId = 'Go'


def process(kwargs):
    logger = logging.getLogger(__name__)
    event_store = kwargs.get('EVENT_STORE', None)
    world = kwargs.get('WORLD', None)
    if event_store is None:
        raise Exception("Can't find eventStore")
    while True:
        # Gets next goto event
        event = yield event_store.get(lambda ev: ev.type == GotoEventTag)
        payload: GotoPayload = event.payload
        entity_poi = world.component_for_entity(payload.entity, POI)
        # Target point
        # TODO: Safety check on index
        target = entity_poi.points[payload.target]
        # Position of entity
        # TODO: Verify if entity is movable
        entity_pos = world.component_for_entity(payload.entity, Position)
        source = (entity_pos.x, entity_pos.y)
        if target == source:
            logger.warning("WARN - Already at destination")
        # Now we create a path and add it to the entity
        new_path = Path([source, target])
        world.add_component(payload.entity, new_path)


# Functions that handle instructions
# A function returns the state of the Script component after executing
def goInstruction(ent: int, args: List[str], script: scriptComponent.Script, event_store: FilterStore) -> scriptComponent.States:
    poi = int(args[0])
    payload = GotoPayload(int(ent), int(poi))
    new_event = EVENT(GotoEventTag, payload)
    event_store.put(new_event)
    # Needs to block the script
    script.state = scriptComponent.States.BLOQUED
    script.expecting.append(EndOfPathTag)
    return script.state







