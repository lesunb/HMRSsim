import esper
import logging

from typing import NamedTuple, List, Union
from typehints.dict_types import SystemArgs

from simpy import FilterStore

import components.Script as scriptComponent
from components.Path import Path
from components.POI import POI
from components.Position import Position

from main import EVENT

from systems.PathProcessor import EndOfPathTag
GotoPoiPayload = NamedTuple('GotoPoiPayload', [('entity', int), ('target', int)])
GotoPosPayload = NamedTuple('GotoPosPayload', [('entity', int), ('target', list)])
GotoPoiEventTag = 'GoToPoiEvent'
GotoPosEventTag = 'GoToPosEvent'
GotoInstructionId = 'Go'


def process(kwargs: SystemArgs):
    logger = logging.getLogger(__name__)
    event_store = kwargs.get('EVENT_STORE', None)
    world = kwargs.get('WORLD', None)
    if event_store is None:
        raise Exception("Can't find eventStore")
    while True:
        # Gets next goto event
        event = yield event_store.get(lambda ev: ev.type in [GotoPoiEventTag, GotoPosEventTag])
        payload: Union[GotoPoiPayload, GotoPosPayload] = event.payload
        if event.type == GotoPoiEventTag:
            entity_poi = world.component_for_entity(payload.entity, POI)
            # Target point
            # TODO: Safety check on index
            target = entity_poi.points[payload.target]
        else:
            target = list(map(lambda p: float(p), payload.target))
        # Position of entity
        # TODO: Verify if entity is movable
        entity_pos = world.component_for_entity(payload.entity, Position)
        source = entity_pos.center
        if target == source:
            logger.warning("WARN - Already at destination")
        # Now we create a path and add it to the entity
        new_path = Path([source, target])
        logger.debug(f'Adding Path component to entity {payload.entity} - {new_path}')
        world.add_component(payload.entity, new_path)


# Functions that handle instructions
# A function returns the state of the Script component after executing
def goInstruction(ent: int, args: List[str], script: scriptComponent.Script, event_store: FilterStore) -> scriptComponent.States:
    logger = logging.getLogger(__name__)
    # logger.debug(f'Go instruction with args {args}')
    if len(args) == 1:
        poi = int(args[0])
        payload = GotoPoiPayload(ent, poi)
        new_event = EVENT(GotoPoiEventTag, payload)
    elif len(args) == 2:
        payload = GotoPosPayload(ent, [float(args[0]), float(args[1])])
        new_event = EVENT(GotoPosEventTag, payload)
    else:
        raise Exception('GO instruction failed. Go <poi> OR Go <x> <y>')
    # logger.debug(f'New Go event - {new_event}')
    event_store.put(new_event)
    # Needs to block the script
    script.state = scriptComponent.States.BLOQUED
    script.expecting.append(EndOfPathTag)
    return script.state







