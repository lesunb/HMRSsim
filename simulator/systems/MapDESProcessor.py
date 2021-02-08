import logging

from typing import NamedTuple, List

from simpy import FilterStore

from components.Map import Map
from components.Path import Path
from main import EVENT

import components.Script as scriptComponent
from systems.PathProcessor import EndOfPathTag

MapPayload = NamedTuple('MapPayload', entity=int, route=str)
MapEventTag = 'MapEvent'
MapInstructionId = 'Map'

def process(kwargs):
    logger = logging.getLogger(__name__)
    event_store = kwargs.get('EVENT_STORE', None)
    world = kwargs.get('WORLD', None)
    if event_store is None:
        raise Exception("Can't find eventStore")

    while True:
        # Gets next map event
        # TODO: Verify if entity is close to path start
        event = yield event_store.get(lambda ev: ev.type == MapEventTag)
        payload: MapPayload = event.payload
        entity_map = world.component_for_entity(1, Map)
        path_to_follow = entity_map.paths.get(payload.route, None)
        if path_to_follow is None:
            logger.debug(f"Path {payload.route} not found!")
            continue
        path_component = Path(points=path_to_follow)
        # logger.debug(f"Path to follow is {path_to_follow} ({isinstance(path_component, Path)})")
        logger.debug(f"Adding path {payload.route} to entity {payload.entity}")
        world.add_component(payload.entity, path_component)

# Functions that handle instructions
# A function returns the state of the Script component after executing
def mapInstruction(ent: int, args: List[str], script: scriptComponent.Script, event_store: FilterStore) -> scriptComponent.States:
    payload = MapPayload(ent, args[0])
    new_event = EVENT(MapEventTag, payload)
    event_store.put(new_event)
    script.state = scriptComponent.States.BLOQUED
    script.expecting.append(EndOfPathTag)
    return script.state
