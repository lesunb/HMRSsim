import esper
import logging

from typing import NamedTuple, List, Union, Callable

from components.Map import Map
from typehints.component_types import Point
from typehints.dict_types import SystemArgs

from simpy import FilterStore

import components.Script as scriptComponent
from components.Path import Path
from components.Position import Position

from systems.NavigationSystem import find_route

from main import EVENT

from systems.PathProcessor import EndOfPathTag
from utils.Navigation import PathNotFound

GotoPoiPayload = NamedTuple('GotoPoiPayload', [('entity', int), ('target', str)])
GotoPosPayload = NamedTuple('GotoPosPayload', [('entity', int), ('target', list)])
GotoPoiEventTag = 'GoToPoiEvent'
GotoPosEventTag = 'GoToPosEvent'
GotoInstructionId = 'Go'
NavigationFunction = Callable[[Map, Point, Point], Path]


def init(navigation_function: NavigationFunction = find_route):
    def process(kwargs: SystemArgs):
        logger = logging.getLogger(__name__)
        event_store = kwargs.get('EVENT_STORE', None)
        world = kwargs.get('WORLD', None)
        world_map = world.component_for_entity(1, Map)
        if event_store is None:
            raise Exception("Can't find eventStore")
        while True:
            # Gets next goto event
            event = yield event_store.get(lambda ev: ev.type in [GotoPoiEventTag, GotoPosEventTag])
            payload: Union[GotoPoiPayload, GotoPosPayload] = event.payload
            if event.type == GotoPoiEventTag:
                # Target point
                try:
                    target = world_map.pois[payload.target]
                except KeyError:
                    logger.error(f'POI {payload.target} does not exist in map')
                    # TODO: Fire event for control system
                    continue
            else:
                target = tuple(map(lambda p: float(p), payload.target))
            # Position of entity
            entity_pos = world.component_for_entity(payload.entity, Position)
            source = entity_pos.center
            if target == source:
                logger.warning("WARN - Already at destination")
                continue
            # Now we create a path and add it to the entity
            try:
                new_path = navigation_function(world_map, source, target)
            except PathNotFound as err:
                logger.warning(f'Failed to go to point (entity {payload.entity}) - {err.message}')
                # TODO: Check how to handle this
                best_path = err.partial_path
                logger.warning(f'Best path - {best_path}')
                continue
            logger.debug(f'Adding Path component to entity {payload.entity} - {new_path}')
            world.add_component(payload.entity, new_path)

    return process


# Functions that handle instructions
# A function returns the state of the Script component after executing
def goInstruction(ent: int, args: List[str], script: scriptComponent.Script,
                  event_store: FilterStore) -> scriptComponent.States:
    logger = logging.getLogger(__name__)
    # logger.debug(f'Go instruction with args {args}')
    if len(args) == 1:
        payload = GotoPoiPayload(ent, args[0])
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
