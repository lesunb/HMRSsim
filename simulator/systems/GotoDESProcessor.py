import esper
import logging

from typing import NamedTuple, List, Union, Callable

from simulator.components.Map import Map
from typehints.component_types import Point
from typehints.dict_types import SystemArgs

from simpy import FilterStore

import simulator.components.Script as scriptComponent
from simulator.components.Path import Path
from simulator.components.Position import Position
from simulator.components.Script import Script

from simulator.systems.NavigationSystem import find_route

from simulator.typehints.component_types import EVENT, ERROR

from simulator.systems.PathProcessor import EndOfPathTag
from simulator.utils.Navigation import PathNotFound, add_nodes_from_points

GotoPoiPayload = NamedTuple('GotoPoiPayload', [('entity', int), ('target', str)])
GotoPosPayload = NamedTuple('GotoPosPayload', [('entity', int), ('target', list)])
GotoPoiEventTag = 'GoToPoiEvent'
GotoPosEventTag = 'GoToPosEvent'
GotoInstructionId = 'Go'
NavigationFunction = Callable[[Map, Point, Point], Path]

PathErrorTag = 'PathError'
PathErrorPayload = NamedTuple('PathErrorPayload', [('error', str), ('entity', int), ('best_path', Path)])
PathNotFoundTag = 'PathNotFound'
PoiNotFoundTag = 'PoiNotFound'


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
            # logger.debug(f'Event received {event}')
            payload: Union[GotoPoiPayload, GotoPosPayload] = event.payload
            if event.type == GotoPoiEventTag:
                # Target point
                try:
                    target = world_map.pois[payload.target]
                except KeyError:
                    logger.error(f'POI {payload.target} does not exist in map')
                    new_event = ERROR(
                        PathErrorTag,
                        payload.entity,
                        PathErrorPayload(PoiNotFoundTag, payload.entity, payload.target)
                    )
                    event_store.put(new_event)
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
                # Expand map with the points found
                # logger.debug(f'Update map with path {new_path}')
                add_nodes_from_points(world_map, new_path.points)
                logger.debug(f'Add Path component to entity {payload.entity} - {new_path}')
                world.add_component(payload.entity, new_path)
            except PathNotFound as err:
                logger.warning(f'Failed to go to point (entity {payload.entity}) - {err.message}')
                best_path = err.partial_path
                new_event = ERROR(
                    PathErrorTag,
                    payload.entity,
                    PathErrorPayload(PathNotFoundTag, payload.entity, best_path)
                )
                event_store.put(new_event)
                logger.warning(f'Best path - {best_path}')
                continue

    return process


# Functions that handle instructions
# A function returns the state of the Script component after executing
def goInstruction(ent: int, args: List[str], script: scriptComponent.Script,
                  event_store: FilterStore) -> scriptComponent.States:
    if len(args) == 1:
        payload = GotoPoiPayload(ent, args[0])
        new_event = EVENT(GotoPoiEventTag, payload)
    elif len(args) == 2:
        payload = GotoPosPayload(ent, [float(args[0]), float(args[1])])
        new_event = EVENT(GotoPosEventTag, payload)
    else:
        raise Exception('GO instruction failed. Go <poi> OR Go <x> <y>')
    event_store.put(new_event)
    # Needs to block the script
    script.state = scriptComponent.States.BLOCKED
    script.expecting.append(EndOfPathTag)
    return script.state


def handle_PathError(payload: PathErrorPayload, kwargs: SystemArgs):
    logger = logging.getLogger(__name__)

    if payload.error == PathNotFoundTag:
        world: esper.World = kwargs.get("WORLD")
        logger.info(f'Add best path {payload.best_path} to entity {payload.entity}')
        world.add_component(payload.entity, payload.best_path)
        # Update the script
        script = world.component_for_entity(payload.entity, Script)
        script.logs.append(f'Add best path {payload.best_path}.')
    else:
        logger.error(f"Can't solve POI not found. Missing POI is {payload.best_path}")
