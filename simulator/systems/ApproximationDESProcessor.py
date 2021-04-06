from esper import World
from simpy import FilterStore, Environment
from simulator.components.Script import Script
from simulator.components.Camera import Camera
from simulator.components.ApproximationHistory import ApproximationHistory
from simulator.systems.PathProcessor import EndOfApproximationTag
from simulator.components.Position import Position
from typehints.dict_types import SystemArgs
from simulator.systems.CameraProcessor import DetectedPayload
from simulator.systems.GotoDESProcessor import GotoPosEventTag, GotoPosPayload
from typehints.component_types import EVENT
import logging

_EVENT_STORE: FilterStore
_WORLD: World
_ENV: Environment

def process(kwargs: SystemArgs):
    global _EVENT_STORE
    global _WORLD
    global _ENV

    _EVENT_STORE = kwargs.get('EVENT_STORE', None)
    _WORLD = kwargs.get('WORLD', None)
    _ENV = kwargs.get('ENV', None)
    logger = logging.getLogger(__name__)
    if _EVENT_STORE is None:
        raise Exception("Can't find eventStore")
    while True:
        event = yield _EVENT_STORE.get(lambda ev: ev.type == 'Detected')
        payload: DetectedPayload = event.payload
        logger.debug(f'Approximation event {payload}')
        move_to_detected_target(payload.entity, payload.target_id)
        if _WORLD.has_component(payload.entity, Camera):
            _WORLD.remove_component(payload.entity, Camera)

def move_to_detected_target(entity, target):
    target_position = _WORLD.component_for_entity(target, Position)
    destiny_position = target_position.center

    if _WORLD.has_component(entity, ApproximationHistory):
        return

    history = ApproximationHistory(target)
    history.destiny_position = destiny_position
    _WORLD.add_component(entity, history)

    payload = GotoPosPayload(entity, destiny_position)
    new_event = EVENT(GotoPosEventTag, payload)
    _EVENT_STORE.put(new_event)

    script = _WORLD.component_for_entity(entity,Script)
    script.expecting.append(EndOfApproximationTag)
