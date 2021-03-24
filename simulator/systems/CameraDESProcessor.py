from esper import World
from simpy import FilterStore, Store, Environment
from simulator.components.Detectable import Detectable
from simulator.components.Position import Position
from simulator.components.Camera import Camera
from typehints.dict_types import SystemArgs
from typing import NamedTuple
import logging
import math

_EVENT_STORE: FilterStore
_WORLD: World
_ENV: Environment

CameraPayload = NamedTuple('CameraPayload', [('entity', int),])
CameraTag = 'recognitionEvent'

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
        event = yield _EVENT_STORE.get(lambda ev: ev.type == CameraTag)
        logger.debug(f'Camera event {event.payload}')
        get_field_of_view_objects(event.payload.entity) # pegar todos os objetos que estão no raio da camera

def get_field_of_view_objects(me: int):
    if _WORLD.has_component(me, Camera):
        camera = _WORLD.component_for_entity(me, Camera)
        camera.captured_entities = []
    else:
        raise Exception("Can't find Camera Component.")

    if _WORLD.has_component(me, Position):
        me_pos = _WORLD.component_for_entity(me, Position)
    else:
        raise Exception("Can't find Position Component.")
    
    for entity, (detect, other_pos) in _WORLD.get_components(Detectable, Position):
        if detect.detectable is True:
            if is_in_the_radius(me_pos, other_pos, camera.radius):
                camera.captured_entities.append(entity)

def is_in_the_radius(me_pos, other_pos, radius):
    c = me_pos.center
    p = other_pos.center

    if ((c[0] - p[0]) ** 2) + ((c[1] - p[1]) ** 2) > (radius ** 2):
        return False
    return True
