from esper import World
from simpy import FilterStore, Store, Environment
from simulator.components.Detectable import Detectable
from simulator.components.Position import Position
from simulator.components.Camera import Camera
from typehints.dict_types import SystemArgs
from typing import NamedTuple
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
        print("Approximation System")
        event = yield _EVENT_STORE.get(lambda ev: ev.type == 'Detected')
        logger.debug(f'Approximation event {event.payload}')