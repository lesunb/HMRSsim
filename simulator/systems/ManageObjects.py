from typing import NamedTuple
from enum import Enum
from components.Renderable import Renderable

class ObjectManagerOps(Enum):
    REMOVE = 'remove'
    RECREATE = 'recreate'

MANAGER_EVENT = NamedTuple('ManagerPayload', [('entity', int), ('op', str)])
MANAGER_TAG = 'ManagerEventTag'


def process(kwargs):
    event_store = kwargs.get('EVENT_STORE', None)
    world = kwargs.get('WORLD', None)
    if event_store is None:
        raise Exception("Can't find eventStore")

    while True:
        event = yield event_store.get(lambda ev: ev.type == MANAGER_TAG)
        payload: MANAGER_EVENT = event.payload
        if payload.op == ObjectManagerOps.REMOVE:
            remove_entity(payload.entity, world)


def remove_entity(ent, world):
    if ent == -1:
        print(f'No entity.')
        return
    r = world.component_for_entity(ent, Renderable)
    if r.is_primitive:
        r.sprite.delete()
    world.delete_entity(ent)
