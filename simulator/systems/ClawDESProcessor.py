from enum import Enum

from typing import NamedTuple
from esper import World
from simpy import FilterStore

from components.Position import Position
from components.Claw import Claw
from components.Collidable import Collidable
from components.Pickable import Pickable
from components.Inventory import Inventory

from collision import collide
from primitives import Ellipse
from utils import helpers

import systems.ManageObjects as ObjectManager


class ClawOps(Enum):
    GRAB = 'Grab'
    DROP = 'Drop'


GRAB_ClawPayload = NamedTuple('ClawGrabPayload', [('op', str), ('obj', str), ('me', int)])
ClawTag = 'ClawAction'

EVENT = NamedTuple('Event', [('type', str), ('payload', object)])

_EVENT_STORE: FilterStore
_WORLD: World


def process(kwargs):
    global _EVENT_STORE
    global _WORLD
    _EVENT_STORE = kwargs.get('EVENT_STORE', None)
    _WORLD = kwargs.get('WORLD', None)
    if _EVENT_STORE is None:
        raise Exception("Can't find eventStore")
    while True:
        event = yield _EVENT_STORE.get(lambda ev: ev.type == ClawTag)
        op = event.payload.op
        if op == ClawOps.GRAB:
            _pick_object(event.payload.obj, event.payload.me)
        elif op == ClawOps.DROP:
            _drop_object(event.payload.obj, event.payload.me)


def _pick_object(obj_name, me):
    pos = _WORLD.component_for_entity(me, Position)
    claw = _WORLD.component_for_entity(me, Claw)
    # Create boundaries, if necessary
    if claw.boundaries is None:
        span = Ellipse(pos.center, claw.max_range, claw.max_range)
        col = Collidable(shape=helpers.collision_from_points(span, pos.center))
        claw.boundaries = col
    # For every pickable component, see if it's within range
    for _, (pick, col) in _WORLD.get_components(Pickable, Collidable):
        if pick.name == obj_name:
            # This is the object we want. Let's see if it's in range and under limit weight
            for s1 in col.shapes:
                if collide(claw.boundaries.shapes[0], s1):
                    if pick.weight <= claw.max_weight:
                        # Take the object
                        payload = ObjectManager.GrabPayload(obj_name, ObjectManager.ObjectManagerOps.REMOVE)
                        event = EVENT(ObjectManager.MANAGER_TAG, payload)
                        # TODO: Add response to this request to see if removal was OK
                        # For example, maybe I'm trying to pick up something that other robots want
                        # So I don't know if I've actually picked it until the removal request is complete.
                        _EVENT_STORE.put(event)
                        # Add removed component to my inventory
                        if not _WORLD.has_component(me, Inventory):
                            _WORLD.add_component(me, Inventory())
                        inventory = _WORLD.component_for_entity(me, Inventory)
                        inventory.objects[obj_name] = pick.skeleton
                        print(f'Picked {obj_name}. My inventory: {inventory.objects}')
                        return True
                    else:
                        print(f'Pickable {obj_name} too heavy. Max weight:{claw.max_weight}. Object weight: {pick.weight}')
                else:
                    print(f'Pickable {obj_name} not within claw range!')


def _drop_object(obj_name, me):
    pos = _WORLD.component_for_entity(me, Position)
    inventory = _WORLD.component_for_entity(me, Inventory)
    skeleton = inventory.objects.get(obj_name, None)
    if skeleton is None:
        print(f'Not holding object {obj_name}')
    drop_offset = (pos.center[0], pos.center[1] + pos.h)
    drop_payload = ObjectManager.DropPayload(
        obj_name,
        ObjectManager.ObjectManagerOps.RECREATE,
        skeleton,
        drop_offset
    )
    _EVENT_STORE.put(EVENT(ObjectManager.MANAGER_TAG, drop_payload))
