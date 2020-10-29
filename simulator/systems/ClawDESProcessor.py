from enum import Enum

from typing import NamedTuple, List
from esper import World
from simpy import FilterStore, Store, Environment
from main import EVENT

from components.Position import Position
from components.Claw import Claw
from components.Collidable import Collidable
from components.Pickable import Pickable
from components.Inventory import Inventory
from components.Script import Script, States

from collision import collide
from primitives import Ellipse
from utils import helpers

import systems.ManageObjects as ObjectManager


class ClawOps(Enum):
    GRAB = 'Grab'
    DROP = 'Drop'


GRAB_ClawPayload = NamedTuple('ClawGrabPayload', op=ClawOps, obj=str, me=int)
RESPONSE_ClawPayload = NamedTuple('ClawOperationDone', op=ClawOps, ent=int, success=bool, msg=str)
ClawTag = 'ClawAction'
ClawDoneTag = 'ClawAttemptComplete'

GrabInstructionTag = 'Grab'

_EVENT_STORE: FilterStore
_WORLD: World
_ENV: Environment


def process(kwargs):
    global _EVENT_STORE
    global _WORLD
    global _ENV
    _EVENT_STORE = kwargs.get('EVENT_STORE', None)
    _WORLD = kwargs.get('WORLD', None)
    _ENV = kwargs.get('ENV', None)
    if _EVENT_STORE is None:
        raise Exception("Can't find eventStore")
    while True:
        event = yield _EVENT_STORE.get(lambda ev: ev.type == ClawTag)
        op = event.payload.op
        print(f'Claw Received op {op}')
        if op == ClawOps.GRAB:
            pick_object(event.payload.obj, event.payload.me)
        elif op == ClawOps.DROP:
            _drop_object(event.payload.obj, event.payload.me)


def pick_object(obj_name: str, me: int):
    pos = _WORLD.component_for_entity(me, Position)
    claw = _WORLD.component_for_entity(me, Claw)
    success: bool = False
    msg: str = f'Object {obj_name} not found.'
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
                        reply_channel = Store(_ENV)
                        payload = ObjectManager.GrabPayload(obj_name, ObjectManager.ObjectManagerOps.REMOVE,
                                                            reply_channel)
                        event = EVENT(ObjectManager.ManagerTag, payload)
                        _EVENT_STORE.put(event)
                        # Wait for reply
                        response = yield reply_channel.get()
                        if response.get('success', False):
                            success = True
                            # Add removed component to my inventory
                            if not _WORLD.has_component(me, Inventory):
                                _WORLD.add_component(me, Inventory())
                            inventory = _WORLD.component_for_entity(me, Inventory)
                            inventory.objects[obj_name] = pick.skeleton
                            msg = f'Picked {obj_name}. My inventory: {inventory.objects}'
                        else:
                            success = False
                            msg = response.get('msg', '')
                    else:
                        msg = f'Pickable {obj_name} too heavy. Max weight:{claw.max_weight}. Object weight: {pick.weight}'
                        success = False
                else:
                    msg = f'Pickable {obj_name} not within claw range!'
                    success = False
    response = RESPONSE_ClawPayload(op=ClawOps.GRAB, ent=me, success=success, msg=msg)
    event_to_put = EVENT(ClawDoneTag, response)
    _EVENT_STORE.put(event_to_put)
    return success, msg


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
    _EVENT_STORE.put(EVENT(ObjectManager.ManagerTag, drop_payload))

def grabInstruction(ent: int, args: List[str], script: Script, event_store: FilterStore) -> States:
    _ENV.process(pick_object(obj_name=args[0], me=ent))
    script.state = States.BLOQUED
    script.expecting.append(ClawDoneTag)
    return script.state
