from enum import Enum

from typing import NamedTuple, List
from esper import World
from simpy import FilterStore, Store, Environment
from typehints.component_types import EVENT
from typehints.dict_types import SystemArgs

from components.Position import Position
from components.Claw import Claw
from components.Collidable import Collidable
from components.Pickable import Pickable
from components.Inventory import Inventory
from components.Script import Script, States

from collision import collide
from primitives import Ellipse

import logging
import simulator.systems.ManageObjects as ObjectManager


class ClawOps(Enum):
    GRAB = 'Grab'
    DROP = 'Drop'


GRAB_ClawPayload = NamedTuple('ClawGrabPayload', op=ClawOps, obj=str, me=int)
RESPONSE_ClawPayload = NamedTuple('ClawOperationDone', op=ClawOps, ent=int, success=bool, msg=str)
ClawTag = 'ClawAction'
ClawDoneTag = 'ClawAttemptComplete'

GrabInstructionTag = 'Grab'
DropInstructionTag = 'Drop'

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
        event = yield _EVENT_STORE.get(lambda ev: ev.type == ClawTag)
        op = event.payload.op
        logger.debug(f'Claw Received op {op}')
        if op == ClawOps.GRAB:
            pick_object(event.payload.obj, event.payload.me)
        elif op == ClawOps.DROP:
            drop_object(event.payload.obj, event.payload.me)


def pick_object(obj_name: str, me: int):
    logger = logging.getLogger(__name__)
    pos = _WORLD.component_for_entity(me, Position)
    claw = _WORLD.component_for_entity(me, Claw)
    success: bool = False
    msg: str = f'Object {obj_name} not found.'
    # Squares are faster to create and test collision against.
    points = [
        (pos.center[0] - claw.max_range // 2, pos.center[1] - claw.max_range // 2),
        (pos.center[0] + claw.max_range // 2, pos.center[1] - claw.max_range // 2),
        (pos.center[0] + claw.max_range // 2, pos.center[1] + claw.max_range // 2),
        (pos.center[0] - claw.max_range // 2, pos.center[1] + claw.max_range // 2)
    ]
    claw_col = Collidable([(pos.center, points)])
    # For every pickable component, see if it's within range
    for _, (pick, col) in _WORLD.get_components(Pickable, Collidable):
        if pick.name == obj_name:
            # This is the object we want. Let's see if it's in range and under limit weight
            for s1 in col.shapes:
                if collide(claw_col.shapes[0], s1):
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
    if not success:
        return success, msg
    # TODO: Need to warn control module about this
    response = RESPONSE_ClawPayload(op=ClawOps.GRAB, ent=me, success=success, msg=msg)
    event_to_put = EVENT(ClawDoneTag, response)
    _EVENT_STORE.put(event_to_put)
    return success, msg


def drop_object(obj_name, me):
    pos = _WORLD.component_for_entity(me, Position)
    inventory = _WORLD.component_for_entity(me, Inventory)
    skeleton = inventory.objects.get(obj_name, None)
    success: bool = False
    msg: str = "Something went wrong"
    if skeleton is None:
        msg = f'Not holding object {obj_name}'
    else:
        reply_channel = Store(_ENV)
        drop_offset = (pos.center[0], pos.center[1] + pos.h)
        drop_payload = ObjectManager.DropPayload(
            obj_name,
            ObjectManager.ObjectManagerOps.RECREATE,
            skeleton,
            drop_offset,
            reply_channel
        )
        _EVENT_STORE.put(EVENT(ObjectManager.ManagerTag, drop_payload))
        # Wait for reply
        response = yield reply_channel.get()
        if response.get('success', False):
            success = True
            msg = ''
    response = RESPONSE_ClawPayload(op=ClawOps.DROP, ent=me, success=success, msg=msg)
    event_to_put = EVENT(ClawDoneTag, response)
    _EVENT_STORE.put(event_to_put)
    return success, msg

def grabInstruction(ent: int, args: List[str], script: Script, event_store: FilterStore) -> States:
    _ENV.process(pick_object(obj_name=args[0], me=ent))
    script.state = States.BLOCKED
    script.expecting.append(ClawDoneTag)
    return script.state

def dropInstrution(ent: int, args: List[str], script: Script, event_store: FilterStore) -> States:
    _ENV.process(drop_object(obj_name=args[0], me=ent))
    script.state = States.BLOCKED
    script.expecting.append(ClawDoneTag)
    return script.state
