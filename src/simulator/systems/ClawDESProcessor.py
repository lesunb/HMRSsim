from enum import Enum
from abc import abstractmethod

from moveit_msgs.action import Pickup, Place

from typing import NamedTuple, List
from esper import World
from simpy import FilterStore, Store, Environment
from rclpy.action import CancelResponse, GoalResponse
from rclpy.action.server import ServerGoalHandle

from simulator.typehints.component_types import EVENT
from simulator.typehints.dict_types import SystemArgs
from simulator.typehints.ros_types import RosTopicServer, RosActionServer
from simulator.components.Velocity import Velocity
from simulator.components.NavToPoseRosGoal import NavToPoseRosGoal
from simulator.components.Position import Position
from simulator.components.Claw import Claw
from simulator.components.Collidable import Collidable
from simulator.components.Pickable import Pickable
from simulator.components.Inventory import Inventory
from simulator.components.Script import Script, States
import simulator.systems.ManageObjects as ObjectManager

from collision import collide

import logging


class RosClawService(RosActionServer):

    @abstractmethod
    def execute_goal(self, succeed=False, msg=None):
        pass

class ClawOps(Enum):
    GRAB = 'Grab'
    DROP = 'Drop'


GRAB_ClawPayload = NamedTuple('ClawGrabPayload', op=ClawOps, obj=str, me=int, ros_service=RosClawService)
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
            yield from pick_object(event.payload.obj, event.payload.me, event.payload.ros_service)
        elif op == ClawOps.DROP:
            yield from drop_object(event.payload.obj, event.payload.me, event.payload.ros_service)


def pick_object(obj_name: str, me: int, ros_service: RosClawService = None):
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
    if ros_service is not None:
        ros_service.execute_goal(success, msg)
    if not success:
        return success, msg
    # TODO: Need to warn control module about this
    response = RESPONSE_ClawPayload(op=ClawOps.GRAB, ent=me, success=success, msg=msg)
    event_to_put = EVENT(ClawDoneTag, response)
    _EVENT_STORE.put(event_to_put)
    return success, msg


def drop_object(obj_name, me, ros_service: RosClawService = None):
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
            reply_channel,
            pos.sector
        )
        _EVENT_STORE.put(EVENT(ObjectManager.ManagerTag, drop_payload))
        # Wait for reply
        response = yield reply_channel.get()
        if response.get('success', False):
            success = True
            msg = ''
    if ros_service is not None:
        ros_service.execute_goal(success, msg)
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


def create_grab_and_drop_for_each_robot(world, event_store):
    services = []
    for ent, (vel, pos, ros_goal) in world.get_components(Velocity, Position, NavToPoseRosGoal):
        grab = RosClawGrabService(
            event_store=event_store, world=world, robot_name=ros_goal.name)
        drop = RosClawDropService(
            event_store=event_store, world=world, robot_name=ros_goal.name)
        services.append(grab)
        services.append(drop)
    return services


class RosClawGrabService(RosClawService):

    def __init__(self, **kwargs):
        super().__init__()
        self.logger = logging.getLogger(__name__)
        self.event_store = kwargs.get('event_store', None)
        self.world = kwargs.get('world', None)
        self.robot_name = kwargs.get('robot_name', None)
        self.goal_handle = None
        self.succeed = False
        self.msg = ''

    def get_name(self):
        return self.robot_name + "/grab"

    def result_callback(self, goal_handle: ServerGoalHandle):
        self.update_goal_handle(goal_handle)
        result = Pickup.Result()
        result.trajectory_descriptions = [self.msg]
        return result

    def update_goal_handle(self, goal_handle: ServerGoalHandle):
        if self.succeed:
            goal_handle.succeed()
            return
        goal_handle.abort()

    def get_execute_callback(self):
        return self.result_callback

    def execute_goal(self, succeed=False, msg=None):
        if self.goal_handle is None:
            return
        self.succeed = succeed
        self.msg = msg
        self.goal_handle.execute()

    def goal_callback(self, goal_request):
        entity = find_robot_in_world(self.world, self.robot_name)
        if entity is not None:
            return GoalResponse.ACCEPT
        self.logger.warn(f"Coudn't find a robot named {self.robot_name}")
        return GoalResponse.REJECT

    def get_goal_callback(self):
        return self.goal_callback

    def handle_accepted_goal_callback(self, goal_handle: ServerGoalHandle):
        object = goal_handle.request.target_name
        self.logger.info(
            f'Received order for {self.robot_name} to grab {object}')
        entity = find_robot_in_world(self.world, self.robot_name)
        if entity is None:
            return
        self.goal_handle = goal_handle
        payload = GRAB_ClawPayload(
            op=ClawOps.GRAB, obj=object, me=entity, ros_service=self)
        event = EVENT(ClawTag, payload)
        self.event_store.put(event)

    def get_handle_accepted_goal_callback(self):
        return self.handle_accepted_goal_callback

    def cancel_callback(self, goal_handle: ServerGoalHandle):
        return CancelResponse.REJECT

    def get_cancel_callback(self):
        return self.cancel_callback

    def get_service_type(self):
        return Pickup


class RosClawDropService(RosClawService):

    def __init__(self, **kwargs):
        super().__init__()
        self.logger = logging.getLogger(__name__)
        self.event_store = kwargs.get('event_store', None)
        self.world = kwargs.get('world', None)
        self.robot_name = kwargs.get('robot_name', None)
        self.goal_handle = None
        self.succeed = False
        self.msg = ''

    def get_name(self):
        return self.robot_name + "/drop"

    def result_callback(self, goal_handle: ServerGoalHandle):
        self.update_goal_handle(goal_handle)
        result = Place.Result()
        result.trajectory_descriptions = [self.msg]
        return result

    def update_goal_handle(self, goal_handle: ServerGoalHandle):
        if self.succeed:
            goal_handle.succeed()
            return
        goal_handle.abort()

    def get_execute_callback(self):
        return self.result_callback

    def execute_goal(self, succeed=False, msg=None):
        if self.goal_handle is None:
            return
        self.succeed = succeed
        self.msg = msg
        self.goal_handle.execute()

    def goal_callback(self, goal_request):
        entity = find_robot_in_world(self.world, self.robot_name)
        if entity is not None:
            return GoalResponse.ACCEPT
        self.logger.warn(f"Coudn't find a robot named {self.robot_name}")
        return GoalResponse.REJECT

    def get_goal_callback(self):
        return self.goal_callback

    def handle_accepted_goal_callback(self, goal_handle: ServerGoalHandle):
        object = goal_handle.request.attached_object_name
        self.logger.info(
            f'Received order for {self.robot_name} to drop {object}')
        entity = find_robot_in_world(self.world, self.robot_name)
        if entity is None:
            return
        self.goal_handle = goal_handle
        payload = GRAB_ClawPayload(
            op=ClawOps.DROP, obj=object, me=entity, ros_service=self)
        event = EVENT(ClawTag, payload)
        self.event_store.put(event)

    def get_handle_accepted_goal_callback(self):
        return self.handle_accepted_goal_callback

    def cancel_callback(self, goal_handle: ServerGoalHandle):
        return CancelResponse.REJECT

    def get_cancel_callback(self):
        return self.cancel_callback

    def get_service_type(self):
        return Place


def find_robot_in_world(world, robot_name):
    for ent, (vel, pos, ros_goal) in world.get_components(Velocity, Position, NavToPoseRosGoal):
        if ros_goal.name == robot_name:
            return ent
    logging.getLogger(__name__).info(
        'Could not find a robot with the name ' + robot_name)
    return None
