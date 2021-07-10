import logging
import re
import random

from collision import Vector
from esper import World
from collections import namedtuple

from simulator.typehints.dict_types import SystemArgs

from simulator.components.Skeleton import Skeleton
from simulator.components.Velocity import Velocity
from simulator.components.Position import Position
from simulator.components.Collidable import Collidable
from components.Hover import Hover, HoverState
from components.Control import ControlResponseFormat, Control

from simulator.utils.Navigation import distance
from systems.CollisionAvoidance import find_safe_route
ActionResponse = namedtuple('ActionResponse', ['control_response', 'change_state'])


def init(hover_interval=0.15, max_fix_speed=0.2, max_speed=1):

    actions = {
        HoverState.HOVERING: (hover_action, [max_fix_speed]),
        HoverState.MOVING: (movement_action, [max_speed]),
        HoverState.LANDED: (landed_action, []),
        HoverState.CRASHED: (crashed_action, [])
    }

    logging.getLogger(__name__).debug(f'Init hoverSystem with actions {actions}')

    def process(kwargs: SystemArgs):
        logger = logging.getLogger(__name__)
        world = kwargs.get('WORLD', None)
        env = kwargs.get('ENV', None)
        event_store = kwargs.get('EVENT_STORE')
        if env is None:
            raise Exception("Can't find environment")
        # Local ref most used functions for performance
        get_components = world.get_components
        component_for_ent = world.component_for_entity
        sleep = env.timeout
        #
        while True:
            all_collidables = get_components(Collidable, Position)
            for ent, (hover, pos, velocity, col) in get_components(Hover, Position, Velocity, Collidable):
                # Check collision here
                for shape in col.shapes:
                    shape.pos = Vector(*pos.center)
                    shape.angle = pos.angle
                close_entities = list(map(
                    lambda t: t[1][1],
                    filter(
                        lambda ent_and_components: ent_and_components[1][1].sector in pos.adjacent_sectors,
                        all_collidables
                    )
                ))
                hover.crowded = close_entities
                try:
                    (action, extra_args) = actions[hover.status]
                    # logger.debug(f'entity {ent} - {hover} @ {pos} - related action: {action}')
                    res: ActionResponse = action(ent, hover, pos, velocity, *extra_args)
                    if res.control_response is not None:
                        control_component = component_for_ent(1, Control)
                        response = ControlResponseFormat(ent, res.control_response)
                        control_component.channel.put(response)
                    if res.change_state is not None:
                        change_hover_state(world, ent, res.change_state)
                except KeyError:
                    logger.error(f'No action for {hover.status}')
            req = event_store.get(lambda ev: ev.type == 'genericCollision')
            switch = yield req | sleep(hover_interval)
            if req in switch:
                ev = switch[req]
                ent = ev.payload.ent
                other_ent = ev.payload.other_ent
                for d in [ent, other_ent]:
                    hover = component_for_ent(d, Hover)
                    if world.has_component(d, Velocity):
                        world.remove_component(d, Velocity)
                    if hover.status != HoverState.CRASHED:
                        control_component = component_for_ent(1, Control)
                        change_hover_state(world, d, HoverState.CRASHED)
                        warn_control = ControlResponseFormat(d, False)
                        control_component.channel.put(warn_control)
    return process


def hover_action(ent: int, hover: Hover, pos: Position, velocity: Velocity, max_fix_speed: float):
    """Tries to fix hover position to be on target."""
    target = hover.target
    drone_pos = pos.center
    if target == drone_pos:
        return ActionResponse(None, None)
    dx = target[0] - drone_pos[0]
    dy = target[1] - drone_pos[1]
    velocity.x = min(dx, max_fix_speed) if dx > 0 else max(dx, -max_fix_speed)
    velocity.y = min(dy, max_fix_speed) if dy > 0 else max(dy, -max_fix_speed)
    return ActionResponse(None, None)


def movement_action(ent: int, hover: Hover, pos: Position, velocity: Velocity, max_speed: float):
    """Moves drone to new target point. Change state to HOVERING when reaches target"""
    target = hover.target
    drone_pos = pos.center
    if distance(drone_pos, target) <= 5.0:
        return ActionResponse(True, HoverState.HOVERING)
    dx = target[0] - drone_pos[0]
    dy = target[1] - drone_pos[1]
    velocity.x = (min(dx, max_speed) if dx > 0 else max(dx, -max_speed)) * (1 + random.random())
    velocity.y = (min(dy, max_speed) if dy > 0 else max(dy, -max_speed)) * (1 + random.random())
    find_safe_route(hover, pos, velocity, hover.crowded)
    return  ActionResponse(None, None)


def landed_action(ent: int, hover: Hover, pos: Position, velocity: Velocity):
    return ActionResponse(None, None)


def crashed_action(ent: int, hover: Hover, pos: Position, velocity: Velocity):
    return ActionResponse(None, None)


def change_hover_state(world: World, ent: int, new_state: HoverState):
    hover = world.component_for_entity(ent, Hover)
    skeleton = world.component_for_entity(ent, Skeleton)
    hover.status = new_state
    skeleton.style = re.sub(r'fillColor=#[\d\w]{6}', f'fillColor={new_state.value[0]}', skeleton.style)
    skeleton.changed = True
