import logging
from typing import List

import esper

from swarmSimulation.components.Control import Control, ControlResponseFormat
from swarmSimulation.components.Hover import Hover, HoverState
from simulator.utils.Navigation import distance

from simulator.components.Position import Position


# The controller for now
from typehints.component_types import Point
from typehints.dict_types import SystemArgs


def control(kwargs: SystemArgs):
    logger = logging.getLogger(__name__)
    world = kwargs['WORLD']
    env = kwargs['ENV']
    kill_switch = kwargs['_KILL_SWITCH']
    control_component: Control = world.component_for_entity(1, Control)
    logger = logging.getLogger(__name__)
    for drone, (hover, pos) in world.get_components(Hover, Position):
        hover.target = (pos.center[0], pos.center[1])
        hover.status = HoverState.HOVERING
    yield env.timeout(1)
    logger.debug(f'MOVING DRONES TO CIRCLE FORMATION')
    circle_config = control_component.configs['CIRCLE']
    assign_positions(world, circle_config)
    control_component.awaiting = len(circle_config)
    while control_component.awaiting != control_component.success + control_component.error:
        ev = yield control_component.channel.get()
        if ev.success:
            control_component.success += 1
        else:
            control_component.error += 1
        logger.debug(f'Drone {ev.drone} responded. Success? {ev.success}')
    logger.debug(
        f'All {control_component.awaiting} drones responded.'
        f'Success: {control_component.success}  Fail: {control_component.error}'
    )
    control_component.awaiting = 0
    control_component.success = 0
    control_component.error = 0
    yield env.timeout(2)
    kill_switch.succeed()


def assign_positions(world: esper.World, config: List[Point]):
    logger = logging.getLogger(__name__)
    components = {}
    assigned = {}
    for point in config:
        distances = []
        for ent, (hover, pos) in world.get_components(Hover, Position):
            if assigned.get(ent, False):
                continue
            components[ent] = (hover, pos)
            center = pos.center
            distances.append((distance(point, center), ent))
        distances.sort()
        # logger.debug(f'Point {point}: {distances}')
        for closer in distances:
            if assigned.get(closer[1], False):
                continue
            # logger.debug(f'Assigning point {point} to entity {closer[1]}')
            hover = components[closer[1]][0]
            hover.target = point
            hover.status = HoverState.MOVING
            assigned[closer[1]] = True
            break





