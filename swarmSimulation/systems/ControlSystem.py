import logging
import random
from typing import List

import esper

from swarmSimulation.components.Control import Control
from swarmSimulation.components.Hover import Hover, HoverState
from swarmSimulation.systems.HoverSystem import change_hover_state
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
    logger.debug(f'MOVING DRONES TO DRONE FORMATION')
    circle_config = control_component.configs['DRONE']
    assign_positions(world, circle_config)
    control_component.awaiting = len(circle_config)
    responded = {}
    while control_component.awaiting != control_component.success + control_component.error:
        ev = yield control_component.channel.get()
        if responded.get(ev.drone, False):
            logger.debug(f'Repeated reply from drone {ev.drone}.')
            continue
        responded[ev.drone] = True
        if ev.success:
            control_component.success += 1
        else:
            control_component.error += 1
        logger.debug(
            f'[{env.now}] Drone {ev.drone} responded. Success? {ev.success}. '
            f'{control_component.success + control_component.error} / {control_component.awaiting} responses'
        )
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
    for ent, (hover, pos) in world.get_components(Hover, Position):
        components[ent] = (hover, pos)
    for point in config:
        distances = []
        for ent, (_, pos) in components.items():
            center = pos.center
            distances.append((distance(point, center), ent))
        distances.sort()
        # logger.debug(f'Point {point}: {distances}')
        for closer in distances:
            # logger.debug(f'Assigning point {point} to entity {closer[1]}')
            hover = components[closer[1]][0]
            hover.target = point
            change_hover_state(world, closer[1], HoverState.MOVING)
            components.pop(closer[1])
            break





