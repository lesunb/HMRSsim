import logging
import esper

from swarmSimulation.components.Hover import Hover, HoverState
from simulator.utils.Navigation import distance

from simulator.components.Position import Position

CIRCLE = [
    (218, 208), (246, 158), (266, 158),
    (286, 168), (226, 168), (218, 188),
    (226, 228), (246, 240), (266, 240),
    (298, 188), (298, 208), (286, 228)
]


# The controller for now
def control(world, env, kill_switch):
    logger = logging.getLogger(__name__)
    for drone, (hover, pos) in world.get_components(Hover, Position):
        hover.target = (pos.center[0], pos.center[1])
        hover.status = HoverState.HOVERING
    yield env.timeout(2)
    logger.debug(f'MOVING DRONES TO CIRCLE FORMATION')
    assign_positions(world)
    yield env.timeout(5)
    kill_switch.succeed()


def assign_positions(world: esper.World):
    logger = logging.getLogger(__name__)
    components = {}
    assigned = {}
    for point in CIRCLE:
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





