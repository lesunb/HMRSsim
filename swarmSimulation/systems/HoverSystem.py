import logging

from simulator.typehints.dict_types import SystemArgs

from components.Velocity import Velocity
from components.Position import Position
from swarmSimulation.components.Hover import Hover, HoverState

from simulator.utils.Navigation import distance


def init(hover_interval=0.15, max_fix_speed=0.2, max_speed=1):

    actions = {
        HoverState.HOVERING: (hover_action, [max_fix_speed]),
        HoverState.MOVING: (movement_action, [max_speed]),
        HoverState.LANDED: (landed_action, [])
    }

    logging.getLogger(__name__).debug(f'Init hoverSystem with actions {actions}')

    def process(kwargs: SystemArgs):
        logger = logging.getLogger(__name__)
        world = kwargs.get('WORLD', None)
        env = kwargs.get('ENV', None)
        if env is None:
            raise Exception("Can't find environment")
        while True:
            for ent, (hover, pos, velocity) in world.get_components(Hover, Position, Velocity):
                try:
                    (action, extra_args) = actions[hover.status]
                    # logger.debug(f'entity {ent} - {hover} @ {pos} - related action: {action}')
                    action(hover, pos, velocity, *extra_args)
                except KeyError:
                    logger.error(f'No action for {hover.status}')
            yield env.timeout(hover_interval)
    return process


def hover_action(hover: Hover, pos: Position, velocity: Velocity, max_fix_speed: float):
    """Tries to fix hover position to be on target."""
    target = hover.target
    drone_pos = pos.center
    if target == drone_pos:
        return
    dx = target[0] - drone_pos[0]
    dy = target[1] - drone_pos[1]
    velocity.x = min(dx, max_fix_speed) if dx > 0 else max(dx, -max_fix_speed)
    velocity.y = min(dy, max_fix_speed) if dy > 0 else max(dy, -max_fix_speed)


def movement_action(hover: Hover, pos: Position, velocity: Velocity, max_speed: float):
    """Moves drone to new target point. Change state to HOVERING when reaches target"""
    target = hover.target
    drone_pos = pos.center
    if distance(drone_pos, target) <= 3.0:
        hover.status = HoverState.HOVERING
        return
    dx = target[0] - drone_pos[0]
    dy = target[1] - drone_pos[1]
    velocity.x = min(dx, max_speed) if dx > 0 else max(dx, -max_speed)
    velocity.y = min(dy, max_speed) if dy > 0 else max(dy, -max_speed)


def landed_action(hover: Hover, pos: Position, velocity: Velocity):
    pass
