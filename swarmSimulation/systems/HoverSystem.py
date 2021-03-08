import logging
import random

from simulator.typehints.dict_types import SystemArgs

from components.Velocity import Velocity
from components.Position import Position
from swarmSimulation.components.Hover import Hover, HoverState


def init(hover_interval=1, max_fix_speed=1):
    def process(kwargs: SystemArgs):
        logger = logging.getLogger(__name__)
        world = kwargs.get('WORLD', None)
        env = kwargs.get('ENV', None)
        if env is None:
            raise Exception("Can't find eventStore")
        while True:
            for ent, (hover, pos, velocity) in world.get_components(Hover, Position, Velocity):
                if hover.status != HoverState.HOVERING:
                    continue
                target = hover.target
                drone_pos = pos.center
                if target == drone_pos:
                    continue
                dx = target[0] - drone_pos[0]
                dy = target[1] - drone_pos[1]
                velocity.x = min(dx, max_fix_speed) if dx > 0 else max(dx, -max_fix_speed)
                velocity.y = min(dy, max_fix_speed) if dy > 0 else max(dy, -max_fix_speed)
            yield env.timeout(hover_interval)
    return process
