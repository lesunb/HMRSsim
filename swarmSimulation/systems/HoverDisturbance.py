import logging
import random

from simulator.typehints.dict_types import SystemArgs

from simulator.components.Velocity import Velocity
from swarmSimulation.components.Hover import Hover, HoverState


def init(disturbance_interval=1, prob_disturbance=0.2, max_disturbance=0.5):
    def process(kwargs: SystemArgs):
        logger = logging.getLogger(__name__)
        world = kwargs.get('WORLD', None)
        env = kwargs.get('ENV', None)
        if env is None:
            raise Exception("Can't find eventStore")
        while True:
            for ent, (hover, velocity) in world.get_components(Hover, Velocity):
                if hover.status != HoverState.HOVERING:
                    continue
                if random.random() < prob_disturbance:
                    disturbance = random.random() % max_disturbance
                    flip = random.random() < 0.5
                    if random.random() < 0.5:
                        velocity.x += (disturbance if not flip else -disturbance)
                    else:
                        velocity.y += (disturbance if not flip else -disturbance)
            yield env.timeout(disturbance_interval)
    return process
