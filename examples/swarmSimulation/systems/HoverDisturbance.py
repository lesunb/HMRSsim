import logging
import random

from simulator.typehints.dict_types import SystemArgs

from simulator.components.Velocity import Velocity
from components.Hover import Hover, HoverState


def init(disturbance_interval=1, prob_disturbance=0.2, max_disturbance=0.5):
    def process(kwargs: SystemArgs):
        logger = logging.getLogger(__name__)
        world = kwargs.get('WORLD', None)
        env = kwargs.get('ENV', None)
        if env is None:
            raise Exception("Can't find eventStore")
        # Local ref most used functions for performance
        get_components = world.get_components
        sleep = env.timeout
        prob = random.random
        #
        while True:
            for ent, (hover, velocity) in get_components(Hover, Velocity):
                if hover.status != HoverState.HOVERING:
                    continue
                if prob() < prob_disturbance:
                    disturbance = prob() % max_disturbance
                    flip = prob() < 0.5
                    if prob() < 0.5:
                        velocity.x += (disturbance if not flip else -disturbance)
                    else:
                        velocity.y += (disturbance if not flip else -disturbance)
            yield sleep(disturbance_interval)
    return process
