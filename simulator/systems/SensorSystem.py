import logging

from typehints.dict_types import SystemArgs
from typehints.component_types import EVENT
from typing import NamedTuple, List
from simulator.components.Position import Position
from simulator.components.Velocity import Velocity
from simulator.components.Collidable import Collidable

from collision import collide

CloseEntity = NamedTuple(
    'CloseEntity',
    [('other_ent', int), ('other_pos', Position)]
)

SensorPayload = NamedTuple(
    'SensorPayload',
    [('ent', int), ('pos', Position), ('vel', Velocity), ('close_entities', List[CloseEntity])]
)


def init(sensor_type, frequency):
    def process(kwargs: SystemArgs):
        logger = logging.getLogger(__name__)
        env = kwargs.get('ENV', None)
        world = kwargs.get('WORLD', None)
        if env is None:
            raise Exception("Can't find env")
        # Local ref most used variables
        get_components = world.get_components
        sleep = env.timeout
        while True:
            for ent, (pos, vel, sensor) in get_components(Position, Velocity, sensor_type):
                # logger.debug(f'Analysing ent {ent}')
                points = [
                    (pos.center[0] - sensor.range, pos.center[1] - sensor.range),
                    (pos.center[0] + sensor.range, pos.center[1] - sensor.range),
                    (pos.center[0] + sensor.range, pos.center[1] + sensor.range),
                    (pos.center[0] - sensor.range, pos.center[1] + sensor.range)
                ]
                col = Collidable([(pos.center, points)])
                closeEntities = []
                for otherEnt, (otherCol, otherPos) in get_components(Collidable, Position):
                    if ent == otherEnt:
                        continue
                    for s1 in otherCol.shapes:
                        if collide(col.shapes[0], s1):
                            closeEntities.append(CloseEntity(otherEnt, otherPos))
                            break
                if closeEntities:
                    event = EVENT('SensorEvent', SensorPayload(ent, pos, vel, closeEntities))
                    sensor.reply_channel.put(event)
            yield sleep(frequency)

    return process
