import logging

from datetime import datetime, timedelta
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
        total = timedelta()
        runs = 0
        while True:
            start = datetime.now()
            for ent, (pos, vel, sensor) in get_components(Position, Velocity, sensor_type):
                # logger.debug(f'Analysing ent {ent}')
                center_x, center_y = pos.center
                sensor_range = sensor.range
                points = [
                    (center_x - sensor_range, center_y - sensor_range),
                    (center_x + sensor_range, center_y - sensor_range),
                    (center_x + sensor_range, center_y + sensor_range),
                    (center_x - sensor_range, center_y + sensor_range)
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
            end = datetime.now()
            runs += 1
            total += end - start
            if runs % 50 == 0:
                logger.debug(f'runs: {runs}; total: {total}; avg = {total / runs}')

    return process
