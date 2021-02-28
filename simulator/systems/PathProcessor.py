import esper
import logging
from typing import NamedTuple
from typehints.dict_types import SystemArgs

from simpy import FilterStore

from components.Path import Path
from components.Position import Position
from components.Velocity import Velocity
from typing import List
from typehints.component_types import Point, EVENT


EndOfPathPayload = NamedTuple('EndOfPathPayload', [('ent', int), ('timestamp', str), ('path', List[Point])])
EndOfPathTag = 'EndOfPath'


class PathProcessor(esper.Processor):
    def __init__(self):
        super().__init__()
        self.logger = logging.getLogger(__name__)

    def process(self, kwargs: SystemArgs):
        event_store: FilterStore = kwargs.get('EVENT_STORE', None)
        env = kwargs.get('ENV', None)
        for ent, (pos, path, vel) in self.world.get_components(Position, Path, Velocity):
            # self.logger.debug(f"[{env.now}] Processing {ent}")
            point = path.points[path.curr_point]
            pos_center = pos.center
            if pos_center[0] == point[0] and pos_center[1] == point[1]:
                self.logger.debug(f"[{env.now}] Entity {ent} moved point in path path - {point}[idx={path.curr_point}] - {pos.center} - {vel}")
                path.curr_point += 1
                if path.curr_point == len(path.points):
                    # end of path
                    vel.x = 0
                    vel.y = 0
                    self.logger.debug(
                        f"Removed Path component from {ent} (pos={pos.center}). Last point of path is {path.points[-1]}"
                    )
                    # Adds an EndOfPath event, in case anyone is listening
                    end_of_path = EVENT(EndOfPathTag, EndOfPathPayload(ent, str(env.now), path=path.points))
                    # self.logger.debug(f'[{env.now}] PathProcessor adding EndOfPath event {end_of_path}')
                    event_store.put(end_of_path)
                    pos.changed = False
                    self.world.remove_component(ent, Path)
                    return
            else:
                dx = point[0] - pos_center[0]
                if dx > 0:
                    vel.x = min(path.speed, dx)
                else:
                    vel.x = max(- path.speed, dx)
                dy = point[1] - pos_center[1]
                if dy > 0:
                    vel.y = min(path.speed, dy)
                else:
                    vel.y = max(- path.speed, dy)
