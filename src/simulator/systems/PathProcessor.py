import esper
import logging
from typing import NamedTuple
from simulator.typehints.dict_types import SystemArgs

from simpy import FilterStore

from simulator.components.Path import Path
from simulator.components.Position import Position
from simulator.components.Velocity import Velocity
from simulator.components.ApproximationHistory import ApproximationHistory
from typing import List
from simulator.typehints.component_types import Point, EVENT


EndOfPathPayload = NamedTuple('EndOfPathPayload', [('ent', int), ('timestamp', str), ('path', List[Point])])
EndOfPathTag = 'EndOfPath'

EndOfApproximationPayload = NamedTuple('EndOfApproximation', [('ent', int), ('timestamp', str),])
EndOfApproximationTag = 'EndOfApproximation'

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

                    if self.world.has_component(end_of_path.payload.ent, ApproximationHistory):
                        history = self.world.component_for_entity(end_of_path.payload.ent, ApproximationHistory)
                        if not history.approximated:
                            self.send_end_of_approximation_event(end_of_path.payload.ent, event_store, str(env.now))

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

    def send_end_of_approximation_event(self, ent, event_store, now):
        entity_pos = self.world.component_for_entity(ent, Position).center
        history = self.world.component_for_entity(ent, ApproximationHistory)
        history.entity_final_approx_pos = (entity_pos[0], entity_pos[1])
        history.approximated = True

        end_of_approximation_event = EVENT(EndOfApproximationTag, EndOfApproximationPayload(ent, now))
        event_store.put(end_of_approximation_event)