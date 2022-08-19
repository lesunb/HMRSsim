import esper
import simpy
from typing import List
from simulator.typehints.dict_types import SystemArgs
from simulator.typehints.build_types import WindowOptions
from simulator.typehints.component_types import Point, ShapeDefinition
from typing import NamedTuple
from urdf_parser_py import urdf
from simulator.utils.create_components import initialize_components, import_external_component

RobotSpawnEventTag = 'RobotEntityEvent'


# RobotSpawnPayload = NamedTuple('RobotSpawnEvent', ('robot_definition', string))

def collidable_from_position(pos: Point) -> List[ShapeDefinition]:
    center = (pos[0] + 2.5, pos[1] + 2.5)
    points = [
        (pos[0], pos[1]),
        (pos[0] + 5, pos[1]),
        (pos[0] + 5, pos[1] + 5),
        (pos[0], pos[1] + 5)
    ]
    return [(center, points)]


def init():
    def process(kwargs: SystemArgs):
        event_store = kwargs.get('EVENT_STORE', None)
        world: esper.World = kwargs.get('WORLD', None)
        env: simpy.Environment = kwargs.get('ENV', None)
        draw2ent = kwargs.get('DRAW2ENT', None)
        objects = kwargs.get('OBJECTS', None)
        interactive = kwargs.get('INTERACTIVE', None)
        ent_id = 0

        while True:
            event = yield event_store.get(lambda ev: ev.type == RobotSpawnEventTag)
            if event == None:
                continue

            ent_id += 1
            urdf_xml = event.payload.robot_definition
            robot = urdf.Robot.from_xml_string(urdf_xml)
            pos = robot.joints[0].origin.xyz
            type = 'robot'
            initialized_components = initialize_components(
                {
                    "Position": [pos[0], pos[1], 0, 5, 5],
                    "Collidable": [collidable_from_position((pos[0], pos[1]))],
                    "Skeleton": ['robot_' + str(
                        ent_id) + "rounded=0;whiteSpace=wrap;html=1;strokeColor=#00FF00;fillColor=#000000;"],
                    # "Velocity": [0, 0],
                    # "Script": [["Go exit"], 10]
                })
            ent = world.create_entity(*initialized_components)
            draw2ent[ent_id] = [ent, {'type': type}]
            objects.append((ent, ent_id))

    return process
