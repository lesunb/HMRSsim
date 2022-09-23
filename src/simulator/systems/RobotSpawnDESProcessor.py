import esper
import simpy
import logging
from typing import List
from simulator.typehints.dict_types import SystemArgs
from simulator.typehints.build_types import WindowOptions
from simulator.typehints.component_types import Point, ShapeDefinition
from typing import NamedTuple
from urdf_parser_py import urdf
from simulator.utils.create_components import initialize_components, import_external_component

from simulator.components.Position import Position
from simulator.components.Velocity import Velocity
from simulator.components.NavToPoseRosGoal import NavToPoseRosGoal
from simulator.typehints.ros_types import RosTopicServer
from simulator.typehints.component_types import EVENT
from simulator.systems.Nav2System import Nav2System

RobotSpawnEventTag = 'RobotEntityEvent'
RobotSpawnPayload = NamedTuple('RobotSpawnEvent', [('robot_definition', str)])

def collidable_from_position(pos: Point) -> List[ShapeDefinition]:
    center = (pos[0] + 2.5, pos[1] + 2.5)
    points = [
        (pos[0], pos[1]),
        (pos[0] + 5, pos[1]),
        (pos[0] + 5, pos[1] + 5),
        (pos[0], pos[1] + 5)
    ]
    return [(center, points)]


def init(ros_control=None):
    ros_control = ros_control
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

            # Verificando se o robô já existe
            there_is_already_a_robot = False
            for ent, (vel, pos, ros_goal) in world.get_components(Velocity, Position, NavToPoseRosGoal):
                if ros_goal != None and ros_goal.name == robot.name:
                    logging.getLogger(__name__).warn('There is already a robot with this name')
                    there_is_already_a_robot = True
                    break
            if there_is_already_a_robot:
                continue

            pos = robot.joints[0].origin.xyz
            type = 'robot'
            # TODO Arrumar o tamanho do robô
            initialized_components = initialize_components(
                {
                    "Position": [pos[0], pos[1], 0, 5, 5],
                    "Collidable": [collidable_from_position((pos[0], pos[1]))],
                    "NavToPoseRosGoal": [robot.name],
                    "Skeleton": ['robot_' + str(
                        ent_id), "rounded=0;whiteSpace=wrap;html=1;strokeColor=#00FF00;fillColor=#000000;width=50;height=50;"],
                    "Velocity": [0, 0]
                    # "Script": [["Go exit"], 10]
                })
            ent = world.create_entity(*initialized_components)
            draw2ent[ent_id] = [ent, {'type': type}]
            objects.append((ent, ent_id))

            # TODO Desacoplar o Nav2System daqui
            ros_services = Nav2System.create_services(event_store=event_store, world=world)
            for service in ros_services:
                if any((hasattr(s, "robot_name") and s.robot_name == service.robot_name) for s in ros_control.services):
                    continue
                ros_control.create_action_server(service)
    
    return process


class RobotSpawnerRos(RosTopicServer):

    def __init__(self, **kwargs):
        super().__init__()
        self.logger = logging.getLogger(__name__)
        self.event_store = kwargs.get('event_store', None)

    def get_name(self):
        return 'spawn_robot'

    def listener_callback(self, msg):
        self.logger.info('Received order to spawn a robot')
        event = EVENT(RobotSpawnEventTag, RobotSpawnPayload(msg.data))
        self.event_store.put(event)

    def get_listener_callback(self):
        return self.listener_callback
