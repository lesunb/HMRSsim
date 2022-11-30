from simulator.systems.MovementProcessor import MovementProcessor
from simulator.systems.CollisionProcessor import CollisionProcessor
from simulator.systems.PathProcessor import PathProcessor
import simulator.systems.GotoDESProcessor as NavigationSystem
import simulator.systems.ClawDESProcessor as ClawProcessor
import simulator.systems.ManageObjects as ObjectManager
import simulator.systems.RobotSpawnDESProcessor as RobotSpawnDESProcessor
import simulator.systems.SeerPlugin as Seer
from simulator.systems.Nav2System import Nav2System
from simulator.systems.RosControlPlugin import RosControlPlugin

from simulator.main import Simulator
from simulator.utils.ROS2 import ROS2_conn

from simulator.typehints.component_types import EVENT

import rclpy
import logging
import sys


def main():
    rclpy.init()
    logger = logging.getLogger(__name__)
    logging.basicConfig()
    logging.root.setLevel(logging.DEBUG)

    # Create a simulation with config
    if len(sys.argv) <= 1:
        logger.error("You have to specify a path to a simulation file")
        exit()
    simulator = Simulator(sys.argv[1])
    # Some simulator objects
    width, height = simulator.window_dimensions
    eventStore = simulator.KWARGS['EVENT_STORE']
    world = simulator.KWARGS['WORLD']
    exitEvent = simulator.EXIT_EVENT
    env = simulator.ENV

    NAMESPACE = 'navigation_ros'

    ros2 = ROS2_conn()
    NavigationSystemProcess = NavigationSystem.init()
    ros_control = RosControlPlugin(scan_interval=0.1)
    claw_services = ClawProcessor.create_grab_and_drop_for_each_robot(world=world, event_store=eventStore)
    for service in claw_services:
        ros_control.create_action_server(service)
    nav2_services = Nav2System.create_services(event_store=eventStore, world=world)
    for service in nav2_services:
        ros_control.create_action_server(service)

    # Defines and initializes esper.Processor for the simulation
    normal_processors = [
        MovementProcessor(minx=0, miny=0, maxx=width, maxy=height),
        CollisionProcessor(),
        PathProcessor()
    ]

    # Defines DES processors
    des_processors = [
        (NavigationSystemProcess,),
        (ClawProcessor.process,),
        (ObjectManager.process,),
        (ros_control.process, ros_control.end),
        (Nav2System.end_path_event_listener,),
        Seer.init([ros2.seer_consumer], 0.25, False)
    ]

    # Add processors to the simulation, according to processor type
    for p in normal_processors:
        simulator.add_system(p)
    for p in des_processors:
        simulator.add_des_system(p)

    simulator.run()

if __name__ == '__main__':
    main()
