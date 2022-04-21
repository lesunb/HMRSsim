from simulator.systems.MovementProcessor import MovementProcessor
from simulator.systems.CollisionProcessor import CollisionProcessor
from simulator.systems.PathProcessor import PathProcessor

import simulator.systems.GotoDESProcessor as NavigationSystem
import simulator.systems.SeerPlugin as Seer
from simulator.systems.RosNavigationSystem import RosNavigationSystem
from simulator.systems.RosControlPlugin import RosControlPlugin

from simulator.main import Simulator

from simulator.utils.Firebase import Firebase_conn

import logging

import sys

def main():
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
    firebase = Firebase_conn(NAMESPACE)
    firebase.clean_old_simulation()
    build_report = simulator.build_report
    firebase.send_build_report(build_report)

    NavigationSystemProcess = NavigationSystem.init()
    ros_control = RosControlPlugin(scan_interval=0.1)
    move_base_service = RosNavigationSystem(event_store=eventStore, exit_event=exitEvent, world=world)
    ros_control.create_action_server(move_base_service)

    # Defines and initializes esper.Processor for the simulation
    normal_processors = [
        MovementProcessor(minx=0, miny=0, maxx=width, maxy=height),
        CollisionProcessor(),
        PathProcessor()
    ]
    # Defines DES processors
    des_processors = [
        Seer.init([firebase.seer_consumer], 0.05, False),
        (NavigationSystemProcess,),
        (ros_control.process, ros_control.end)
    ]
    # Add processors to the simulation, according to processor type
    for p in normal_processors:
        simulator.add_system(p)
    for p in des_processors:
        simulator.add_des_system(p)

    simulator.run()

if __name__ == '__main__':
    main()
