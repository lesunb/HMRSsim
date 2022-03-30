from simulator.systems.MovementProcessor import MovementProcessor
from simulator.systems.CollisionProcessor import CollisionProcessor
from simulator.systems.PathProcessor import PathProcessor

import simulator.systems.EnergyConsumptionDESProcessor as energySystem
import simulator.systems.ManageObjects as ObjectManager
import simulator.systems.ScriptEventsDES as ScriptSystem
import simulator.systems.GotoDESProcessor as NavigationSystem
import simulator.systems.SeerPlugin as Seer
from simulator.systems.MoveBaseSystem import MoveBaseSystem
from simulator.systems.RosControlSystem import RosControlProcessor

from simulator.components.Script import Script

from simulator.main import Simulator

from simulator.utils.Firebase import Firebase_conn

import logging

def main():
    logging.basicConfig(level=logging.DEBUG)
    # Create a simulation with config
    simulator = Simulator('/home/kalley/Workspace/unb/tg/HMRSsim/examples/navigationSimulationRos/simulation.json')
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

    extra_instructions = [
        (NavigationSystem.GotoInstructionId, NavigationSystem.goInstruction)
    ]
    NavigationSystemProcess = NavigationSystem.init()
    ros_control_processor = RosControlProcessor(scan_interval=0.1)
    move_base_service = MoveBaseSystem(event_store=eventStore, exit_event=exitEvent, world=world)
    ros_control_processor.create_subscription(move_base_service)

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
        (ros_control_processor.process, ros_control_processor.end)
    ]
    # Add processors to the simulation, according to processor type
    for p in normal_processors:
        simulator.add_system(p)
    for p in des_processors:
        simulator.add_des_system(p)


    # Create the error handlers dict
    error_handlers = {
        NavigationSystem.PathErrorTag: NavigationSystem.handle_PathError
    }

    simulator.run()

if __name__ == '__main__':
    main()