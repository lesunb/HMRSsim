from simulator.systems.MovementProcessor import MovementProcessor
from simulator.systems.CollisionProcessor import CollisionProcessor
from simulator.systems.PathProcessor import PathProcessor

import simulator.systems.EnergyConsumptionDESProcessor as energySystem
import simulator.systems.ManageObjects as ObjectManager
import simulator.systems.ScriptEventsDES as ScriptSystem
import simulator.systems.GotoDESProcessor as NavigationSystem
import simulator.systems.SeerPlugin as Seer
from simulator.systems.MoveBaseSystem import MoveBaseObserver
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
    # window = simulator.window
    eventStore = simulator.KWARGS['EVENT_STORE']
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
    ScriptProcessor = ScriptSystem.init(extra_instructions, [])
    NavigationSystemProcess = NavigationSystem.init()
    # movebase_processor = MoveBaseProcessor(exit_event=exitEvent)
    ros_control_processor = RosControlProcessor()
    ros_control_processor.add_observer(MoveBaseObserver())

    # Defines and initializes esper.Processor for the simulation
    normal_processors = [
        MovementProcessor(minx=0, miny=0, maxx=width, maxy=height),
        CollisionProcessor(),
        PathProcessor(),
        ros_control_processor
    ]
    # Defines DES processors
    des_processors = [
        Seer.init([firebase.seer_consumer], 0.05, False),
        (NavigationSystemProcess,)
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
    # Adding error handlers to the robot

    robot = simulator.objects[0][0]
    script = simulator.world.component_for_entity(robot, Script)
    script.error_handlers = error_handlers

    simulator.run()
    print("Robot's script logs")
    print("\n".join(script.logs))
    ros_control_processor.end()

if __name__ == '__main__':
    main()