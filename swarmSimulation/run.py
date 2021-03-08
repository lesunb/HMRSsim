import sys
import json


import simulator.systems.ScriptEventsDES as ScriptSystem
import simulator.systems.GotoDESProcessor as NavigationSystem
import simulator.systems.SeerPlugin as Seer
from simulator.systems.MovementProcessor import MovementProcessor
from simulator.systems.CollisionProcessor import CollisionProcessor
from simulator.systems.PathProcessor import PathProcessor

import swarmSimulation.systems.HoverDisturbance as HoverDisturbance
import swarmSimulation.systems.HoverSystem as HoverSystem


from simulator.components.Script import Script
from swarmSimulation.components.Hover import Hover, HoverState

from simulator.main import Simulator
from simulator.utils.Firebase import db, clean_old_simulation

# Prep Script and Navigation systems
extra_instructions = [
    (NavigationSystem.GotoInstructionId, NavigationSystem.goInstruction),
]
ScriptProcessor = ScriptSystem.init(extra_instructions, [])
NavigationSystemProcess = NavigationSystem.init()
# Create a simulation with config
simulator = Simulator(sys.argv[1])
# Some simulator objects
width, height = simulator.window_dimensions
fps = simulator.FPS
# window = simulator.window
eventStore = simulator.KWARGS['EVENT_STORE']
exitEvent = simulator.EXIT_EVENT
env = simulator.ENV

# Prep Seer plugin
NAMESPACE = 'simulator'
clean_old_simulation(NAMESPACE)


def firebase_seer_consumer(message, msg_idx):
    """Sends Seer messages to firebase"""
    if msg_idx >= 0:
        if msg_idx == 1:
            for idx, j in enumerate(message):
                db.child(NAMESPACE).child('live_report').child(msg_idx).child(idx).set({j: message[j]})
        else:
            _ = db.child(NAMESPACE).child('live_report').child(msg_idx).set(message)


# Defines and initializes esper.Processor for the simulation
normal_processors = [
    MovementProcessor(minx=0, miny=0, maxx=width, maxy=height),
    CollisionProcessor(),
    PathProcessor()
]
# Defines DES processors
des_processors = [
    Seer.init([firebase_seer_consumer], 0.05, False),
    # (NavigationSystemProcess,),
    # (ScriptProcessor,),
    (HoverDisturbance.init(max_disturbance=0.1, prob_disturbance=0.4, disturbance_interval=(1 / (fps / 3))),),
    (HoverSystem.init(max_fix_speed=0.2, hover_interval=(1 / (fps / 6))),)
]
# Add processors to the simulation, according to processor type
for p in normal_processors:
    simulator.add_system(p)
for p in des_processors:
    simulator.add_des_system(p)


# Create the error handlers dict
# error_handlers = {
#     NavigationSystem.PathErrorTag: NavigationSystem.handle_PathError
# }
# # Adding error handlers to the robot
# robot = simulator.objects[0][0]
# script = simulator.world.component_for_entity(robot, Script)
# script.error_handlers = error_handlers

drone = simulator.objects[0][0]
hover = simulator.world.component_for_entity(drone, Hover)
hover.target = (242.5, 202.5)
hover.status = HoverState.HOVERING

if __name__ == "__main__":
    # NOTE!  schedule_interval will automatically pass a "delta time" argument
    #        to world.process, so you must make sure that your Processor classes
    #        account for this. See the example Processors above.
    simulator.run()
    # print("Robot's script logs")
    # print("\n".join(script.logs))
    # pass
