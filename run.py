import sys
import json

from systems.MovementProcessor import MovementProcessor
from systems.CollisionProcessor import CollisionProcessor
from systems.PathProcessor import PathProcessor


import systems.EnergyConsumptionDESProcessor as energySystem
import systems.ManageObjects as ObjectManager
import systems.ClawDESProcessor as ClawProcessor
import systems.ScriptEventsDES as ScriptSystem
import systems.GotoDESProcessor as NavigationSystem

from components.Script import Script

from utils.Firebase import db, NAMESPACE

from main import Simulator
import systems.SeerPlugin as Seer

extra_instructions = [
    (NavigationSystem.GotoInstructionId, NavigationSystem.goInstruction),
    (ClawProcessor.GrabInstructionTag, ClawProcessor.grabInstruction),
    (ClawProcessor.DropInstructionTag, ClawProcessor.dropInstrution)
]
ScriptProcessor = ScriptSystem.init(extra_instructions, [ClawProcessor.ClawDoneTag])
NavigationSystemProcess = NavigationSystem.init()

# File to output the report
fd = open('report.json', 'w')


# Clean up function to be executed after the simulation exists
def clean():
    fd.close()
    print("Closed fd.")


# Create a simulation with config
simulator = Simulator(sys.argv[1], clean)
# Some simulator objects
width, height = simulator.window_dimensions
# window = simulator.window
eventStore = simulator.KWARGS['EVENT_STORE']
exitEvent = simulator.EXIT_EVENT
env = simulator.ENV


def my_seer_consumer(message, _):
    """Saves Seer messages in a file"""
    fd.write(json.dumps(message) + '\n')


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
    (ClawProcessor.process,),
    (ObjectManager.process,),
    (energySystem.process,),
    (NavigationSystemProcess,),
    (ScriptProcessor,),
]
# Add processors to the simulation, according to processor type
for p in normal_processors:
    simulator.add_system(p)
for p in des_processors:
    simulator.add_des_system(p)


# @window.event
# def on_draw():
#     # Clear the window to background color
#     window.clear()
#     # Draw the batch of Renderables:
#     simulator.batch.draw()
#
#
# @window.event
# def on_close():
#     global EXIT
#     print(f'Exiting from window')
#     EXIT = False
#     exitEvent.succeed()

# Create the error handlers dict
error_handlers = {
    NavigationSystem.PathErrorTag: NavigationSystem.handle_PathError
}
# Adding error handlers to the robot
robot = simulator.objects[0][0]
script = simulator.world.component_for_entity(robot, Script)
script.error_handlers = error_handlers

if __name__ == "__main__":
    # NOTE!  schedule_interval will automatically pass a "delta time" argument
    #        to world.process, so you must make sure that your Processor classes
    #        account for this. See the example Processors above.
    simulator.run()
    print("Robot's script logs")
    print("\n".join(script.logs))
