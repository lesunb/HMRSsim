import sys
import json

from systems.MovementProcessor import MovementProcessor
from systems.CollisionProcessor import CollisionProcessor
from systems.PathProcessor import PathProcessor

import systems.GotoDESProcessor as gotoProcessor
import systems.NavigationSystem as mapProcessor
import systems.StopCollisionDESProcessor as StopCollision
import systems.EnergyConsumptionDESProcessor as energySystem
import systems.ManageObjects as ObjectManager
import systems.ClawDESProcessor as ClawProcessor
from systems.ScriptEventsDES import init
from main import Simulator, EVENT

import systems.SeerPlugin as Seer

extra_instructions = [
    (gotoProcessor.GotoInstructionId, gotoProcessor.goInstruction),
    (ClawProcessor.GrabInstructionTag, ClawProcessor.grabInstruction),
    (ClawProcessor.DropInstructionTag, ClawProcessor.dropInstrution)
]
ScriptProcessor = init(extra_instructions, [ClawProcessor.ClawDoneTag])

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


def my_seer_consumer(message):
    fd.write(json.dumps(message) + '\n')


# Defines and initializes esper.Processor for the simulation
normal_processors = [
    MovementProcessor(minx=0, miny=0, maxx=width, maxy=height),
    CollisionProcessor(),
    PathProcessor()
]
# Defines DES processors
des_processors = [
    Seer.init([my_seer_consumer], 0.05, False),
    (ClawProcessor.process,),
    (ObjectManager.process,),
    (StopCollision.process,),
    (gotoProcessor.process,),
    (energySystem.process,),
    # (ScriptProcessor,),
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


if __name__ == "__main__":
    # NOTE!  schedule_interval will automatically pass a "delta time" argument
    #        to world.process, so you must make sure that your Processor classes
    #        account for this. See the example Processors above.
    simulator.run()
