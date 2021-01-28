from simulator.systems.MovementProcessor import MovementProcessor
from simulator.systems.CollisionProcessor import CollisionProcessor
from simulator.systems.PathProcessor import PathProcessor

import simulator.systems.GotoDESProcessor as gotoProcessor
import simulator.systems.MapDESProcessor as mapProcessor
import simulator.systems.StopCollisionDESProcessor as StopCollision
import simulator.systems.EnergyConsumptionDESProcessor as energySystem
import simulator.systems.ManageObjects as ObjectManager
import simulator.systems.ClawDESProcessor as ClawProcessor
from simulator.systems.ScriptEventsDES import init
import simulator.systems.SeerPlugin as Seer

from simulator.main import Simulator
import json

# File to output the report
fd = open('report.json', 'w')

def my_seer_consumer(message):
    fd.write(json.dumps(message) + '\n')

# Clean up function to be executed after the simulation exists
def clean():
    fd.close()
    print("Closed fd.")

def CreateSimulation(file):
    simulator = Simulator(file, clean)

    extra_instructions = [
        (gotoProcessor.GotoInstructionId, gotoProcessor.goInstruction),
        (mapProcessor.MapInstructionId, mapProcessor.mapInstruction),
        (ClawProcessor.GrabInstructionTag, ClawProcessor.grabInstruction),
        (ClawProcessor.DropInstructionTag, ClawProcessor.dropInstrution)
    ]

    ScriptProcessor = init(extra_instructions, [ClawProcessor.ClawDoneTag])

    # Some simulator objects
    width, height = simulator.window_dimensions
    eventStore = simulator.KWARGS['EVENT_STORE']
    exitEvent = simulator.EXIT_EVENT
    env = simulator.ENV
    
    # Defines and initializes esper.Processor for the simulation
    normal_processors = [
        MovementProcessor(minx=0, miny=0, maxx=width, maxy=height),
        CollisionProcessor(),
        PathProcessor()
    ]

    # Defines DES processors
    des_processors = [
        Seer.init([my_seer_consumer], 0.5, True),
        (ClawProcessor.process,),
        (ObjectManager.process,),
        (StopCollision.process,),
        (gotoProcessor.process,),
        (mapProcessor.process,),
        (energySystem.process,),
        (ScriptProcessor,),
    ]

    for p in normal_processors:
        simulator.add_system(p)
    for p in des_processors:
        simulator.add_DES_system(p)

    return simulator
