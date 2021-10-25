import simpy

import simulator.systems.ScriptEventsDES as ScriptSystem
import simulator.systems.GotoDESProcessor as NavigationSystem
import simulator.systems.SeerPlugin as Seer
import simulator.systems.ClockSystem as ClockSystem

from simulator.systems.MovementProcessor import MovementProcessor
from simulator.systems.CollisionProcessor import CollisionProcessor

import systems.HoverDisturbance as HoverDisturbance
import systems.HoverSystem as HoverSystem
from systems.ControlSystem import control
from components.Control import Control

from simulator.main import Simulator
from simulator.utils.Firebase import Firebase_conn

from generate_simulation_json import generate_simulation_json, generate_shapes

DRONE_COUNT = 48
# Prep Script and Navigation systems
extra_instructions = [
    (NavigationSystem.GotoInstructionId, NavigationSystem.goInstruction),
]
ScriptProcessor = ScriptSystem.init(extra_instructions, [])
NavigationSystemProcess = NavigationSystem.init()
# Create a simulation with config
simulator = Simulator(generate_simulation_json(DRONE_COUNT))
# Some simulator objects
width, height = simulator.window_dimensions
fps = simulator.FPS
# window = simulator.window
eventStore = simulator.KWARGS['EVENT_STORE']
exitEvent = simulator.EXIT_EVENT
env = simulator.ENV

# Prep Seer plugin
NAMESPACE = 'simulator'
firebase = Firebase_conn(NAMESPACE)
firebase.clean_old_simulation()
build_report = simulator.build_report
firebase.send_build_report(build_report)

# Defines and initializes esper.Processor for the simulation
normal_processors = [
    MovementProcessor(minx=0, miny=0, maxx=width, maxy=height, sector_size=20),
    CollisionProcessor(),
]
# Defines DES processors
des_processors = [
    Seer.init([firebase.seer_consumer], 0.1, simulator.verbose),
    (HoverDisturbance.init(max_disturbance=0.1, prob_disturbance=0.4, disturbance_interval=(1 / (fps / 3))),),
    (HoverSystem.init(max_fix_speed=0.2, hover_interval=(1.0 / fps), max_speed=2.5),),
    (ClockSystem.process, ClockSystem.clean)
]
# Add processors to the simulation, according to processor type
for p in normal_processors:
    simulator.add_system(p)
for p in des_processors:
    simulator.add_des_system(p)

control_component = Control(configs=generate_shapes(DRONE_COUNT), channel=simpy.Store(env))
simulator.world.add_component(1, control_component)

if __name__ == "__main__":
    env.process(control(simulator.KWARGS))
    simulator.run()

