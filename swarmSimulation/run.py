import sys
import simpy
import logging
import math

import simulator.systems.ScriptEventsDES as ScriptSystem
import simulator.systems.GotoDESProcessor as NavigationSystem
import simulator.systems.SeerPlugin as Seer
import simulator.systems.SensorSystem as SensorSystem

from simulator.systems.MovementProcessor import MovementProcessor
from simulator.systems.CollisionProcessor import CollisionProcessor
from simulator.systems.PathProcessor import PathProcessor


import swarmSimulation.systems.HoverDisturbance as HoverDisturbance
import swarmSimulation.systems.HoverSystem as HoverSystem
from swarmSimulation.systems.Control import control

from simulator.components.ProximitySensor import ProximitySensor
from swarmSimulation.components.Hover import Hover

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
    (SensorSystem.init(ProximitySensor, 1),),
    # (NavigationSystemProcess,),
    # (ScriptProcessor,),
    (HoverDisturbance.init(max_disturbance=0.1, prob_disturbance=0.4, disturbance_interval=(1 / (fps / 3))),),
    (HoverSystem.init(max_fix_speed=0.2, hover_interval=(1 / (fps / 3)), max_speed=1),)
]
# Add processors to the simulation, according to processor type
for p in normal_processors:
    simulator.add_system(p)
for p in des_processors:
    simulator.add_des_system(p)


for drone, _ in simulator.objects:
    hover = simulator.world.component_for_entity(drone, Hover)
    sensor: ProximitySensor = simulator.world.component_for_entity(drone, ProximitySensor)
    sensor.reply_channel = simpy.Store(env)


def capture(sensor: ProximitySensor):
    while True:
        ev = yield sensor.reply_channel.get()
        payload = ev.payload
        me = payload.ent
        they = payload.close_entities
        # TODO: if (GoingToCrashIntoEachOther()) dont();
        # print(f'Drone {me} close to {they}')
        # mypos = payload.pos
        # myvel = payload.vel
        # other_pos = payload.other_pos
        # dx = math.fabs(other_pos.x - mypos.x)
        # dy = math.fabs(other_pos.y - mypos.y)
        # ndx = math.fabs(other_pos.x - (mypos.x - myvel.x))
        # ndy = math.fabs(other_pos.y - (mypos.y - myvel.y))
        # print(f'{me} {they}. [{dx}, {ndx}]. [{dy}, {ndy}].')
        # if math.fabs(ndx - dx) <= 2:
        #     myvel.x = 0
        # if math.fabs(ndy - dy) <= 2:
        #     myvel.y = 0


if __name__ == "__main__":
    # env.process(control(simulator.KWARGS['_KILL_SWITCH']))
    for drone, _ in simulator.objects:
        sensor: ProximitySensor = simulator.world.component_for_entity(drone, ProximitySensor)
        env.process(capture(sensor))
    env.process(control(simulator.world, env, simulator.KWARGS['_KILL_SWITCH']))
    simulator.run()

