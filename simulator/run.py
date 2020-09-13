import pyglet
import os
import sys
import json
import simpy
import pathlib
import esper

from typing import NamedTuple
# Include current directory in path
sys.path.append(os.getcwd())

import map_parser

from components.Map import Map

from systems.MovementProcessor import MovementProcessor
from systems.CollisionProcessor import CollisionProcessor
from systems.RenderProcessor import RenderProcessor
from systems.PathProcessor import PathProcessor

import systems.GotoProcessor as gotoProcessor
import systems.MapProcessor as mapProcessor


EVENT = NamedTuple('Event', [('type', str), ('payload', object)])
FPS = 60
DEFAULT_LINE_WIDTH = 10
CONFIG = 'simulation.json' if len(sys.argv) == 1 else sys.argv[1]

with open(CONFIG) as fd:
    config = json.load(fd)
FILE = pathlib.Path(config.get('context', '.')) / config.get('map', 'map.drawio')
SIMULATION_DURATION = config.get('duration', -1)

simulation = map_parser.build_simulation_from_map(FILE)
world: esper.World = simulation['world']
window = simulation['window']
batch = simulation['batch']
(window_name, (WIDTH, HEIGHT), BKGD) = simulation['window_props']
draw2ent = simulation['draw_map']
objects = simulation['objects']

print(f"==> Simulation objects")
for id, objId in objects:
    entity = draw2ent.get(objId)
    print(f"OBJ #{id} (draw {objId}). Type {entity[1]['type']}")
    # print(f"Object has components {world.components_for_entity(id)}")
    if world.has_component(id, Map):
        ent_map = world.component_for_entity(id, Map)
        print("\tAvailable paths:")
        for idx, key in enumerate(ent_map.paths.keys()):
            print(f"\t{idx}. {key}")
robot1 = objects[0] if len(objects) > 0 else None

# Create some Processor instances, and assign them to the World to be processed:
movement_processor = MovementProcessor(
    minx=0, miny=0, maxx=WIDTH, maxy=HEIGHT)
world.add_processor(movement_processor)

collision_processor = CollisionProcessor()
world.add_processor(collision_processor)

render_processor = RenderProcessor()
world.add_processor(render_processor)

path_processor = PathProcessor()
world.add_processor(path_processor)

################################################
#  Set up pyglet events for input and rendering:
################################################

KEYS = pyglet.window.key
GOTO = False
MAP = False
buff = []

@window.event
def on_text(text):
    if GOTO and text != 'G':
        buff.append(text)
    elif MAP:
        buff.append(text)


@window.event
def on_key_press(key, mod):
    global GOTO
    global MAP
    if not GOTO and mod & KEYS.MOD_SHIFT and key == KEYS.G:
        print(f"Goto key pressed. Usage: ^G ent-poi ENTER")
        GOTO = True
    if not MAP and mod & KEYS.MOD_SHIFT and key == KEYS.M:
        print(f"Map key pressed. Usage: ^M ent key")
        MAP = True
    if key == KEYS.ENTER or key == KEYS.RETURN:
        if GOTO:
            ent, poi = "".join(buff).split('-')
            print(f"End of Goto sequence. Taking entity {ent} to point {poi}")
            payload = gotoProcessor.GOTO_PAYLOAD(int(ent), int(poi))
            new_event = EVENT(gotoProcessor.GOTO_EVENT_TAG, payload)
            eventStore.put(new_event)
        elif MAP:
            ent, key = "".join(buff[1:]).split(' ')
            print(f"End of Map sequence. Taking entity {ent} to path {key}")
            payload = mapProcessor.MAP_PAYLOAD(int(ent), key)
            new_event = EVENT(mapProcessor.MAP_EVENT_TAG, payload)
            eventStore.put(new_event)
        buff.clear()
        GOTO = False
        MAP = False

@window.event
def on_draw():
    # Clear the window to background color
    window.clear()
    # Draw the batch of Renderables:
    batch.draw()


@window.event
def on_close():
    global EXIT
    EXIT = True


####################################################
#  Schedule a World update and start the pyglet app:
####################################################

def simulation_loop(pass_switch_ref):
    kwargs = {
        "ENV": env,
        "WORLD": world,
        "_KILLSWITCH": (killswitch if pass_switch_ref else None),
        "EVENT_STORE": eventStore
    }
    # Discrete processors
    env.process(gotoProcessor.process(kwargs))
    env.process(mapProcessor.process(kwargs))
    # Other processors
    while not EXIT:
        pyglet.clock.tick()
        world.process(kwargs)
        # For many windows
        for w in pyglet.app.windows:
            w.switch_to()
            w.dispatch_events()
            w.dispatch_event('on_draw')
            w.flip()
        # ticks on the clock
        switch = yield killswitch | env.timeout(1.0 / FPS, False)
        if killswitch in switch:
            break


EXIT = False
if __name__ == "__main__":
    # NOTE!  schedule_interval will automatically pass a "delta time" argument
    #        to world.process, so you must make sure that your Processor classes
    #        account for this. See the example Processors above.
    env = simpy.Environment()
    eventStore = simpy.FilterStore(env, capacity=10)
    killswitch = env.event()
    if SIMULATION_DURATION > 0:
        env.process(simulation_loop(False))
        env.run(until=SIMULATION_DURATION)
    else:
        env.process(simulation_loop(True))
        env.run()
