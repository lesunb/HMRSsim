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
from components.Renderable import Renderable

from systems.MovementProcessor import MovementProcessor
from systems.CollisionProcessor import CollisionProcessor
from systems.RenderProcessor import RenderProcessor
from systems.PathProcessor import PathProcessor

import systems.GotoDESProcessor as gotoProcessor
import systems.MapDESProcessor as mapProcessor
import systems.StopCollisionDESProcessor as StopCollision
import systems.EnergyConsumptionDESProcessor as energySystem
import systems.ManageObjects as ObjectManager
from models.mxCellDecoder import parse_object


def remove_entity(ent_name):
    ent, style = interactive[ent_name]
    payload = ObjectManager.MANAGER_EVENT(ent, 'remove')
    event = EVENT(ObjectManager.MANAGER_TAG, payload)
    eventStore.put(event)
    interactive[ent_name][0] = -1
    draw2ent[style['id']][0] = -1


def recreate_entity(name, newpos=(100, 100)):
    ent, style = interactive[name]
    if ent != -1:
        print(f'Alredy entity with name {name} (is #{ent})')
        return
    new_obj = parse_object(style['skeleton'], batch, ((WIDTH, HEIGHT), DEFAULT_LINE_WIDTH))
    components, attributes = new_obj
    pos = components[0]
    pos.x = newpos[0] - (pos.w // 2)
    pos.y = newpos[1] - (pos.h // 2)
    pos.center = newpos
    pos.changed = True
    newent = world.create_entity()
    for c in components:
        world.add_component(newent, c)
    interactive[name] = [newent, attributes]
    draw2ent[style['id']][0] = -1

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
interactive = simulation['interactive']

print(f'==> Interactive')
for name, obj in interactive.items():
    print(f'{name} - entity #{obj[0]}')

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
    if key == KEYS.P:
        print('Removing')
        remove_entity('medicine')
    if key == KEYS.R:
        print('Re-creating')
        recreate_entity('medicine')
    if key == KEYS.D:
        robot_has_renderable = world.has_component(1, Renderable)
        med_has_renderable = world.has_component(13, Renderable)
        print(f'Robot has renderable? {robot_has_renderable}')
        print(f'Medicine has renderable? {med_has_renderable}')
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
    print(f'Exiting from window')
    EXIT = False


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
    env.process(StopCollision.process(kwargs))
    # env.process(energySystem.process(kwargs))
    env.process(ObjectManager.process(kwargs))
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
    return 0


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
