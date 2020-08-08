import pyglet
import os
import sys
import json
import simpy
import pathlib

# Include current directory in path
sys.path.append(os.getcwd())

from components.Velocity import Velocity

import map_parser

from systems.MovementProcessor import MovementProcessor
from systems.CollisionProcessor import CollisionProcessor
from systems.RenderProcessor import RenderProcessor
from systems.PathProcessor import PathProcessor

FPS = 60
DEFAULT_LINE_WIDTH = 10
CONFIG = 'simulation.json' if len(sys.argv) == 1 else sys.argv[1]

with open(CONFIG) as fd:
    config = json.load(fd)
FILE = pathlib.Path(config.get('context', '.')) / config.get('map', 'map.drawio')
SIMULATION_DURATION = config.get('duration', -1)

simulation = map_parser.build_simulation_from_map(FILE)
world = simulation['world']
window = simulation['window']
batch = simulation['batch']
(window_name, (WIDTH, HEIGHT), BKGD) = simulation['window_props']
draw2ent = simulation['draw_map']
objects = simulation['objects']

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


@window.event
def on_key_press(key, mod):
    if key == pyglet.window.key.RIGHT:
        world.component_for_entity(robot1, Velocity).x = 3
    if key == pyglet.window.key.LEFT:
        world.component_for_entity(robot1, Velocity).x = -3
    if key == pyglet.window.key.UP:
        world.component_for_entity(robot1, Velocity).y = 3
    if key == pyglet.window.key.DOWN:
        world.component_for_entity(robot1, Velocity).y = -3


@window.event
def on_key_release(key, mod):
    if key in (pyglet.window.key.RIGHT, pyglet.window.key.LEFT):
        world.component_for_entity(robot1, Velocity).x = 0
    if key in (pyglet.window.key.UP, pyglet.window.key.DOWN):
        world.component_for_entity(robot1, Velocity).y = 0


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
    while not EXIT:
        pyglet.clock.tick()
        world.process([env, killswitch])

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
    if SIMULATION_DURATION > 0:
        env.process(simulation_loop(False))
        env.run(until=SIMULATION_DURATION)
    else:
        killswitch = env.event()
        env.process(simulation_loop(True))
        env.run()
