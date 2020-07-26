import pyglet
import esper
import os
import sys

# Include current directory in path
sys.path.append(os.getcwd())


from components.Velocity import Velocity

import map_parser
import prefabs

from systems.MovementProcessor import MovementProcessor
from systems.CollisionProcessor import CollisionProcessor
from systems.RenderProcessor import RenderProcessor
from systems.PathProcessor import PathProcessor


FPS = 60
DEFAULT_LINE_WIDTH = 10
FILE = 'tilted_walls.drawio' if len(sys.argv) == 1 else sys.argv[1] 

simulation = map_parser.build_simulation_from_map(FILE)

world = simulation['world']
window = simulation['window']
batch = simulation['batch']
(window_name, (WIDTH, HEIGHT), BKGD) = simulation['window_props']
draw2ent = simulation['draw_map']
objects = simulation['objects']

robot1 = objects[0] if len(objects) > 0 else None

# Create some Processor instances, and asign them to the World to be processed:
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


####################################################
#  Schedule a World update and start the pyglet app:
####################################################
if __name__ == "__main__":
    # NOTE!  schedule_interval will automatically pass a "delta time" argument
    #        to world.process, so you must make sure that your Processor classes
    #        account for this. See the example Processors above.
    pyglet.clock.schedule_interval(world.process, interval=1.0/FPS)
    pyglet.app.run()
