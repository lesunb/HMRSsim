import pyglet
import esper

import prefabs

from components.Velocity import Velocity

from systems.MovementProcessor import MovementProcessor
from systems.CollisionProcessor import CollisionProcessor
from systems.RenderProcessor import RenderProcessor


FPS = 60
RESOLUTION = 720, 480


###############################################
#  Initialize pyglet window and graphics batch:
###############################################
window = pyglet.window.Window(width=RESOLUTION[0],
                              height=RESOLUTION[1],
                              caption="Esper pyglet example")
batch = pyglet.graphics.Batch()

# Initialize Esper world, and create a "player" Entity with a few Components:
world = esper.World()
robot1 = prefabs.robot(world, batch, x=0, y=0)
robot2 = prefabs.robot(world, batch, x=100, y=100)

# Create some Processor instances, and asign them to the World to be processed:
movement_processor = MovementProcessor(
    minx=0, miny=0, maxx=RESOLUTION[0], maxy=RESOLUTION[1])
world.add_processor(movement_processor)

collision_processor = CollisionProcessor()
world.add_processor(collision_processor)

render_processor = RenderProcessor()
world.add_processor(render_processor)

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
    # Clear the window:
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
