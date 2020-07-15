import pyglet
import esper
import os
import sys

# Include current directory in path
sys.path.append(os.getcwd())

from primitives import Rectangle
from models.Shape.Shape import Shape
from components.Position import Position
from components.Collidable import Collidable
from collision import Poly, Vector

from components.Velocity import Velocity
from components.Collidable import Collidable
from components.Position import Position
from systems.MovementProcessor import MovementProcessor
from systems.CollisionProcessor import CollisionProcessor
from systems.RenderProcessor import RenderProcessor

world = esper.World()
window = pyglet.window.Window(width=500,
                              height=500,
                              caption="Sample")

batch = pyglet.graphics.Batch()
rectangle = Rectangle(100, 100, 64, 64, { 'fillColor': '#FF0000'})
pos = Position(x=100, y=100, w=64, h=64)
col = Collidable(shape=Poly.from_box(Vector(132, 132), 64, 64))
shape = Shape(pos, rectangle, col)

shape.add_to_batch(batch)
entity = shape.add_to_world(world)

world.add_component(entity, Velocity())

# collision_processor = CollisionProcessor()
# world.add_processor(collision_processor)

render_processor = RenderProcessor()
world.add_processor(render_processor)

movement_processor = MovementProcessor(
    minx=0, miny=0, maxx=500, maxy=500)
world.add_processor(movement_processor)

@window.event
def on_key_press(key, mod):
    if key == pyglet.window.key.RIGHT:
        world.component_for_entity(entity, Velocity).x = 3
    if key == pyglet.window.key.LEFT:
        world.component_for_entity(entity, Velocity).x = -3
    if key == pyglet.window.key.UP:
        world.component_for_entity(entity, Velocity).y = 3
    if key == pyglet.window.key.DOWN:
        world.component_for_entity(entity, Velocity).y = -3
    if key == pyglet.window.key.P:
        col = world.component_for_entity(entity, Collidable)
        pos = world.component_for_entity(entity, Position)
        print("[Robot]\nCollision: {}\nPosition: {}".format(col.shape, pos))

@window.event
def on_key_release(key, mod):
    if key in (pyglet.window.key.RIGHT, pyglet.window.key.LEFT):
        world.component_for_entity(entity, Velocity).x = 0
    if key in (pyglet.window.key.UP, pyglet.window.key.DOWN):
        world.component_for_entity(entity, Velocity).y = 0

@window.event
def on_draw():
    # Clear the window to background color
    window.clear()
    # Draw the batch of Renderables:
    batch.draw()

if __name__ == "__main__":
    # NOTE!  schedule_interval will automatically pass a "delta time" argument
    #        to world.process, so you must make sure that your Processor classes
    #        account for this. See the example Processors above.
    pyglet.clock.schedule_interval(world.process, interval=1.0/ 60)
    pyglet.app.run()