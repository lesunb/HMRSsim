import pyglet
import esper
import os
import sys

# Include current directory in path
sys.path.append(os.getcwd())

import prefabs
import simulator.utils.helpers as helpers
import simulator.resources.load_resources as loader
from simulator.models.Wall.Wall import Wall
from simulator.models.WallCorner import WallCorner
from simulator.models.WallU import WallU
from simulator.models.Room import Room
from simulator.models.Shape.Shape import Shape

from components.Path import Path
from components.Velocity import Velocity
from components.Collidable import Collidable
from components.Position import Position
from systems.MovementProcessor import MovementProcessor
from systems.CollisionProcessor import CollisionProcessor
from systems.RenderProcessor import RenderProcessor
from systems.PathProcessor import PathProcessor


FPS = 60
DEFAULT_LINE_WIDTH = 10
FILE = 'tilted_walls.drawio' if len(sys.argv) == 1 else sys.argv[1] 

# Load map from .drawio file
window_name, map_content = loader.mapFromDrawio(FILE)
WIDTH = int(map_content.attrib.get('pageWidth', 500))
HEIGHT = int(map_content.attrib.get('pageHeight', 500))

BKGD = helpers.hex_to_rgb('#FFFFFF')
if map_content.attrib.get('background', 'none') != 'none':
    BKGD = helpers.hex_to_rgb(map_content.attrib['background'])
content_root = map_content[0]
###############################################
#  Initialize pyglet window and graphics batch:
###############################################
window = pyglet.window.Window(width=WIDTH,
                              height=HEIGHT,
                              caption=window_name)
batch = pyglet.graphics.Batch()
# Define default line width
pyglet.gl.glLineWidth(DEFAULT_LINE_WIDTH)
# Define background clear color
pyglet.gl.glClearColor(BKGD[0], BKGD[1], BKGD[2], BKGD[3])
pyglet.gl.glClear(pyglet.gl.GL_COLOR_BUFFER_BIT | pyglet.gl.GL_DEPTH_BUFFER_BIT)

# Initialize Esper world, and create a "player" Entity with a few Components:
world = esper.World()

# Create Walls
walls = []
robot1 = None
draw2entity = {}
for cell in content_root:
    if cell.tag == 'mxCell' and cell.attrib.get('style', None) is not None:
        cell_style = helpers.parse_style(cell.attrib['style'])
        style = cell_style.get('shape', '')
        if style == 'mxgraph.floorplan.wallCorner':
            walls.append(WallCorner.from_mxCell(cell, (WIDTH, HEIGHT), DEFAULT_LINE_WIDTH))
        elif style == 'mxgraph.floorplan.wallU':
            walls.append(WallU.from_mxCell(cell, (WIDTH, HEIGHT), DEFAULT_LINE_WIDTH))
        elif style == 'mxgraph.floorplan.room':
            walls.append(Room.from_mxCell(cell, (WIDTH, HEIGHT), DEFAULT_LINE_WIDTH))
        elif style == 'mxgraph.floorplan.wall':
            walls.append(Wall.from_mxCell(cell, (WIDTH, HEIGHT), DEFAULT_LINE_WIDTH))
        else:
            walls.append(Shape.from_mxCell(cell, (WIDTH, HEIGHT), DEFAULT_LINE_WIDTH))
        walls[-1].add_to_batch(batch)
        walls[-1].add_to_world(world)
    if cell.tag == 'object':
        if cell.attrib['type'] == 'robot':
            s = Shape.from_object(cell, (WIDTH, HEIGHT), DEFAULT_LINE_WIDTH)
            s.add_to_batch(batch)
            robot1 = s.add_to_world(world)
            draw2entity[cell.attrib['id']] = robot1
        elif cell.attrib['type'] == 'path':
            mxCell = cell[0]
            points = Path.from_mxCell(mxCell, HEIGHT)
            obj = mxCell.attrib.get('source', None)
            ent = draw2entity.get(obj, None)
            if ent is None:
                print("Path origin not found")
            else:
                print("Adding path to entity")
                world.add_component(ent, Path(points))
                print("[run] Path", world.component_for_entity(ent, Path))
                print("[run] Position", world.component_for_entity(ent, Position))
        else:
            print("Unrecognized object", cell)

if robot1 == None:
    print("Creating robot from prefab")
    robot1 = prefabs.robot(world, batch, x=200, y=200)
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
    if key == pyglet.window.key.P:
        for i, w in enumerate(walls):
            print("Wall {} - Position: {}\nCollision: {}\nDraw: {}".format(i, w.pos, w.boundaries, w.draw))
        col = world.component_for_entity(robot1, Collidable)
        pos = world.component_for_entity(robot1, Position)
        print("[Robot]\nCollision: {}\nPosition: {}".format(col.shape, pos))

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
