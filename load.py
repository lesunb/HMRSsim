from components.Collision import Collision
from components.Path import Path
from components.Position import Position
from main import Simulator
from systems.PathProcessor import PathProcessor
from systems.MovementProcessor import MovementProcessor
from systems.CollisionProcessor import CollisionProcessor
from tests.aux.testaux import get_component, create_path, have_collided, get_collision
import systems.GotoDESProcessor as gotoProcessor
from tests.aux.report import clean, set_report

import systems.CollisionDetectorDESProcessor as collisionDetector


config = {
        "context": "tests/data",
        "FPS": 60,
        "DLW": 10,
        "duration": 10
    }

config["map"] = "collidable_wall_map.drawio"

simulation = Simulator(config)

print(simulation.objects)
print(simulation.draw2ent)

robot = get_component(simulation, Position, 'robot')
wall = get_component(simulation, Position, 'collidable_wall')
points = [robot.center, wall.center]
create_path(simulation, 'robot', points)

wall = get_component(simulation, Position, 'second_collidable_wall')
points = [wall.center]
create_path(simulation, 'robot', points)  #só pode ter um path também, adicionar nos paths.

simulation.add_des_system((collisionDetector.process,))

simulation.add_system(CollisionProcessor())
width, height = simulation.window_dimensions
simulation.add_system(PathProcessor())
simulation.add_system(MovementProcessor(minx=0, miny=0, maxx=width, maxy=height))

simulation.run()

collisions = get_component(simulation, Collision, 'robot')

print(collisions)

print(get_collision(simulation, 'robot', 'second_collidable_wall'))
print(have_collided(simulation, 'robot', 'second_collidable_wall'))

#print(have_collided(simulation, 'robot', 'collidable_wall'))
#print("robot position after: ", robot_position.center)
#print(simulation.draw2ent)