from components.Collision import Collision
from components.Path import Path
from components.Position import Position
from components.Map import Map
from main import Simulator
from systems.PathProcessor import PathProcessor
from systems.MovementProcessor import MovementProcessor
from systems.CollisionProcessor import CollisionProcessor
from tests.aux.testaux import get_component, create_path, store_goto_poi_event, get_center, store_goto_position_event, add_component
import systems.GotoDESProcessor as NavigationSystem
from tests.aux.report import clean, set_report

import systems.CollisionDetectorDESProcessor as collisionDetector


config = {
        "context": "tests/data",
        "FPS": 60,
        "DLW": 10,
        "duration": 10
    }

config["map"] = "three_room_map.drawio"

simulation = Simulator(config)

NavigationSystemProcess = NavigationSystem.init()
simulation.add_des_system((NavigationSystemProcess,))
width, height = simulation.window_dimensions
simulation.add_system(PathProcessor())
simulation.add_system(MovementProcessor(minx=0, miny=0, maxx=width, maxy=height))

if not simulation.world.has_component(1, Map):
    simulation.world.add_component(1, Map())

"""
room_three_center = get_center(simulation, 'room3')
print("room three center: ", room_three_center)
store_goto_position_event(simulation, 'robot', room_three_center)
"""

room_three_center = get_center(simulation, 'room3')
map = simulation.world.component_for_entity(1, Map)
map.pois["room_three_center"] = room_three_center

store_goto_poi_event(simulation, 'robot', 'room_three_center')


robot_position = get_component(simulation, Position, 'robot')
print("before: ", robot_position.center)

simulation.run()

robot_position = get_component(simulation, Position, 'robot')
print(robot_position.center)
