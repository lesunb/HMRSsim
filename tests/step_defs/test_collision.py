from components.Position import Position
from components.Path import Path
from tests.aux.testaux import get_component, get_style_component, create_path, have_collided
import pytest
from pytest_bdd import scenarios, given, when, then
from main import Simulator
from systems.CollisionProcessor import CollisionProcessor
from systems.PathProcessor import PathProcessor
from systems.MovementProcessor import MovementProcessor

import systems.CollisionDetectorDESProcessor as collisionDetector

scenarios('../features/collision.feature')

@pytest.fixture
def config():
    config = {
        "context": "tests/data",
        "FPS": 60,
        "DLW": 10,
        "duration": 10
    }
    return config

@given("a map with a collidable wall", target_fixture="simulation")
def map_with_colliding_wall(config):
    config["map"] = "collidable_wall_map.drawio"
    simulation = Simulator(config)
    return simulation

@given("a robot with ability to collide with the wall")
def collides_with_the_all(simulation):
    simulation.add_system(CollisionProcessor())
    simulation.add_des_system((collisionDetector.process,))

@given("a robot with the ability to follow a path")
def ability_to_follow_path(simulation):
    width, height = simulation.window_dimensions
    simulation.add_system(PathProcessor())
    simulation.add_system(MovementProcessor(minx=0, miny=0, maxx=width, maxy=height))

@given("a path from the robot to the collidable wall")
def path_to_collidable_wall(simulation):
    robot = get_component(simulation, Position, 'robot')
    wall = get_style_component(simulation, Position, 'collidable_wall')
    points = [robot.center, wall.center]
    create_path(simulation, 'robot', points)

@when("after run simulation")
def run_simulation(simulation):
    simulation.run()

@then("the robot collides with the wall")
def collided_into_the_wall(simulation):  # Não eh possivel validar a posicao que colidiu
    assert have_collided(simulation, 'robot', 'collidable_wall') == True
