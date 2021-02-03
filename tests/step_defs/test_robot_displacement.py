from tests.aux.testaux import get_component
import pytest
from pytest_bdd import scenarios, given, when, then
from main import Simulator
from components.Path import Path
from components.Position import Position
from systems.PathProcessor import PathProcessor
from systems.MovementProcessor import MovementProcessor

scenarios('../features/robot_displacement.feature')

@pytest.fixture
def config():
    config = {
        "context": "tests/data",
        "FPS": 60,
        "DLW": 10,
        "duration": 10
    }
    return config

@given("a map", target_fixture="simulation") 
def map(config):
    config["map"] = "room3.drawio"
    simulation = Simulator(config)
    return simulation

@given("a 'room3' as a POI in 2,2")
def room3_poi():
    pass #como definir um poi no mapa?

@pytest.fixture
@given("a path to 'room3'")
def target_path(simulation):
    path = get_component(simulation, Path, '15')
    target_path = path.points[-1]
    return { 'x': target_path[0], 'y': target_path[1]}

@when("the robot receives a move-to 'room3'")
def movement(simulation, target_path):
    width, height = simulation.window_dimensions
    simulation.add_system(PathProcessor())
    simulation.add_system(MovementProcessor(minx=0, miny=0, maxx=width, maxy=height))
    simulation.run()

@when("after some time")
def pass_time():
    pass

@then("the robot is in 'room3'")
def check_position(simulation, target_path):
    position = get_component(simulation, Position, '15')
    assert target_path['x'] == position.x
    assert target_path['y'] == position.y