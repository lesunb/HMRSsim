from tests.aux.testaux import get_component, store_gotoPos_event, get_path_last_point
import pytest
from pytest_bdd import scenarios, given, when, then
from main import Simulator
from components.Path import Path
from components.Position import Position
from components.POI import POI
from systems.PathProcessor import PathProcessor
from systems.MovementProcessor import MovementProcessor
import systems.GotoDESProcessor as gotoProcessor

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

@given("a 'room3' as a POI in 2,2", target_fixture="poi")
def poi(simulation):
    store_gotoPos_event(simulation, 'robot', [2,2])
    simulation.add_des_system(gotoProcessor.process)

@pytest.fixture
@given("a path to 'room3'")
def target_path(simulation):
    path = get_component(simulation, Path, 'robot')
    return get_path_last_point(path)

@when("the robot receives a move-to 'room3'")
def movement(simulation, target_path):
    width, height = simulation.window_dimensions
    simulation.add_system(PathProcessor())
    simulation.add_system(MovementProcessor(minx=0, miny=0, maxx=width, maxy=height))

@when("after some time")
def pass_time(simulation):
    simulation.run()

@then("the robot is in 'room3'")
def check_position(simulation, target_path):
    # TODO: fazer o assert do POI, só está verificando do target_path
    position = get_component(simulation, Position, 'robot')
    assert target_path['x'] == position.center[0]
    assert target_path['y'] == position.center[1]