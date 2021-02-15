from tests.aux.testaux import get_component, store_goto_position_event, get_path_last_point
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

@given("a map with three rooms", target_fixture="simulation") 
def map(config):
    config["map"] = "room3.drawio"
    simulation = Simulator(config)
    return simulation

@given("a Position 2,2 in 'room3'")
def position_in_2_2(simulation):
    store_goto_position_event(simulation, 'robot', [2,2])

@pytest.fixture
@given("a path to 'room3'")
def path_to_room_3(simulation):
    path = get_component(simulation, Path, 'robot')
    return path

@when("the robot receives a move-to Position 2,2 in 'room3'")
def receive_move_to_2_2(simulation):
    width, height = simulation.window_dimensions
    simulation.add_des_system((gotoProcessor.process,))
    simulation.add_system(PathProcessor())
    simulation.add_system(MovementProcessor(minx=0, miny=0, maxx=width, maxy=height))

@when("the robot receives a move-to end of path in 'room3'")
def receive_move_to_end_of_path(simulation):
    width, height = simulation.window_dimensions
    simulation.add_system(PathProcessor())
    simulation.add_system(MovementProcessor(minx=0, miny=0, maxx=width, maxy=height))

@when("after run simulation")
def run_simulation(simulation, path_to_room_3):
    simulation.run()

@then("the robot is in Position 2,2 of 'room3'")
def assert_robot_is_in_position_2_2(simulation):
    position = get_component(simulation, Position, 'robot')
    assert position.center[0] == 2
    assert position.center[1] == 2

@then("the robot is in the end of path in 'room3'")
def assert_robot_is_in_end_of_path(simulation, path_to_room_3):
    position = get_component(simulation, Position, 'robot')
    target_path = get_path_last_point(path_to_room_3)
    assert target_path['x'] == position.center[0]
    assert target_path['y'] == position.center[1]