from tests.aux.testaux import create_path, get_component, store_goto_position_event, get_center
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

@pytest.fixture
def robot():
    return "robot"


@given("a map with three rooms", target_fixture="simulation") 
def map(config):
    config["map"] = "three_room_map.drawio"
    simulation = Simulator(config)
    return simulation

@given("a robot with the ability to follow a path")
def ability_to_follow_path(simulation):
    width, height = simulation.window_dimensions
    simulation.add_system(PathProcessor())
    simulation.add_system(MovementProcessor(minx=0, miny=0, maxx=width, maxy=height))

@given("a robot with the ability to move to a specific position")
def ability_to_move_to_specific_position(simulation):
    width, height = simulation.window_dimensions
    simulation.add_des_system((gotoProcessor.process,))
    simulation.add_system(PathProcessor())
    simulation.add_system(MovementProcessor(minx=0, miny=0, maxx=width, maxy=height))

@given("a goto event to the center of 'room three'")
def goto_event_to_center_of_room_three(simulation, robot):
    room_three_center = get_center(simulation, 'room3')
    store_goto_position_event(simulation, robot, room_three_center)

@given("a path from the robot to the center of 'room three'")
def path_to_room_three(simulation, robot):
    room_three_center = get_center(simulation, 'room3')
    robot_center = get_component(simulation, Position, robot).center
    points = [robot_center, room_three_center]
    create_path(simulation, robot, points)

@when("after run simulation")
def run_simulation(simulation):
    simulation.run()

@then("the robot is in the center of 'room three'")
def robot_is_in_center_of_room_three(simulation, robot):
    robot_position = get_component(simulation, Position, robot)
    room_three_center = get_center(simulation, 'room3')

    assert robot_position.center[0] == room_three_center[0]
    assert robot_position.center[1] == room_three_center[1]