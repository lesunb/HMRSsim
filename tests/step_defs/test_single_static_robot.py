import pytest
from pytest_bdd import scenarios, given, when, then
from main import Simulator
from tests.aux.testaux import get_component
from components.Position import Position

scenarios('../features/single_static_robot.feature')


@given("a map with one room", target_fixture="simulation")
def map_with_one_room():
    config = {
        "context": "tests/data",
        "map": "one_room_map.drawio",
        "FPS": 60,
        "DLW": 10,
        "duration": 10
    }
    simulation = Simulator(config)
    return simulation

@given("a robot in position 1,1 inside the room")
def a_robot_in_position_1_1(simulation):
    robot_position = get_component(simulation, Position, 'robot')
    robot_position.x = 1
    robot_position.y = 1

@when("the robot stay still")
def stay_still(simulation):
    simulation.run()

@then("the robot is in 1,1")
def check_the_robot_is_in_1_1(simulation):
    robot_position = get_component(simulation, Position, 'robot')
    assert robot_position.x == 1
    assert robot_position.y == 1