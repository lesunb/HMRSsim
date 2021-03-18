import pytest
from pytest_bdd import scenarios, given, when, then
from main import Simulator

from tests.helpers.ScenarioCreationHelper import ScenarioCreationHelper
from tests.helpers.AssertionHelper import AssertionHelper

scenarios('../features/single_static_robot.feature')

@pytest.fixture
def scenario_helper(simulation):
    return ScenarioCreationHelper(simulation)

@pytest.fixture
def assertion_helper(simulation):
    return AssertionHelper(simulation)

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
def a_robot_in_position_1_1(scenario_helper):
    scenario_helper.set_position('robot', 1, 1)

@when("the robot stay still")
def stay_still(simulation):
    simulation.run()

@then("the robot is in 1,1")
def check_the_robot_is_in_1_1(assertion_helper):
    assert assertion_helper.is_in_position('robot', (1,1))