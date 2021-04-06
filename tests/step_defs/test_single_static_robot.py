import pytest
from pytest_bdd import scenarios, given, when, then, parsers
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

@given(parsers.parse("a robot with id '{robot}' in position '{position}' inside the room"))
def a_robot_in_given_position(scenario_helper: ScenarioCreationHelper, robot, position):
    pos = position.replace(' ', '').split(',')
    scenario_helper.set_position(robot, int(pos[0]), int(pos[1]))

@when("the robot stay still")
def stay_still(simulation):
    simulation.run()

@then(parsers.parse("the robot with id '{robot}' is in '{position}'"))
def check_the_robot_is_in_position(assertion_helper: AssertionHelper, robot, position):
    pos = position.replace(' ', '').split(',')
    assert assertion_helper.is_in_position(robot, (int(pos[0]),int(pos[1])))