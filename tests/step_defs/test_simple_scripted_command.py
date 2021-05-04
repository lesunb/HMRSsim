import pytest
from pytest_bdd import scenarios, given, when, then, parsers
from tests.helpers.ScenarioCreationHelper import ScenarioCreationHelper
from tests.helpers.AssertionHelper import AssertionHelper
from simulator.main import Simulator

@pytest.fixture
def scenario_helper(simulation):
    return ScenarioCreationHelper(simulation)

@pytest.fixture
def assertion_helper(simulation):
    return AssertionHelper(simulation)

scenarios('../features/simple_scripted_command.feature')

@pytest.fixture
def config():
    config = {
        "context": "tests/data",
        "FPS": 60,
        "DLW": 10,
        "duration": 10
    }
    return config

@given("a map with a medicine room, a patient room and a robot with commands", target_fixture="simulation")
def med_and_patient_room_map(config):
    config["map"] = "med_and_patient_room_map_with_commands.drawio"
    simulation = Simulator(config)
    return simulation

@given("the robot has the ability to use a script command")
def ability_to_use_script(scenario_helper):
    scenario_helper.add_script_ability()

@given("the robot has the ability to navigate")
def ability_to_navigate(scenario_helper):
    scenario_helper.add_ability_to_navigate()

@when("after run simulation")
def run_simulation(simulation):
    simulation.run()

@then(parsers.parse("the '{pickable}' is in the '{poi_tag}' poi"))
def pickable_is_in_the_correct_poi(assertion_helper, pickable, poi_tag):
    assert assertion_helper.robot_drop_pickable_in_poi('robot', pickable, poi_tag)