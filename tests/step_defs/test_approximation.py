import pytest
from pytest_bdd import scenarios, given, when, then, parsers
from main import Simulator

from tests.helpers.ScenarioCreationHelper import ScenarioCreationHelper
from tests.helpers.AssertionHelper import AssertionHelper

scenarios('../features/approximation.feature')

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
def scenario_helper(simulation):
    return ScenarioCreationHelper(simulation)

@pytest.fixture
def assertion_helper(simulation):
    return AssertionHelper(simulation)

@given("a map containing a robot with a camera", target_fixture="simulation")
def map_robot_with_camera(config):
    config["map"] = "camera_map.drawio"
    simulation = Simulator(config)
    return simulation

@given("a robot with a camera component")
def add_camera_to_robot(scenario_helper):
    scenario_helper.add_camera('robot')

@given("a robot with an approximation history component")
def add_approximation_history_to_robot(scenario_helper):
    scenario_helper.add_approximation_history_component('robot')

@given("a robot with approximation ability")
def ability_to_approximate(scenario_helper):
    scenario_helper.add_approximation_ability()

@given("an ability to detect other entities")
def recognition_ability(scenario_helper):
    scenario_helper.add_detection_ability()

@given("a robot with the ability to navigate")
def ability_to_navigate(scenario_helper):
    scenario_helper.add_ability_to_navigate()

@given(parsers.parse("a Go to command for the robot to pass through POIs '{pois_tag}'"))
def pass_through_pois(scenario_helper, pois_tag):
    scenario_helper.add_script_ability()
    pois = pois_tag.replace(' ', '').split(',')
    for poi in pois:
        scenario_helper.add_go_command('robot', poi)

@given("a detectable entity named 'person3' in the cameras field of view")
def person_in_field_of_view(scenario_helper):
    scenario_helper.make_detectable('person3')

@given("a detectable entity named 'person2' that isnt in the cameras field of view")
def person_not_in_field_of_view(scenario_helper):
    scenario_helper.make_detectable('person2')

@given(parsers.parse("a camera event to detect '{person}'"))
def add_camera_detection(scenario_helper, person):
    scenario_helper.add_camera_detection_event('robot', person)  

@when("after run simulation")
def run_simulation(simulation):
    simulation.run()

@then(parsers.parse("the robot approximated the '{person}'"))
def did_approximated(assertion_helper, person):
    assert assertion_helper.approximated('robot', person)

@then(parsers.parse("the robot did not approximated the '{person}'"))
def did_not_approximated(assertion_helper, person):
    assert assertion_helper.approximated('robot', person) is False