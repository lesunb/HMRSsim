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
def add_camera_to_robot(scenario_helper: ScenarioCreationHelper):
    scenario_helper.add_camera('robot')

@given("a robot with approximation ability")
def ability_to_approximate(scenario_helper: ScenarioCreationHelper):
    scenario_helper.add_approximation_ability()

@given("an ability to detect other entities")
def recognition_ability(scenario_helper: ScenarioCreationHelper):
    scenario_helper.add_detection_ability('robot')

@given("a robot with the ability to navigate")
def ability_to_navigate(scenario_helper: ScenarioCreationHelper):
    scenario_helper.add_ability_to_navigate()

@given(parsers.parse("a Go to command for the robot to pass through POIs '{pois_tag}'"))
def pass_through_pois(scenario_helper: ScenarioCreationHelper, pois_tag):
    scenario_helper.add_script_ability()
    pois = pois_tag.replace(' ', '').split(',')
    for poi in pois:
        scenario_helper.add_go_command('robot', poi)

@given("a camera event to detect a 'person3' that is in the camera field of view")
def add_event_to_detect_person3(scenario_helper: ScenarioCreationHelper):
    scenario_helper.add_camera_detection_event('robot', 'person3') # TODO: mudar o nome dessas pessoas

@given("a camera event to detect a 'person2' that is not in the camera field of view")
def add_event_to_detect_person2(scenario_helper: ScenarioCreationHelper):
    scenario_helper.add_camera_detection_event('robot', 'person2')

@when("after run simulation")
def run_simulation(simulation):
    simulation.run()

@then(parsers.parse("the robot approximated the '{person}'"))
def did_approximated(assertion_helper: AssertionHelper, person):
    assert assertion_helper.approximated('robot', person)

@then(parsers.parse("the robot did not approximated the '{person}'"))
def did_not_approximated(assertion_helper: AssertionHelper, person):
    assert assertion_helper.do_not_approximated('robot', person)