import pytest
from pytest_bdd import scenarios, given, when, then, parsers
from main import Simulator

from tests.helpers.ScenarioCreationHelper import ScenarioCreationHelper
from tests.helpers.AssertionHelper import AssertionHelper

scenarios('../features/camera.feature')

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

@given("an ability to detect other entities")
def recognition_ability(scenario_helper):
    scenario_helper.add_detection_ability()

@given("a detectable entity named 'person1' in the cameras field of view")
def person_in_field_of_view(scenario_helper):
    scenario_helper.make_detectable('person1')

@given("a detectable entity named 'person2' that isnt in the cameras field of view")
def person_not_in_field_of_view(scenario_helper):
    scenario_helper.make_detectable('person2')

@given(parsers.parse("a camera event to detect '{person}'"))
def add_camera_detection_event(scenario_helper, person):
    scenario_helper.add_camera_detection_event('robot', person)

@when("after run simulation")
def run_simulation(simulation):
    simulation.run()

@then("information about the entity 'person1' is detected by the camera")
def captured_the_persons_information(assertion_helper):
    assert assertion_helper.captured_camera_info('robot', 'person1') is True

@then("information about the entity 'person2' is not detected by the camera")
def did_not_captured_the_persons_information(assertion_helper):
    assert assertion_helper.captured_camera_info('robot', 'person2') is False