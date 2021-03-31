import pytest
from pytest_bdd import scenarios, given, when, then
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
def recognition_ability(scenario_helper: ScenarioCreationHelper):
    scenario_helper.add_detection_ability('robot')

@given("a camera event to detect a 'person1' that is in the camera field of view")
def add_event_to_detect_person1(scenario_helper: ScenarioCreationHelper):
    scenario_helper.add_camera_detection_event('robot', 'person1')

@given("a camera event to detect a 'person2' that is not in the camera field of view")
def add_event_to_detect_person2(scenario_helper: ScenarioCreationHelper):
    scenario_helper.add_camera_detection_event('robot', 'person2')

@when("after run simulation")
def run_simulation(simulation):
    simulation.run()

@then("information about the entity 'person1' is detected by the camera")
def captured_the_persons_information(assertion_helper):
    assert assertion_helper.captured_camera_info('robot', 'person1') is True

@then("information about the entity 'person2' is not detected by the camera")
def did_not_captured_the_persons_information(assertion_helper):
    assert assertion_helper.captured_camera_info('robot', 'person2') is False