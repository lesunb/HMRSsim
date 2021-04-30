import pytest
from pytest_bdd import scenarios, given, when, then, parsers
from simulator.main import Simulator
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

@given("a map containing robots with camera components", target_fixture="simulation")
def map_robot_with_camera(config):
    config["map"] = "camera_map.drawio"
    simulation = Simulator(config)
    return simulation

@given(parsers.parse("a robot with id '{robot}' that has a camera component"))
def add_camera_to_robot(scenario_helper: ScenarioCreationHelper, robot):
    scenario_helper.add_camera(robot)

@given("all the simulation robots has detection ability")
def recognition_ability(scenario_helper: ScenarioCreationHelper):
    scenario_helper.add_detection_ability()

@given(parsers.parse("a camera event for robot '{robot}' detect a '{person}' that is in the camera field of view"))
def add_event_to_detect_person1(scenario_helper: ScenarioCreationHelper, robot, person):
    scenario_helper.add_camera_detection_event(robot, person)

@given(parsers.parse("a camera event for robot '{robot}' detect a '{person}' that is not in the camera field of view"))
def add_event_to_detect_person2(scenario_helper: ScenarioCreationHelper, robot, person):
    scenario_helper.add_camera_detection_event(robot, person)

@when("after run simulation")
def run_simulation(simulation):
    simulation.run()

@then(parsers.parse("information about the entity '{person}' was detected by the '{robot}' robot camera"))
def captured_the_persons_information(assertion_helper: AssertionHelper, robot, person):
    assert assertion_helper.detected(robot, person) is True

@then(parsers.parse("information about the entity '{person}' was not detected by the '{robot}' robot camera"))
def did_not_captured_the_persons_information(assertion_helper: AssertionHelper, robot, person):
    assert assertion_helper.detected(robot, person) is False