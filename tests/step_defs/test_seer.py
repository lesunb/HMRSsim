import pytest
from pytest_bdd import scenarios, given, when, then, parsers
import json
from main import Simulator
from tests.helpers.ScenarioCreationHelper import ScenarioCreationHelper
from tests.helpers.AssertionHelper import AssertionHelper

scenarios('../features/seer.feature')

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

fd = open('tests/data/seer_report.txt', 'w')

def clean():
    fd.close()

def seer_consumer(message, _):
    fd.write(json.dumps(message) + '\n')

@given("a map with three rooms", target_fixture="simulation")
def map_with_three_rooms(config):
    config["map"] = "three_room_map.drawio"
    simulation = Simulator(config, cleanup=clean)
    return simulation

@given("a robot with the ability to follow a path")
def ability_to_follow_path(scenario_helper: ScenarioCreationHelper):
    scenario_helper.add_ability_to_follow_path()

@given("a path from the robot to the center of 'room_three'")
def path_to_room_three(scenario_helper: ScenarioCreationHelper):
    room_three_center = scenario_helper.get_center('room_three')
    robot_center = scenario_helper.get_center('robot')
    points = [robot_center, room_three_center]
    scenario_helper.create_path('robot', points)

@given("the simulation has a Seer system to record all snapshots")
def record_snapshots(scenario_helper: ScenarioCreationHelper):
    scenario_helper.add_seer(seer_consumer)

@when("after run simulation")
def run_simulation(simulation):
    simulation.run()

@then("all snapshots were recorded by the Seer system in a file")
def check_recorded_snapshots():
    with open('tests/data/seer_report.txt', 'r') as report:
        first_line = json.loads(report.readline())

    assert "timestamp" in first_line