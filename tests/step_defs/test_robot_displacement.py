import pytest
from pytest_bdd import scenarios, given, when, then

from simulator.main import Simulator
from simulator.components.Map import Map

from tests.helpers.ScenarioCreationHelper import ScenarioCreationHelper
from tests.helpers.AssertionHelper import AssertionHelper

scenarios('../features/robot_displacement.feature')

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

@given("a map with three rooms", target_fixture="simulation")
def map_with_three_rooms(config):
    config["map"] = "three_room_map.drawio"
    simulation = Simulator(config)

    if not simulation.world.has_component(1, Map):
        simulation.world.add_component(1, Map())

    return simulation

@given("a robot with the ability to follow a path")
def ability_to_follow_path(scenario_helper):
    scenario_helper.add_ability_to_follow_path()

@given("a robot with the ability to navigate")
def ability_to_navigate(scenario_helper):
    scenario_helper.add_ability_to_navigate()

@given("a gotoPos event to the center of 'room_three'")
def goto_event_to_center_of_room_three(scenario_helper):
    room_three_center = scenario_helper.get_center('room_three')
    scenario_helper.add_goto_position_event('robot', room_three_center)

@given("a gotoPoi event to 'room_three_center'")
def goto_poi_event(scenario_helper):
    scenario_helper.add_goto_poi_event('robot', 'room_three_center')

@given("a path from the robot to the center of 'room_three'")
def path_to_room_three(scenario_helper):
    room_three_center = scenario_helper.get_center('room_three')
    robot_center = scenario_helper.get_center('robot')
    points = [robot_center, room_three_center]
    scenario_helper.create_path('robot', points)

@given("a POI named 'room_three_center' in the center of the room")
def poi_in_center_of_room_three(scenario_helper):
    room_three_center = scenario_helper.get_center('room_three')
    scenario_helper.add_poi("room_three_center", room_three_center)

@when("after run simulation")
def run_simulation(simulation):
    simulation.run()

@then("the robot is in the center of 'room_three'")
def robot_is_in_center_of_room_three(assertion_helper):
    assert assertion_helper.is_in_center_of('robot', 'room_three')