from tests.aux.testaux import create_path, get_component, store_goto_position_event, store_goto_poi_event, get_center, is_in_center_of
import pytest
from pytest_bdd import scenarios, given, when, then
from main import Simulator
from components.Path import Path
from components.Map import Map
from components.Position import Position
from systems.PathProcessor import PathProcessor
from systems.MovementProcessor import MovementProcessor
import systems.GotoDESProcessor as NavigationSystem
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
def map(config):
    config["map"] = "three_room_map.drawio"
    simulation = Simulator(config)

    if not simulation.world.has_component(1, Map):
        simulation.world.add_component(1, Map())
    
    return simulation

@given("a robot with the ability to follow a path")
def ability_to_follow_path(scenario_helper):
    scenario_helper.add_ability_to_follow_path()

@given("a robot with the ability to move to a specific POI")
def ability_to_move_to_specific_poi(scenario_helper):
    scenario_helper.add_ability_to_move_to_specific_poi()

@given("a robot with the ability to move to a specific position")
def ability_to_move_to_specific_position(scenario_helper):
    scenario_helper.add_ability_to_move_to_specific_position()

@given("a gotoPos event to the center of 'room three'")
def goto_event_to_center_of_room_three(scenario_helper):
    room_three_center = scenario_helper.get_center('room3')
    scenario_helper.add_goto_position_event('robot', room_three_center)

@given("a goto POI event")
def goto_poi_event(scenario_helper):
    scenario_helper.add_goto_poi_event('robot', 'room_three_center')

@given("a path from the robot to the center of 'room three'")
def path_to_room_three(scenario_helper):
    room_three_center = scenario_helper.get_center('room3')
    robot_center = scenario_helper.get_center('robot')
    points = [robot_center, room_three_center]
    scenario_helper.create_path('robot', points)
    # TODO: metodo para criar um path de um lugar para o outro

@given("a POI in the center of 'room three'")
def poi_in_center_of_room_three(scenario_helper):
    room_three_center = scenario_helper.get_center('room3')
    scenario_helper.add_poi("room_three_center", room_three_center)

@when("after run simulation")
def run_simulation(simulation):
    simulation.run()

@then("the robot is in the center of 'room three'")
def robot_is_in_center_of_room_three(assertion_helper):
    assert assertion_helper.is_in_center_of('robot', 'room3') == True