import pytest
from pytest_bdd import scenarios, given, when, then
from main import Simulator

from tests.helpers.ScenarioCreationHelper import ScenarioCreationHelper
from tests.helpers.AssertionHelper import AssertionHelper

scenarios('../features/collision.feature')

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

@given("a map with a collidable wall", target_fixture="simulation")
def map_with_colliding_wall(config):
    config["map"] = "collidable_wall_map.drawio"
    simulation = Simulator(config)
    return simulation

@given("a robot with ability to collide with the wall")
def ability_to_collide(scenario_helper):
    scenario_helper.add_ability_to_collide()

@given("a robot with the ability to follow a path")
def ability_to_follow_path(scenario_helper):
    scenario_helper.add_ability_to_follow_path()

@given("a path from the robot to the collidable wall")
def path_to_collidable_wall(scenario_helper):
    robot_center = scenario_helper.get_center('robot')
    wall_center = scenario_helper.get_center('collidable_wall')
    points = [robot_center, wall_center]
    scenario_helper.create_path('robot', points)

@given("a path from the robot to the second collidable wall")
def path_to_second_collidable_wall(scenario_helper):
    wall_center = scenario_helper.get_center('second_collidable_wall')
    points = [wall_center]
    scenario_helper.create_path('robot', points)

@when("after run simulation")
def run_simulation(simulation):
    simulation.run()

@then("the robot collides with the wall")
def collided_into_the_wall(assertion_helper):
    assert assertion_helper.have_collided('robot', 'collidable_wall')

@then("the robot collides with the second wall")
def collided_into_second_wall(assertion_helper):
    assert assertion_helper.have_collided('robot', 'second_collidable_wall')