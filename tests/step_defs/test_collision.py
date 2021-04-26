import pytest
from pytest_bdd import scenarios, given, when, then, parsers
from main import Simulator

from simulator.components.Collision import Collision

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

@given("all the simulations robots has the ability to collide with the wall")
def ability_to_collide(scenario_helper: ScenarioCreationHelper):
    scenario_helper.add_ability_to_collide()

@given("all the simulation robots has the ability to follow a path")
def ability_to_follow_path(scenario_helper: ScenarioCreationHelper):
    scenario_helper.add_ability_to_follow_path()

@given(parsers.parse("the '{robot}' has a collision component that stores the collision events"))
def add_collision_component(scenario_helper, robot):
    scenario_helper.add_collision_component(robot)

@given(parsers.parse("a path from the '{robot}' to the '{collidable_wall}'"))
def path_to_collidable_wall(scenario_helper: ScenarioCreationHelper, robot, collidable_wall):
    robot_center = scenario_helper.get_center(robot)
    wall_center = scenario_helper.get_center(collidable_wall)
    points = [robot_center, wall_center]
    scenario_helper.create_path(robot, points)

@when("after run simulation")
def run_simulation(simulation):
    simulation.run()

@then(parsers.parse("the '{robot}' collides with the '{collidable_wall}'"))
def collided_into_the_wall(assertion_helper: AssertionHelper, robot, collidable_wall):
    assert assertion_helper.have_collided(robot, collidable_wall)
