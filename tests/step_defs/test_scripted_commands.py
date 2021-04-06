import pytest
from pytest_bdd import scenarios, given, when, then, parsers

from main import Simulator

from tests.helpers.ScenarioCreationHelper import ScenarioCreationHelper
from tests.helpers.AssertionHelper import AssertionHelper

scenarios('../features/scripted_commands.feature')

@pytest.fixture
def scenario_helper(simulation):
    return ScenarioCreationHelper(simulation)

@pytest.fixture
def assertion_helper(simulation):
    return AssertionHelper(simulation)

@pytest.fixture
def config():
    config = {
        "context": "tests/data",
        "FPS": 60,
        "DLW": 10,
        "duration": 10
    }
    return config

@given("a map with a medicine room and a patient room", target_fixture="simulation")
def med_and_patient_room_map(config):
    config["map"] = "med_and_patient_room_map.drawio"
    simulation = Simulator(config)
    return simulation

@given("all the simulation robots has the ability to follow a script command")
def ability_to_follow_a_script_command(scenario_helper):
    scenario_helper.add_script_ability()

@given(parsers.parse("a robot with id '{robot}' has the ability to grab pickables"))
def ability_to_grab_pickables(scenario_helper: ScenarioCreationHelper, robot):
    scenario_helper.add_claw_ability(robot)

@given("all the simulation robots has the ability to navigate")
def ability_to_navigate(scenario_helper):
    scenario_helper.add_ability_to_navigate()

@given(parsers.parse("a script command 'Go to' '{poi_tag}' poi for '{robot}'"))
def script_command_go_to(scenario_helper: ScenarioCreationHelper, poi_tag, robot):
    scenario_helper.add_go_command(robot, poi_tag)

@given(parsers.parse("a script command '{command_name}' '{pickable_name}' pickable for '{robot}'"))
def script_command_grab_and_drop(scenario_helper: ScenarioCreationHelper, command_name, pickable_name, robot):
    scenario_helper.add_command(f"{command_name} {pickable_name}", robot)

@given(parsers.parse("a list of script commands '{commands}' to robot with id '{robot}'"))
def add_script_commands_to_robot(scenario_helper: ScenarioCreationHelper, commands, robot):
    command_list = [command.strip() for command in commands.split(',')]
    scenario_helper.add_commands(command_list, robot)

@when("after run simulation")
def run_simulation(simulation):
    simulation.run()

@then(parsers.parse("the robot with id '{robot}' is in the '{poi_tag}' poi"))
def robot_is_in_medicine_room(assertion_helper: AssertionHelper, robot, poi_tag):
    assert assertion_helper.is_in_poi(robot, poi_tag)

@then(parsers.parse("the '{robot}' droped the '{medicine}' in the '{poi_tag}' poi"))
def medicine_is_in_patient_room(assertion_helper:AssertionHelper, robot, medicine, poi_tag):
    assert assertion_helper.robot_drop_pickable_in_poi(robot, medicine, poi_tag)