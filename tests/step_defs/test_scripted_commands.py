import pytest
from pytest_bdd import scenarios, given, when, then, parsers

from main import Simulator

scenarios('../features/scripted_commands.feature')

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
def map(config):
    config["map"] = "med_and_patient_room_map.drawio"
    simulation = Simulator(config)
    return simulation

@given("a robot with the ability to follow a script command")
def ability_to_follow_a_script_command():
    pass

@given("a robot with the ability to grab pickables")
def ability_to_grab_pickables():
    pass

@given(parsers.parse("a script command 'Go to' {poi_tag} poi"))
def script_command_go_to(poi_tag):
    pass

@given(parsers.parse("a script command {command_name} {pickable_name} pickable"))
def script_command_grab_and_drop(command_name, pickable_name):
    pass

@when("after run simulation")
def run_simulation(simulation):
    simulation.run()

@then("the robot is in the 'medRoom' poi")
def robot_is_in_medicine_room():
    pass

@then("the 'medicine' is in the 'patientRoom' poi")
def medicine_is_in_patient_room(simulation):
    pass

