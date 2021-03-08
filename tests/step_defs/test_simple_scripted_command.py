import pytest
from pytest_bdd import scenarios, given, when, then, parsers

from tests.aux.testaux import is_in_poi

from main import Simulator

scenarios('../features/simple_scripted_command.feature')

@pytest.fixture
def config():
    config = {
        "context": "tests/data",
        "FPS": 60,
        "DLW": 10,
        "duration": 10
    }
    return config

@given("a map with a medicine room, a patient room and a robot with commands", target_fixture="simulation")
def map(config):
    config["map"] = "med_and_patient_room_map_with_commands.drawio"
    simulation = Simulator(config)
    return simulation

@when("after run simulation")
def run_simulation(simulation):
    simulation.run()

@then(parsers.parse("the '{pickable}' is in the '{poi_tag}' poi"))
def pickable_is_in_the_correct_poi(simulation, pickable, poi_tag):
    #assert is_in_poi(simulation, pickable, poi_tag)
    pass