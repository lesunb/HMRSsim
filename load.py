from simulator.components.Script import Script
from main import Simulator

from tests.helpers.ScenarioCreationHelper import ScenarioCreationHelper


config = {
        "context": "tests/data",
        "FPS": 60,
        "DLW": 10,
        "duration": 10
    }

config["map"] = "med_and_patient_room_map_with_commands.drawio"

simulation = Simulator(config)
scenario_helper = ScenarioCreationHelper(simulation)

scenario_helper.add_script_ability()
scenario_helper.add_ability_to_navigate()
print("objects: ", simulation.objects)
print("robot components: ", simulation.world.components_for_entity(2))
script = simulation.world.component_for_entity(2, Script)

#simulation.world.add_component(2, Script())

#print("Before: ", scenario_helper.get_position('medicine'))
#print("Before: ", scenario_helper.get_position('robot'))
simulation.run()

#print("After: ", scenario_helper.get_position('medicine'))
#print("After: ", scenario_helper.get_position('robot'))