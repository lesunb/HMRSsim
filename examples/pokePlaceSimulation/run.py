
import json
import utils
from simulator.main import Simulator

import examples.pokePlaceSimulation.systems.Cook as CookersSystem
import examples.pokePlaceSimulation.systems.Customers as CustomerSystem
import examples.pokePlaceSimulation.systems.KitchenManagement as KitchenManagement
import simulator.systems.ClockSystem as ClockSystem

SIMULATION_SIZE = "MEDIUM3"

with open('./examples/pokePlaceSimulation/control.json', 'r') as fd:
    control = json.loads(fd.read())
    control = control[SIMULATION_SIZE]

config = {
    "context": "examples/pokePlaceSimulation",
    "map": "kitchen.drawio",
    "verbose": True,
    "duration": 1000,
    "simulationComponents": {
        "KitchenLayout": [control['personnel']],
        "Menu": []
    }
}


with open('./examples/pokePlaceSimulation/recipes.json', 'r') as fd:
    recipes = json.loads(fd.read())
config['simulationComponents']['Menu'] = [{k: utils.recipe_from_json(v, k) for k, v in recipes.items()}]
simulator = Simulator(config)

processors = [
    (KitchenManagement.process,),
    (CookersSystem.process,),
    (ClockSystem.process, ClockSystem.clean),
    (CustomerSystem.init(control['stress']),)
]

for p in processors:
    simulator.add_des_system(p)

simulator.run()
