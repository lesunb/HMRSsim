
import json
import utils
from simulator.main import Simulator

import systems.Cook as CookersSystem
import systems.Customers as CustomerSystem
import systems.KitchenManagement as KitchenManagement
import simulator.systems.ClockSystem as ClockSystem

SIMULATION_SIZE = "MEDIUM3"

with open('./control.json', 'r') as fd:
    control = json.loads(fd.read())
    control = control[SIMULATION_SIZE]

config = {
    "context": ".",
    "map": "kitchen.drawio",
    "verbose": 20,
    "duration": 1000,
    "simulationComponents": {
        "KitchenLayout": [control['personnel']],
        "Menu": []
    }
}


with open('./recipes.json', 'r') as fd:
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
