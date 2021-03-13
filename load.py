from tests.helpers.AssertionHelper import AssertionHelper
from simulator.components.Script import Script
from main import Simulator
import simulator.systems.SeerPlugin as Seer

#from utils.Firebase import db, clean_old_simulation
#NAMESPACE = 'cris'
#clean_old_simulation(NAMESPACE)

from tests.helpers.ScenarioCreationHelper import ScenarioCreationHelper

def firebase_seer_consumer(message, msg_idx):
    """Sends Seer messages to firebase"""
    if msg_idx >= 0:
        if msg_idx == 1:
            for idx, j in enumerate(message):
                db.child(NAMESPACE).child('live_report').child(msg_idx).child(idx).set({j: message[j]})
        else:
            _ = db.child(NAMESPACE).child('live_report').child(msg_idx).set(message)



config = {
        "context": "tests/data",
        "FPS": 60,
        "DLW": 10,
        "duration": 10
    }

config["map"] = "med_and_patient_room_map.drawio"

simulation = Simulator(config)
scenario_helper = ScenarioCreationHelper(simulation)

scenario_helper.add_script_ability()
scenario_helper.add_claw_ability('robot')
scenario_helper.add_ability_to_navigate()

scenario_helper.add_go_command('robot', 'medRoom')
#scenario_helper.add_command("Grab medicine", 'robot')
#scenario_helper.add_go_command('robot', 'patientRoom')
#scenario_helper.add_command("Drop medicine", 'robot')

simulation.run()
print("is in poi: ", AssertionHelper(simulation).robot_drop_pickable_in_poi('robot', 'medicine', 'patientRoom'))

print("Robot: ", scenario_helper.get_center('robot'))
print("Poi: ", scenario_helper.get_poi('medRoom'))