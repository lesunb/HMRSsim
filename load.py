from main import Simulator
#import simulator.systems.SeerPlugin as Seer

#from utils.Firebase import db, clean_old_simulation
#NAMESPACE = 'cris'
#clean_old_simulation(NAMESPACE)

from tests.helpers.ScenarioCreationHelper import ScenarioCreationHelper
from tests.helpers.AssertionHelper import AssertionHelper

#def firebase_seer_consumer(message, msg_idx):
#    """Sends Seer messages to firebase."""
#    if msg_idx >= 0:
#        if msg_idx == 1:
#            for idx, j in enumerate(message):
#                db.child(NAMESPACE).child('live_report').child(msg_idx).child(idx).set({j: message[j]})
#        else:
#            _ = db.child(NAMESPACE).child('live_report').child(msg_idx).set(message)


config = {
        "context": "tests/data",
        "FPS": 60,
        "DLW": 10,
        "duration": 10
    }

config["map"] = "camera_map.drawio"
#config["map"] = "three_room_map.drawio"

simulation = Simulator(config)
scenario_helper = ScenarioCreationHelper(simulation)
assertion_helper = AssertionHelper(simulation)

#simulation.add_des_system(Seer.init([firebase_seer_consumer], 0.05, False),)

scenario_helper.add_camera('robot')
scenario_helper.add_camera('robot2')
scenario_helper.add_camera('robot3')
scenario_helper.add_approximation_ability()
scenario_helper.add_detection_ability()

scenario_helper.add_ability_to_navigate()
pois_tag = 'intersection1, intersection2, patientRoom2, patientRoom3, patientRoom4, intersection2, robotHome'
scenario_helper.add_script_ability()
pois = pois_tag.replace(' ', '').split(',')
for poi in pois:
    scenario_helper.add_go_command('robot', poi)

pois_tag = 'intersection1, intersection2, patientRoom1'
pois = pois_tag.replace(' ', '').split(',')
for poi in pois:
    scenario_helper.add_go_command('robot2', poi)

pois_tag = 'patientRoom3'
pois = pois_tag.replace(' ', '').split(',')
for poi in pois:
    scenario_helper.add_go_command('robot3', poi)

scenario_helper.add_camera_detection_event('robot', 'person3') 
scenario_helper.add_camera_detection_event('robot2', 'person2') 
scenario_helper.add_camera_detection_event('robot3', 'person1') 


simulation.run()


#assert assertion_helper.approximated('robot', 'person3')
