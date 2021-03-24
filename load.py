from simulator.components.Detectable import Detectable
from tests.helpers.AssertionHelper import AssertionHelper
from typehints.component_types import EVENT
from simulator.systems.CameraDESProcessor import CameraTag, CameraPayload
#from simulator.components.Script import Script
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

simulation = Simulator(config)
scenario_helper = ScenarioCreationHelper(simulation)
assertion_helper = AssertionHelper(simulation)

scenario_helper.add_camera('robot')
scenario_helper.add_component(Detectable(), 'person1')  
scenario_helper.add_component(Detectable(), 'person2')
scenario_helper.add_recognition_ability()

entity_id = scenario_helper.cast_id('robot')
payload = CameraPayload(entity_id)
new_event = EVENT(CameraTag, payload)
event_store = scenario_helper.simulation.KWARGS['EVENT_STORE']
event_store.put(new_event)

simulation.run()

print("Captured person1: ", assertion_helper.captured_camera_info('robot', 'person1'))
print("Captured person2: ", assertion_helper.captured_camera_info('robot', 'person2'))