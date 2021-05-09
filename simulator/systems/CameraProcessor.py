from simulator.main import Simulator
from simulator.components.Position import Position
from simulator.components.Camera import Camera
from simulator.typehints.component_types import EVENT
from typing import NamedTuple

DetectedPayload = NamedTuple('CapturedPayload', [('entity', int), ('target_id', int),('target_info', int),])

def process_camera_event(sensor: Camera, target, simulation: Simulator):
    while True:
        event = yield sensor.reply_channel.get() # subscriber - fica esperando a mensagem do sensor
        payload = event.payload
        entities = payload.close_entities
        for entity in entities:
            if target == entity.other_ent:
                detected_position = Position(entity.other_pos.x, entity.other_pos.y)
                sensor.detected_entities[target] = {'detected_position': detected_position, 'time': str(simulation.ENV.now)}
                send_detected_event(payload.ent, target, sensor.detected_entities[target], simulation)

def send_detected_event(me, target_id, target_info, simulation: Simulator):
    payload = DetectedPayload(me, target_id, target_info)
    new_event = EVENT('Detected', payload)
    simulation.KWARGS['EVENT_STORE'].put(new_event)
