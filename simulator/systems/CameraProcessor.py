from esper import World
from simpy import FilterStore,Environment
from main import Simulator
from simulator.components.Detectable import Detectable
from simulator.components.Position import Position
from simulator.components.Camera import Camera
from typehints.component_types import EVENT
from typehints.dict_types import SystemArgs
from typing import NamedTuple
import logging

DetectedPayload = NamedTuple('CapturedPayload', [('entity', int), ('target_id', int),('target_info', int),])

def process_camera_event(sensor, target, simulation: Simulator):
    while True:
        event = yield sensor.reply_channel.get()
        payload = event.payload
        entities = payload.close_entities
        camera: Camera = simulation.world.component_for_entity(payload.ent, Camera)
        for entity in entities:
            if target == entity.other_ent:
                detected_position = Position(entity.other_pos.x, entity.other_pos.y)
                camera.detected_entities[target] = {'detected_position': detected_position, 'time': str(simulation.ENV.now)}
                send_detected_event(payload.ent, target, camera.detected_entities[target], simulation)

def send_detected_event(me, target_id, target_info, simulation: Simulator):
    payload = DetectedPayload(me, target_id, target_info)
    new_event = EVENT('Detected', payload)
    simulation.KWARGS['EVENT_STORE'].put(new_event)
