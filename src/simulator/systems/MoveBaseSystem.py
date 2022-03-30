from simulator.components.Position import Position
from simulator.components.Velocity import Velocity
from simulator.typehints.ros_types import RosService
from simulator.typehints.component_types import EVENT, GotoPosPayload, GotoPoiPayload, GotoPosEventTag, GotoPoiEventTag

import logging

from std_msgs.msg import String
from typing import List

import re

class MoveBaseSystem(RosService):

    def __init__(self, **kwargs):
        super().__init__()
        self.logger = logging.getLogger(__name__)
        self.event_store = kwargs.get('event_store', None)
        self.exit_event = kwargs.get('exit_event', None)
        self.world = kwargs.get('world', None)

    def init(self):
        print('Initializing move base subscription')

    def listener_callback(self, msg):
        self.logger.info('I heard: "%s"' % msg.data)
        if not self.event_store:
            self.logger.warn('Could not find event store')
            return
        if msg.data == "exit" and self.exit_event:
            self.event_store.put(self.exit_event)
            return
        if re.match('goto [0-9]{1,} [0-9]{1,}', msg.data):
            instruction = msg.data.split(' ')
            self.logger.info('position received: ' + instruction[1] + ', ' + instruction[2])
            if not self.world:
                return
            for ent, (vel, pos) in self.world.get_components(Velocity, Position):
                self.go_to(ent, [instruction[1], instruction[2]])
            return

    def go_to(self, ent, args: List[str]):
        if len(args) == 1:
            payload = GotoPoiPayload(ent, args[0])
            new_event = EVENT(GotoPoiEventTag, payload)
        elif len(args) == 2:
            payload = GotoPosPayload(ent, [float(args[0]), float(args[1])])
            new_event = EVENT(GotoPosEventTag, payload)
        else:
            raise Exception('GO instruction failed. Go <poi> OR Go <x> <y>')
        if new_event:
            self.event_store.put(new_event)
    
    def get_listener_callback(self):
        return self.listener_callback
    
    def get_service_type(self):
        return String

    def get_name(self):
        return 'movebase/robot'