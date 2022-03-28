import simulator.systems.GotoDESProcessor as NavigationSystem
import simulator.systems.RosControlSystem as RosControlSystem
from simulator.components.Position import Position
from simulator.components.Velocity import Velocity

import logging

from std_msgs.msg import String

import re

class MoveBaseObserver(RosControlSystem.RosControlObserver):

    def __init__(self):
        super().__init__()
        self.logger = logging.getLogger(__name__)
        self.subscription = RosControlSystem.RosControlProcessor.create_subscription(String, 'movebase/robot', self.listener_callback)
        self.subscription # prevent not used warning

    def notify(self, **kwargs):
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
            print('positions received: ' + instruction[1] + ' and ' + instruction[2])
            if not self.world:
                return
            for ent, (vel, pos) in self.world.get_components(Velocity, Position):
                NavigationSystem.goInstruction(ent, [instruction[1], instruction[2]], None, self.event_store)
            return