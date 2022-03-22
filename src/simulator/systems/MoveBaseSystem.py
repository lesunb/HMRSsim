from simulator.typehints.dict_types import SystemArgs
import simulator.systems.GotoDESProcessor as NavigationSystem
from simulator.typehints.component_types import EVENT, ERROR

import esper
from esper import World
from simpy import FilterStore, Environment
import logging

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

import re


class MoveBaseProcessor(esper.Processor):
    def __init__(self, exit_event=None):
        super().__init__()
        self.logger = logging.getLogger(__name__)
        print('Initializing rclpy')
        rclpy.init()
        self.move_base_sub = MinimalSubscriber()
        self.move_base_sub.exit_event = exit_event

    def process(self, kwargs: SystemArgs):
        __event_store = kwargs.get('EVENT_STORE', None)
        __world: World = kwargs.get('WORLD', None)
        env: Environment = kwargs.get('ENV', None)

        if __event_store is None:
            raise Exception("Can't find eventStore")
        
        self.move_base_sub.event_store = __event_store
        rclpy.spin_once(self.move_base_sub, timeout_sec=0.1)
        self.logger.info('Spinned movebase/robot topic...')
    
    def end(self):
        self.move_base_sub.destroy_node()
        rclpy.shutdown()


class MinimalSubscriber(Node):

    def __init__(self):
        # TODO Depois mudar isso aqui para um tópico do tipo Movebase
        super().__init__('minimal_subscriber')
        self.logger = logging.getLogger(__name__)
        
        self.subscription = self.create_subscription(
            String,
            'movebase/robot',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.exit_event = None
        self.event_store = None

    def listener_callback(self, msg):
        self.logger.info('I heard: "%s"' % msg.data)
        if not self.event_store:
            self.logger.warn('Could not find evento store')
            return
        if msg.data == "exit" and self.exit_event:
            self.event_store.put(self.exit_event)
            return
        if re.match('goto [0-9]{1,} [0-9]{1,}', msg.data):
            instruction = msg.data.split(' ')
            print('positions received: ' + instruction[1] + ' and ' + instruction[2])
            NavigationSystem.goInstruction(2, [instruction[1], instruction[2]], None, self.event_store)
            return