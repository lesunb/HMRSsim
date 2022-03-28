from abc import abstractmethod
from simulator.typehints.dict_types import SystemArgs
from simulator.typehints.component_types import EVENT, ERROR

import esper
import logging

import rclpy
from rclpy.node import Node

import re

class RosControlNode(Node):

    def __init__(self):
        super().__init__('ros_control')
        self.logger = logging.getLogger(__name__)

class RosControlObserver():

    @abstractmethod
    def notify(kwargs):
        pass


class RosControlProcessor(esper.Processor):

    rclpy.init()
    node = RosControlNode()
    observers = []

    @staticmethod
    def create_subscription(type, topic_name, callback):
        """
        Create a new subscription to the node of the RosControl system
        """
        subscription = RosControlProcessor.node.create_subscription(type, topic_name, callback, 10)
        return subscription

    @staticmethod
    def remove_subscription(subscription):
        """
        Removes a subscription from the node
        """
        RosControlProcessor.node.destroy_subscription(subscription)
    
    def __init__(self):
        super().__init__()
        self.logger = logging.getLogger(__name__)
        print('Initializing rclpy')

    def add_observer(self, observer):
        RosControlProcessor.observers.append(observer)
    
    def process(self, kwargs: SystemArgs):
        __event_store = kwargs.get('EVENT_STORE', None)
        exit_event = kwargs.get('_KILL_SWITCH', None)

        for observer in RosControlProcessor.observers:
            observer.notify(event_store=__event_store, exit_event=exit_event, world=self.world)

        rclpy.spin_once(RosControlProcessor.node, timeout_sec=0.1)

    def end(self):
        RosControlProcessor.node.destroy_node()
        rclpy.shutdown()