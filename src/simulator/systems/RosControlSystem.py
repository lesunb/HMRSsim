from simulator.typehints.dict_types import SystemArgs
from simulator.typehints.component_types import EVENT, ERROR
from simulator.typehints.ros_types import RosService

import esper
import logging

import rclpy
from rclpy.node import Node

class RosControlNode(Node):

    def __init__(self):
        super().__init__('hmrsim')
        self.logger = logging.getLogger(__name__)

class RosControlProcessor(esper.Processor):

    def __init__(self):
        super().__init__()
        self.logger = logging.getLogger(__name__)
        rclpy.init()
        self.logger.info("Initialized rclpy...")
        self.node = RosControlNode()

    def create_subscription(self, service: RosService):
        """
        Create a new subscription to the node of the RosControl system
        """
        subscription = self.node.create_subscription(service.get_service_type(),
                                                    service.get_name(),
                                                    service.get_listener_callback(),
                                                    10)
        return subscription

    def remove_subscription(self, subscription):
        """
        Removes a subscription from the node
        """
        self.node.destroy_subscription(subscription)

    def process(self, kwargs: SystemArgs):
        rclpy.spin_once(self.node, timeout_sec=0.1)

    def end(self):
        self.node.destroy_node()
        rclpy.shutdown()