from simulator.typehints.dict_types import SystemArgs
from simulator.typehints.component_types import EVENT, ERROR
from simulator.typehints.ros_types import RosService

import logging

from simpy import Environment

import rclpy
from rclpy.node import Node

class RosControlNode(Node):

    def __init__(self):
        super().__init__('hmrsim')
        self.logger = logging.getLogger(__name__)

class RosControlPlugin(object):
    """
    This object deals with the Ros integration with HMRSim
    """

    def __init__(self, scan_interval: float):
        super().__init__()
        self.logger = logging.getLogger(__name__)
        rclpy.init()
        self.logger.info("Initialized rclpy")
        self.node = RosControlNode()
        self.scan_interval = scan_interval

    def create_subscription(self, service: RosService):
        """
        Creates a new subscription to the node of the RosControl
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
        while True:
            env: Environment = kwargs.get('ENV', None)
            sleep = env.timeout
            rclpy.spin_once(self.node, timeout_sec=0.1)
            yield sleep(self.scan_interval)

    def end(self):
        self.node.destroy_node()
        rclpy.shutdown()
        self.logger.info("RosControl ended")