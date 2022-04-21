from simulator.typehints.dict_types import SystemArgs
from simulator.typehints.component_types import EVENT, ERROR
from simulator.typehints.ros_types import RosActionServer, RosService

import logging

from simpy import Environment

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer

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
        self.services = []
    
    def create_action_server(self, service: RosActionServer):
        """
        Creates an action server to the node of the RosControl
        """
        self.services.append(service)
        action_server = ActionServer(self.node,
                                    service.get_service_type(),
                                    service.get_name(),
                                    execute_callback=service.get_result_callback(),
                                    handle_accepted_callback=service.get_handle_accepted_goal_callback())
        return action_server

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
            for service in self.services:
                service.process()
            yield sleep(self.scan_interval)

    def end(self):
        self.node.destroy_node()
        rclpy.shutdown()
        self.logger.info("RosControl ended")