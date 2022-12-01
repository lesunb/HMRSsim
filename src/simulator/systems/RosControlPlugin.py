from simulator.typehints.dict_types import SystemArgs
from simulator.typehints.component_types import EVENT, ERROR
from simulator.typehints.ros_types import RosActionServer
from simulator.typehints.ros_types import RosTopicServer

import logging

from simpy import Environment

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer

from std_msgs.msg import String

class RosControlNode(Node):
    """
    Ros node of the simulation
    """

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
        self.logger.info("Initialized rclpy.")
        self.node = RosControlNode()
        self.scan_interval = scan_interval
        self.services = []
    
    def create_action_server(self, service: RosActionServer):
        """
        Creates an action server to the node of the RosControl with the service provided.
        Also adds the service to the services used in this plugin.
        """
        self.services.append(service)
        action_server = ActionServer(self.node,
                                    service.get_service_type(),
                                    service.get_name(),
                                    execute_callback=service.get_execute_callback(),
                                    goal_callback=service.get_goal_callback(),
                                    handle_accepted_callback=service.get_handle_accepted_goal_callback(),
                                    cancel_callback=service.get_cancel_callback())
        return action_server

    def create_topic_server(self, service: RosTopicServer):
        """
        Creates a topic server to the node of the RosControl with the service provided.
        Also adds the service to the services used in this plugin.
        """
        self.services.append(service)
        self.node.create_subscription(String, service.get_name(), service.get_listener_callback(), 10)

    def process(self, kwargs: SystemArgs):
        """
        This will spin ros nodes once and then run the processes methods in the services list
        """
        while True:
            env: Environment = kwargs.get('ENV', None)
            sleep = env.timeout
            rclpy.spin_once(self.node, timeout_sec=0.1)

            # notifying the services
            for service in self.services:
                service.process()
            yield sleep(self.scan_interval)

    def end(self):
        self.node.destroy_node()
        rclpy.shutdown()
        self.logger.info("RosControl ended.")
